//! Synchronous, verified Linux hidraw access for USB `2c30:1050`.
//!
//! Requests are private fixed messages. No raw descriptor or arbitrary-write
//! interface is exposed. A connection listens for 500 ms before deciding whether
//! the complete 15-step startup is needed. Dropping it sends no stop command.
//!
//! ```no_run
//! use std::time::Duration;
//! use vrg9080_protocol::{DeviceSession, Event};
//!
//! # fn main() -> Result<(), vrg9080_protocol::TransportError> {
//! let mut device = DeviceSession::connect()?;
//! device.request_current_proximity()?;
//! if let Some(Event::Imu(frame)) = device.next_event(Duration::from_secs(1))? {
//!     println!("acceleration in g: {:?}", frame.acceleration_g);
//! }
//! # Ok(())
//! # }
//! ```

use std::{
    collections::VecDeque,
    error::Error,
    fmt, io,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

use crate::codec::{Event, ParseError, REPORT_SIZE, parse_report};

mod linux;
mod protocol;
#[cfg(test)]
mod tests;

pub use linux::discover_devices;

const PASSIVE_WINDOW: Duration = Duration::from_millis(500);
const WRITE_TIMEOUT: Duration = Duration::from_secs(1);
const MAX_PENDING_EVENTS: usize = 256;

/// One of two fixed startup procedures; neither accepts commands or payloads.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum StartupMode {
    /// The documented 15-step procedure (the default).
    #[default]
    Full,
    /// Only the first, fixed stream-start request, for explicit hardware testing.
    ///
    /// Cold-start stability and asynchronous proximity updates must be verified
    /// on hardware before relying on this mode. A cold-start trial returned
    /// proximity updates but no IMU for 30 seconds. It does not set sampling
    /// rates or query initial proximity. No automatic fallback or retry is performed.
    StreamOnly,
    /// Send the first two fixed requests, for explicit cold-start comparison.
    ///
    /// Two cold-start trials produced sustained IMU updates, but asynchronous
    /// proximity updates were inconsistent. This does not set sampling rates or
    /// query initial proximity. `Full` remains the default.
    FirstTwo,
}

/// Metadata of a rejected input, without any payload bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RejectedReport {
    /// Validation failure.
    pub error: ParseError,
    /// Unvalidated byte 1, if present.
    pub flags_version: Option<u8>,
    /// Unvalidated type/command byte 2, if present.
    pub report_type: Option<u8>,
    /// Unvalidated length byte 3, if present.
    pub protocol_length: Option<u8>,
}

impl fmt::Display for RejectedReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.error)?;
        if let (Some(flags), Some(kind), Some(length)) =
            (self.flags_version, self.report_type, self.protocol_length)
        {
            write!(
                f,
                " (flags/version={flags:#04x}, type={kind:#04x}, length={length})"
            )?;
        }
        Ok(())
    }
}

/// A matching sysfs device; its identity is checked again when it is opened.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceInfo {
    path: PathBuf,
}

impl DeviceInfo {
    /// Verified node path, normally `/dev/hidrawN`.
    pub fn path(&self) -> &Path {
        &self.path
    }
}

/// Result of establishing a connection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConnectionInfo {
    /// False when an IMU frame arrived during the initial passive window.
    pub startup_performed: bool,
    /// Selected procedure. No startup requests were sent if `startup_performed` is false.
    pub startup_mode: StartupMode,
}

/// Counters since opening the session, including the connection phase.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Statistics {
    /// Number of valid IMU reports decoded, including discarded buffered samples.
    pub imu_reports: u64,
    /// Number of valid asynchronous status reports decoded.
    pub status_reports: u64,
    /// Number of requested acknowledgements successfully sent.
    pub acknowledgements_sent: u64,
    /// Invalid reports consumed. During connection these are skipped.
    pub invalid_reports: u64,
    /// Oldest IMU samples discarded to bound the connection buffer to 256 events.
    pub dropped_imu_reports: u64,
    /// Most recently rejected input's error and envelope metadata, never payload.
    pub last_rejected_report: Option<RejectedReport>,
}

/// Transport failures. Errors never carry incoming command-response payloads.
#[derive(Debug)]
#[non_exhaustive]
pub enum TransportError {
    /// No USB device with the supported VID/PID was found.
    NoDevice,
    /// Explicit node selection is needed; discovery never picks by path order.
    MultipleDevices(Vec<PathBuf>),
    /// Only direct `/dev/hidrawN` node paths are accepted.
    InvalidPath(PathBuf),
    /// Sysfs or the opened descriptor identifies a different device.
    IdentityMismatch(PathBuf),
    /// An operating-system error, with operation and node context.
    Io {
        /// Operation that failed.
        operation: &'static str,
        /// Path involved.
        path: PathBuf,
        /// Underlying OS error.
        source: io::Error,
    },
    /// The device disconnected or returned an end-of-file indication.
    Disconnected,
    /// An incoming report was consumed but rejected; reading may continue.
    InvalidReport(ParseError),
    /// A startup response did not arrive within its fixed deadline.
    StartupTimeout {
        /// One-based startup step.
        step: usize,
        /// Number of malformed reports consumed while waiting at this step.
        invalid_reports: u64,
        /// Last malformed report at this step, if any; no payload is retained.
        last_rejected_report: Option<RejectedReport>,
        /// Valid IMU reports received since opening, including the passive window.
        imu_reports: u64,
        /// Valid status reports received since opening, including the passive window.
        status_reports: u64,
        /// Requested acknowledgements sent since opening.
        acknowledgements_sent: u64,
    },
    /// A matching startup command returned a nonzero status.
    CommandFailed {
        /// One-based startup step.
        step: usize,
        /// Received command ID.
        command_id: u8,
        /// Nonzero status code.
        status: u8,
    },
    /// An I/O failure occurred while performing a specific startup step.
    StartupTransport {
        /// One-based startup step.
        step: usize,
        /// Failure of the fixed request, response read or requested acknowledgement.
        source: Box<TransportError>,
    },
    /// A requested status acknowledgement could not be sent.
    AcknowledgementFailed(Box<TransportError>),
    /// A fixed write could not complete within its deadline.
    WriteTimeout,
    /// A write accepted fewer than 65 bytes. The remainder is never sent.
    ShortWrite {
        /// Number of bytes accepted by the OS.
        actual: usize,
    },
    /// More than 256 non-IMU events arrived during connection.
    EventBufferFull,
    /// The requested timeout cannot be represented by the monotonic clock.
    InvalidTimeout,
}

impl fmt::Display for TransportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoDevice => write!(f, "no USB VRG9080 (2c30:1050) found in /sys/class/hidraw"),
            Self::MultipleDevices(paths) => write!(
                f,
                "multiple matching nodes found ({paths:?}); select one explicitly"
            ),
            Self::InvalidPath(path) => {
                write!(f, "{} is not a direct /dev/hidrawN path", path.display())
            }
            Self::IdentityMismatch(path) => write!(
                f,
                "{} is not USB 2c30:1050; refusing access",
                path.display()
            ),
            Self::Io {
                operation,
                path,
                source,
            } => {
                write!(f, "{operation} {}: {source}", path.display())?;
                if source.kind() == io::ErrorKind::PermissionDenied {
                    write!(
                        f,
                        "; ask the system administrator for read/write access to this node"
                    )?;
                } else if source.raw_os_error() == Some(libc::EBUSY) {
                    write!(f, "; close the other reader or driver before trying again")?;
                }
                Ok(())
            }
            Self::Disconnected => write!(f, "hidraw device disconnected"),
            Self::InvalidReport(error) => write!(f, "invalid incoming report: {error}"),
            Self::StartupTimeout {
                step,
                invalid_reports,
                last_rejected_report,
                imu_reports,
                status_reports,
                acknowledgements_sent,
            } => {
                write!(
                    f,
                    "startup step {step} timed out; invalid_reports={invalid_reports}, total_imu={imu_reports}, total_status={status_reports}, acks_sent={acknowledgements_sent}; no automatic retry was attempted"
                )?;
                if let Some(rejected) = last_rejected_report {
                    write!(f, "; last rejected report: {rejected}")?;
                }
                Ok(())
            }
            Self::CommandFailed {
                step,
                command_id,
                status,
            } => write!(
                f,
                "startup step {step} (command {command_id:#04x}) failed with status {status:#04x}"
            ),
            Self::StartupTransport { step, source } => {
                write!(f, "startup step {step}: {source}")
            }
            Self::AcknowledgementFailed(source) => {
                write!(f, "requested status acknowledgement: {source}")
            }
            Self::WriteTimeout => write!(f, "fixed HID write timed out"),
            Self::ShortWrite { actual } => write!(
                f,
                "HID write accepted {actual} bytes instead of 65; request was not retried"
            ),
            Self::EventBufferFull => write!(f, "connection event buffer is full of non-IMU events"),
            Self::InvalidTimeout => write!(f, "timeout exceeds the monotonic clock range"),
        }
    }
}

impl Error for TransportError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            Self::InvalidReport(error) => Some(error),
            Self::StartupTransport { source, .. } | Self::AcknowledgementFailed(source) => {
                Some(source.as_ref())
            }
            _ => None,
        }
    }
}

/// An open, initialized synchronous session. Use one reader per physical device.
///
/// Connection may change transient streaming state and set the fixed sampling
/// rate pair `(1000, 200)`. Closing only releases the file descriptor.
pub struct DeviceSession {
    core: SessionCore<linux::Hidraw>,
    device: DeviceInfo,
    connection: ConnectionInfo,
}

impl DeviceSession {
    /// Discover and connect to the sole matching device.
    ///
    /// Returns `MultipleDevices` if more than one node matches; use [`Self::open`]
    /// in that case. Startup is never automatically retried on failure.
    pub fn connect() -> Result<Self, TransportError> {
        Self::connect_with_startup(StartupMode::Full)
    }

    /// Connect using an explicitly selected fixed startup procedure.
    ///
    /// Both procedures skip startup when valid IMU data is already arriving.
    /// [`StartupMode::StreamOnly`] is an unverified cold-start candidate.
    pub fn connect_with_startup(mode: StartupMode) -> Result<Self, TransportError> {
        let devices = discover_devices()?;
        match devices.as_slice() {
            [] => Err(TransportError::NoDevice),
            [device] => Self::open_with_startup(device.path(), mode),
            _ => Err(TransportError::MultipleDevices(
                devices.into_iter().map(|d| d.path).collect(),
            )),
        }
    }

    /// Verify an explicit `/dev/hidrawN` path, open it and initialize if needed.
    ///
    /// Identity is verified using sysfs before opening and using a read-only
    /// identity ioctl on the descriptor before any report is read or written.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, TransportError> {
        Self::open_with_startup(path, StartupMode::Full)
    }

    /// Verify and open a node with an explicitly selected fixed startup procedure.
    pub fn open_with_startup(
        path: impl AsRef<Path>,
        mode: StartupMode,
    ) -> Result<Self, TransportError> {
        let path = path.as_ref();
        let io = linux::Hidraw::open(path)?;
        let mut core = SessionCore::new(io);
        let connection = core.initialize(mode)?;
        Ok(Self {
            core,
            device: DeviceInfo {
                path: path.to_owned(),
            },
            connection,
        })
    }

    /// Selected node, without exposing its descriptor.
    pub fn device(&self) -> &DeviceInfo {
        &self.device
    }

    /// Whether startup was needed and which fixed procedure was selected.
    pub fn connection_info(&self) -> ConnectionInfo {
        self.connection
    }

    /// Current counters, including reports processed during connection.
    pub fn statistics(&self) -> Statistics {
        self.core.statistics
    }

    /// Send the fixed, payload-free current-proximity request once.
    ///
    /// Receive its [`Event::CurrentProximity`] response via [`Self::next_event`].
    /// This does not wait for the response or schedule repeated queries. Requests
    /// have no transaction identifier; consume one response before querying again.
    pub fn request_current_proximity(&mut self) -> Result<(), TransportError> {
        self.core
            .io
            .write_fixed(&protocol::CURRENT_PROXIMITY, deadline(WRITE_TIMEOUT)?)
    }

    /// Receive the next event, or `None` when the timeout expires.
    ///
    /// A zero timeout checks queued/available data without waiting. Buffered
    /// connection events are returned first; during connection, oldest IMU frames
    /// may be discarded with a counter to bound memory. Non-IMU events are retained.
    /// Invalid reports return [`TransportError::InvalidReport`] and are never
    /// acknowledged. Valid status acknowledgements are sent before returning.
    pub fn next_event(&mut self, timeout: Duration) -> Result<Option<Event>, TransportError> {
        if let Some(event) = self.core.pending.pop_front() {
            return Ok(Some(event));
        }
        self.core.receive(deadline(timeout)?)
    }
}

fn deadline(timeout: Duration) -> Result<Instant, TransportError> {
    Instant::now()
        .checked_add(timeout)
        .ok_or(TransportError::InvalidTimeout)
}

// These types and methods stay private: even test transports cannot be injected
// through the public API to turn the fixed operations into arbitrary writes.
struct Incoming {
    bytes: [u8; REPORT_SIZE],
    len: usize,
}

trait ReportIo {
    fn read_report(&mut self, deadline: Instant) -> Result<Option<Incoming>, TransportError>;
    fn write_fixed(&mut self, report: &[u8; 65], deadline: Instant) -> Result<(), TransportError>;
}

struct SessionCore<T> {
    io: T,
    pending: VecDeque<Event>,
    statistics: Statistics,
}

impl<T: ReportIo> SessionCore<T> {
    fn new(io: T) -> Self {
        Self {
            io,
            pending: VecDeque::new(),
            statistics: Statistics::default(),
        }
    }

    fn receive(&mut self, deadline: Instant) -> Result<Option<Event>, TransportError> {
        let Some(raw) = self.io.read_report(deadline)? else {
            return Ok(None);
        };
        let parsed = if raw.len == REPORT_SIZE {
            parse_report(&raw.bytes)
        } else {
            Err(ParseError::WrongInputSize { actual: raw.len })
        };
        let event = parsed.map_err(|error| {
            self.statistics.invalid_reports += 1;
            let bytes = &raw.bytes[..raw.len.min(REPORT_SIZE)];
            self.statistics.last_rejected_report = Some(RejectedReport {
                error,
                flags_version: bytes.get(1).copied(),
                report_type: bytes.get(2).copied(),
                protocol_length: bytes.get(3).copied(),
            });
            TransportError::InvalidReport(error)
        })?;
        if let Event::Imu(_) = event {
            self.statistics.imu_reports += 1;
        }
        if let Event::Status(status) = event {
            self.statistics.status_reports += 1;
            if status.acknowledgement_requested {
                self.io
                    .write_fixed(&protocol::acknowledgement(&status), deadline)
                    .map_err(|error| TransportError::AcknowledgementFailed(Box::new(error)))?;
                self.statistics.acknowledgements_sent += 1;
            }
        }
        Ok(Some(event))
    }

    fn enqueue(&mut self, event: Event) -> Result<(), TransportError> {
        if self.pending.len() == MAX_PENDING_EVENTS {
            if let Some(index) = self
                .pending
                .iter()
                .position(|event| matches!(event, Event::Imu(_)))
            {
                self.pending.remove(index);
                self.statistics.dropped_imu_reports += 1;
            } else if matches!(event, Event::Imu(_)) {
                self.statistics.dropped_imu_reports += 1;
                return Ok(());
            } else {
                return Err(TransportError::EventBufferFull);
            }
        }
        self.pending.push_back(event);
        Ok(())
    }

    fn initialize(&mut self, mode: StartupMode) -> Result<ConnectionInfo, TransportError> {
        let until = deadline(PASSIVE_WINDOW)?;
        while Instant::now() < until {
            match self.receive(until) {
                Ok(Some(event)) => {
                    let active = matches!(event, Event::Imu(_));
                    self.enqueue(event)?;
                    if active {
                        return Ok(ConnectionInfo {
                            startup_performed: false,
                            startup_mode: mode,
                        });
                    }
                }
                Ok(None) => break,
                Err(TransportError::InvalidReport(_)) => continue,
                Err(error) => return Err(error),
            }
        }

        let steps = match mode {
            StartupMode::Full => &protocol::STARTUP[..],
            StartupMode::StreamOnly => &protocol::STARTUP[..1],
            StartupMode::FirstTwo => &protocol::STARTUP[..2],
        };
        for (index, step) in steps.iter().enumerate() {
            self.io
                .write_fixed(&step.report, deadline(WRITE_TIMEOUT)?)
                .map_err(|error| TransportError::StartupTransport {
                    step: index + 1,
                    source: Box::new(error),
                })?;
            let until = deadline(step.timeout)?;
            let invalid_before_step = self.statistics.invalid_reports;
            loop {
                if Instant::now() >= until {
                    return Err(self.startup_timeout(index + 1, invalid_before_step));
                }
                let event = match self.receive(until) {
                    Ok(Some(event)) => event,
                    Ok(None) => return Err(self.startup_timeout(index + 1, invalid_before_step)),
                    Err(TransportError::InvalidReport(_)) => continue,
                    Err(error) => {
                        return Err(TransportError::StartupTransport {
                            step: index + 1,
                            source: Box::new(error),
                        });
                    }
                };
                let status = match event {
                    Event::StartupResponse(response) if response.command_id == step.report[3] => {
                        Some(response.status)
                    }
                    Event::CurrentProximity(response) if step.report[3] == 0x4a => {
                        Some(response.status)
                    }
                    _ => None,
                };
                if let Some(status) = status {
                    if status != 0 {
                        return Err(TransportError::CommandFailed {
                            step: index + 1,
                            command_id: step.report[3],
                            status,
                        });
                    }
                }
                self.enqueue(event)?;
                if status.is_some() {
                    break;
                }
            }
        }
        Ok(ConnectionInfo {
            startup_performed: true,
            startup_mode: mode,
        })
    }

    fn startup_timeout(&self, step: usize, invalid_before_step: u64) -> TransportError {
        let invalid_reports = self.statistics.invalid_reports - invalid_before_step;
        TransportError::StartupTimeout {
            step,
            invalid_reports,
            last_rejected_report: self
                .statistics
                .last_rejected_report
                .filter(|_| invalid_reports > 0),
            imu_reports: self.statistics.imu_reports,
            status_reports: self.statistics.status_reports,
            acknowledgements_sent: self.statistics.acknowledgements_sent,
        }
    }
}
