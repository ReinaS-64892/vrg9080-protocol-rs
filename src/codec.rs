//! Pure, allocation-free decoding of 64-byte incoming reports.
//!
//! Input excludes the HID report-ID slot. Padding after the checksum is ignored.
//! No function in this module performs I/O or produces outgoing commands.

use std::{error::Error, fmt};

/// Incoming report size, without a report-ID byte.
pub const REPORT_SIZE: usize = 64;

/// Common metadata, with no meaning assigned to the first byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReportHeader {
    /// Uninterpreted first byte.
    pub header0: u8,
    /// High nibble of byte 1, kept in its original bit positions.
    pub flags: u8,
    /// Low nibble of byte 1 (currently 1).
    pub version: u8,
}

/// One sample in the original wire axis order, without calibration or fusion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuFrame {
    /// Uninterpreted first byte.
    pub header0: u8,
    /// High nibble of byte 1, kept in its original bit positions.
    pub flags: u8,
    /// Protocol version.
    pub version: u8,
    /// Acceleration in units of g.
    pub acceleration_g: [f32; 3],
    /// Angular velocity in degrees per second.
    pub angular_velocity_deg_s: [f32; 3],
    /// Magnetic channels; physical units are unspecified.
    pub magnetic_field_raw: [f32; 3],
    /// Wrapping counter with a nominal period of 100 microseconds per tick.
    pub timestamp_ticks: u16,
    /// Unsigned temperature in tenths of a degree Celsius.
    pub temperature_tenths_c: u16,
}

impl ImuFrame {
    /// Temperature in degrees Celsius.
    pub fn temperature_c(&self) -> f32 {
        f32::from(self.temperature_tenths_c) / 10.0
    }

    /// Angular velocity converted to radians per second, without changing axes.
    pub fn angular_velocity_rad_s(&self) -> [f32; 3] {
        self.angular_velocity_deg_s.map(f32::to_radians)
    }
}

/// An asynchronous status update. Absent mask bits yield `None`, never stale data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusUpdate {
    /// Uninterpreted first byte.
    pub header0: u8,
    /// High nibble of byte 1, kept in its original bit positions.
    pub flags: u8,
    /// Protocol version.
    pub version: u8,
    /// Entire update mask, including unknown bits 4 through 6.
    pub mask: u8,
    /// Raw proximity value when mask bit 0 is set; its scale is unspecified.
    pub proximity: Option<u8>,
    /// Uninterpreted status A, selected by mask bit 1.
    pub status_a: Option<u8>,
    /// Uninterpreted status B, selected by mask bit 2.
    pub status_b: Option<u8>,
    /// Uninterpreted event value, selected by mask bit 3.
    pub event: Option<u8>,
    /// Whether mask bit 7 requests the fixed acknowledgement.
    pub acknowledgement_requested: bool,
}

/// Startup response metadata. Identification and other response data are discarded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommandResponse {
    /// Common metadata.
    pub header: ReportHeader,
    /// Received command ID; this is not an outgoing command API.
    pub command_id: u8,
    /// Zero indicates success.
    pub status: u8,
}

/// Response to the fixed current-proximity request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProximityResponse {
    /// Common metadata.
    pub header: ReportHeader,
    /// Zero indicates success.
    pub status: u8,
    /// Present only for a successful length-6 response.
    pub proximity: Option<u8>,
    /// Data from a successful length-7 response, in wire order.
    /// Its meaning has not been established; no proximity value is inferred.
    pub uninterpreted_data: Option<[u8; 2]>,
}

/// A validated report. No variant contains raw command-response payloads.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum Event {
    /// IMU sample.
    Imu(ImuFrame),
    /// Status update, possibly including proximity.
    Status(StatusUpdate),
    /// One of the startup commands other than the proximity query.
    StartupResponse(CommandResponse),
    /// Current-proximity response, including command status on failure.
    CurrentProximity(ProximityResponse),
    /// Unrecognized command response, with its data discarded.
    UnsupportedResponse(CommandResponse),
    /// Valid envelope with an unrecognized report type; payload is discarded.
    UnsupportedReport {
        /// Common metadata.
        header: ReportHeader,
        /// Unrecognized type byte.
        report_type: u8,
        /// Checksum offset in the report.
        protocol_length: u8,
    },
}

/// Vector containing an invalid floating-point value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuChannel {
    /// Acceleration vector.
    Acceleration,
    /// Angular velocity vector.
    AngularVelocity,
    /// Magnetic vector.
    MagneticField,
}

/// Validation failure; no partially decoded report is returned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ParseError {
    /// Input length was not 64.
    WrongInputSize {
        /// Actual input byte count.
        actual: usize,
    },
    /// Version is not supported.
    UnsupportedVersion {
        /// Received low-nibble version.
        version: u8,
    },
    /// Type differs from the specific parser's expected type.
    WrongReportType {
        /// Required type byte.
        expected: u8,
        /// Received type byte.
        actual: u8,
    },
    /// Checksum position is outside the range 4 through 63.
    InvalidProtocolLength {
        /// Received length/checksum offset.
        actual: u8,
    },
    /// Type-specific protocol length is incorrect.
    WrongProtocolLength {
        /// Required length/checksum offset.
        expected: u8,
        /// Received length/checksum offset.
        actual: u8,
    },
    /// A command response lacks its final status byte.
    ResponseTooShort {
        /// Received length/checksum offset.
        actual: u8,
    },
    /// SUM8 does not match the byte at the protocol length offset.
    ChecksumMismatch {
        /// Calculated SUM8.
        expected: u8,
        /// Received checksum byte.
        actual: u8,
    },
    /// A channel is non-finite or its magnitude exceeds 1e9.
    InvalidFloat {
        /// Vector containing the invalid value.
        channel: ImuChannel,
        /// Index in the original wire order (0, 1 or 2).
        axis: usize,
    },
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongInputSize { actual } => write!(f, "expected 64 report bytes, got {actual}"),
            Self::UnsupportedVersion { version } => {
                write!(f, "unsupported protocol version {version}")
            }
            Self::WrongReportType { expected, actual } => {
                write!(f, "expected report type {expected:#04x}, got {actual:#04x}")
            }
            Self::InvalidProtocolLength { actual } => {
                write!(f, "checksum offset {actual} is outside 4..=63")
            }
            Self::WrongProtocolLength { expected, actual } => {
                write!(f, "expected protocol length {expected}, got {actual}")
            }
            Self::ResponseTooShort { actual } => {
                write!(f, "response length {actual} has no status byte")
            }
            Self::ChecksumMismatch { expected, actual } => {
                write!(f, "expected checksum {expected:#04x}, got {actual:#04x}")
            }
            Self::InvalidFloat { channel, axis } => {
                write!(f, "invalid {channel:?} channel at axis {axis}")
            }
        }
    }
}

impl Error for ParseError {}

pub(crate) fn sum8(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0_u8, |sum, byte| sum.wrapping_add(*byte))
}

fn envelope(bytes: &[u8]) -> Result<ReportHeader, ParseError> {
    if bytes.len() != REPORT_SIZE {
        return Err(ParseError::WrongInputSize {
            actual: bytes.len(),
        });
    }
    let length = bytes[3];
    if !(4..=63).contains(&length) {
        return Err(ParseError::InvalidProtocolLength { actual: length });
    }
    let version = bytes[1] & 0x0f;
    if version != 1 {
        return Err(ParseError::UnsupportedVersion { version });
    }
    let expected = sum8(&bytes[..usize::from(length)]);
    let actual = bytes[usize::from(length)];
    if expected != actual {
        return Err(ParseError::ChecksumMismatch { expected, actual });
    }
    Ok(ReportHeader {
        header0: bytes[0],
        flags: bytes[1] & 0xf0,
        version,
    })
}

fn require_layout(bytes: &[u8], report_type: u8, length: u8) -> Result<(), ParseError> {
    if bytes[2] != report_type {
        return Err(ParseError::WrongReportType {
            expected: report_type,
            actual: bytes[2],
        });
    }
    if bytes[3] != length {
        return Err(ParseError::WrongProtocolLength {
            expected: length,
            actual: bytes[3],
        });
    }
    Ok(())
}

/// Decode an IMU report, validating envelope, checksum, layout and all nine floats.
pub fn parse_imu_report(bytes: &[u8]) -> Result<ImuFrame, ParseError> {
    let header = envelope(bytes)?;
    imu(bytes, header)
}

fn imu(bytes: &[u8], header: ReportHeader) -> Result<ImuFrame, ParseError> {
    require_layout(bytes, 0x0a, 44)?;
    fn vector(bytes: &[u8], start: usize, channel: ImuChannel) -> Result<[f32; 3], ParseError> {
        let mut result = [0.0; 3];
        for (axis, value) in result.iter_mut().enumerate() {
            let i = start + axis * 4;
            *value = f32::from_le_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]]);
            if !value.is_finite() || value.abs() > 1e9 {
                return Err(ParseError::InvalidFloat { channel, axis });
            }
        }
        Ok(result)
    }
    Ok(ImuFrame {
        header0: header.header0,
        flags: header.flags,
        version: header.version,
        acceleration_g: vector(bytes, 4, ImuChannel::Acceleration)?,
        angular_velocity_deg_s: vector(bytes, 16, ImuChannel::AngularVelocity)?,
        magnetic_field_raw: vector(bytes, 28, ImuChannel::MagneticField)?,
        timestamp_ticks: u16::from_le_bytes([bytes[40], bytes[41]]),
        temperature_tenths_c: u16::from_le_bytes([bytes[42], bytes[43]]),
    })
}

/// Decode a status report; only mask-selected fields are returned.
pub fn parse_status_report(bytes: &[u8]) -> Result<StatusUpdate, ParseError> {
    let header = envelope(bytes)?;
    status(bytes, header)
}

fn status(bytes: &[u8], header: ReportHeader) -> Result<StatusUpdate, ParseError> {
    require_layout(bytes, 0x0b, 9)?;
    let mask = bytes[4];
    Ok(StatusUpdate {
        header0: header.header0,
        flags: header.flags,
        version: header.version,
        mask,
        proximity: (mask & 1 != 0).then_some(bytes[5]),
        status_a: (mask & 2 != 0).then_some(bytes[6]),
        status_b: (mask & 4 != 0).then_some(bytes[7]),
        event: (mask & 8 != 0).then_some(bytes[8]),
        acknowledgement_requested: mask & 0x80 != 0,
    })
}

/// Decode a report, giving the response flag precedence over its type byte.
///
/// Unknown types retain only metadata. Response payloads other than successful
/// current-proximity response data are never returned, including in errors or `Debug`.
pub fn parse_report(bytes: &[u8]) -> Result<Event, ParseError> {
    let header = envelope(bytes)?;
    if header.flags & 0x10 != 0 {
        let length = bytes[3];
        if length < 5 {
            return Err(ParseError::ResponseTooShort { actual: length });
        }
        let command_id = bytes[2];
        let status = bytes[usize::from(length) - 1];
        if command_id == 0x4a {
            if length == 7 {
                return Ok(Event::CurrentProximity(ProximityResponse {
                    header,
                    status,
                    proximity: None,
                    uninterpreted_data: (status == 0).then_some([bytes[4], bytes[5]]),
                }));
            }
            require_layout(bytes, 0x4a, 6)?;
            return Ok(Event::CurrentProximity(ProximityResponse {
                header,
                status,
                proximity: (status == 0).then_some(bytes[4]),
                uninterpreted_data: None,
            }));
        }
        let response = CommandResponse {
            header,
            command_id,
            status,
        };
        return Ok(match command_id {
            0x8c | 0x8d | 0x80 | 0x01 | 0x7f | 0x02 | 0x40 | 0x47 | 0x86 | 0x4e | 0x4f | 0x81 => {
                Event::StartupResponse(response)
            }
            _ => Event::UnsupportedResponse(response),
        });
    }
    match bytes[2] {
        0x0a => imu(bytes, header).map(Event::Imu),
        0x0b => status(bytes, header).map(Event::Status),
        report_type => Ok(Event::UnsupportedReport {
            header,
            report_type,
            protocol_length: bytes[3],
        }),
    }
}
