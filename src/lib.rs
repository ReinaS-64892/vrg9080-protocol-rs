//! Unofficial VRG9080 IMU and proximity report decoding.
//!
//! The [`codec`] is independent of device I/O. On Linux, the default `hidraw`
//! feature adds a synchronous `DeviceSession` without an async runtime.
//! Disable default features to use only the codec.
//!
//! Closing a session only closes its descriptor. The device may keep streaming.
//! Concurrent access by another driver or reader is unsupported.

pub mod codec;

#[cfg(all(target_os = "linux", feature = "hidraw"))]
pub mod transport;

pub use codec::{
    CommandResponse, Event, ImuChannel, ImuFrame, ParseError, ProximityResponse, ReportHeader,
    StatusUpdate, parse_imu_report, parse_report, parse_status_report,
};
#[cfg(all(target_os = "linux", feature = "hidraw"))]
pub use transport::{
    ConnectionInfo, DeviceInfo, DeviceSession, RejectedReport, StartupMode, Statistics,
    TransportError, discover_devices,
};
