//! Explicitly opted-in checks requiring a physical VRG9080.

#![cfg(all(target_os = "linux", feature = "hidraw"))]

use std::time::{Duration, Instant};
use vrg9080_protocol::{DeviceSession, Event, TransportError};

#[test]
#[ignore = "requires an explicitly connected USB 2c30:1050 device and exclusive access"]
fn receives_imu_and_explicit_proximity_response() {
    let mut session = match std::env::var_os("VRG9080_HIDRAW") {
        Some(path) => DeviceSession::open(std::path::PathBuf::from(path)),
        None => DeviceSession::connect(),
    }
    .expect("connect to VRG9080");
    let until = Instant::now() + Duration::from_secs(5);
    let mut samples = 0;
    // Consume the buffered startup query before issuing our explicit query, so
    // its old response cannot falsely satisfy the second half of this test.
    let mut initial_proximity_seen = !session.connection_info().startup_performed;
    while (samples < 100 || !initial_proximity_seen) && Instant::now() < until {
        match session.next_event(until.saturating_duration_since(Instant::now())) {
            Ok(Some(Event::Imu(_))) => samples += 1,
            Ok(Some(Event::CurrentProximity(response))) => {
                assert_eq!(response.status, 0);
                assert!(response.proximity.is_some() || response.uninterpreted_data.is_some());
                initial_proximity_seen = true;
            }
            Err(TransportError::InvalidReport(_)) => {}
            result => {
                result.expect("read sensors");
            }
        }
    }
    assert!(
        samples >= 100,
        "expected 100 IMU samples within five seconds"
    );
    assert!(
        initial_proximity_seen,
        "startup proximity event was not delivered"
    );
    session
        .request_current_proximity()
        .expect("send one proximity query");
    let until = Instant::now() + Duration::from_secs(2);
    while Instant::now() < until {
        match session.next_event(until.saturating_duration_since(Instant::now())) {
            Ok(Some(Event::CurrentProximity(response))) => {
                assert_eq!(response.status, 0);
                assert!(response.proximity.is_some() || response.uninterpreted_data.is_some());
                return;
            }
            Err(TransportError::InvalidReport(_)) => {}
            result => {
                result.expect("receive proximity response");
            }
        }
    }
    panic!("no current-proximity response within two seconds");
}
