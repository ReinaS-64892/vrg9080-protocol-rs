use super::*;

#[path = "../../tests/common/mod.rs"]
mod common;
use common::{checksum, imu, response, status};

enum Action {
    Read([u8; 64], usize),
    Timeout,
    Write([u8; 65]),
    FailWrite([u8; 65]),
}

#[derive(Default)]
struct Script {
    actions: VecDeque<Action>,
    writes: Vec<[u8; 65]>,
    read_timeouts: Vec<Duration>,
}

impl Script {
    fn read(&mut self, report: [u8; 64]) {
        self.actions.push_back(Action::Read(report, 64));
    }
    fn write(&mut self, report: [u8; 65]) {
        self.actions.push_back(Action::Write(report));
    }
    fn timeout(&mut self) {
        self.actions.push_back(Action::Timeout);
    }
    fn complete_step(&mut self, index: usize) {
        let step = &protocol::STARTUP[index];
        self.write(step.report);
        let command = step.report[3];
        self.read(response(
            command,
            0,
            if command == 0x4a {
                &[1]
            } else {
                b"synthetic identification"
            },
        ));
    }
}

impl ReportIo for Script {
    fn read_report(&mut self, deadline: Instant) -> Result<Option<Incoming>, TransportError> {
        self.read_timeouts
            .push(deadline.saturating_duration_since(Instant::now()));
        match self.actions.pop_front().expect("unexpected extra read") {
            Action::Read(bytes, len) => Ok(Some(Incoming { bytes, len })),
            Action::Timeout => Ok(None),
            Action::Write(_) | Action::FailWrite(_) => panic!("read before required write"),
        }
    }
    fn write_fixed(&mut self, report: &[u8; 65], _: Instant) -> Result<(), TransportError> {
        match self
            .actions
            .pop_front()
            .expect("unexpected extra write/retry")
        {
            Action::Write(expected) => assert_eq!(*report, expected),
            Action::FailWrite(expected) => {
                assert_eq!(*report, expected);
                return Err(TransportError::Io {
                    operation: "write fixed hidraw request",
                    path: PathBuf::from("/dev/hidraw0"),
                    source: io::Error::from_raw_os_error(libc::ETIMEDOUT),
                });
            }
            _ => panic!("write before matching response"),
        }
        self.writes.push(*report);
        Ok(())
    }
}

fn padded(prefix: &[u8]) -> [u8; 65] {
    let mut bytes = [0; 65];
    bytes[..prefix.len()].copy_from_slice(prefix);
    bytes
}

#[test]
fn os_write_timeout_identifies_step_without_retry() {
    for failing_index in 0..15 {
        let mut script = Script::default();
        script.timeout();
        for index in 0..failing_index {
            script.complete_step(index);
        }
        script
            .actions
            .push_back(Action::FailWrite(protocol::STARTUP[failing_index].report));
        let mut core = SessionCore::new(script);
        let error = core.initialize(StartupMode::Full).unwrap_err();
        assert!(
            matches!(error, TransportError::StartupTransport { step, .. } if step == failing_index + 1)
        );
        assert!(
            error
                .to_string()
                .contains(&format!("startup step {}:", failing_index + 1))
        );
        assert!(core.io.actions.is_empty());
        assert_eq!(core.io.writes.len(), failing_index);
    }
}

#[test]
fn passive_acknowledgement_failure_is_named() {
    let mut script = Script::default();
    script.read(status(0x80));
    script.actions.push_back(Action::FailWrite(padded(&[
        0, 0xaa, 0x11, 0x0b, 5, 1, 0xcc,
    ])));
    let mut core = SessionCore::new(script);
    let error = core.initialize(StartupMode::Full).unwrap_err();
    assert!(matches!(error, TransportError::AcknowledgementFailed(_)));
    assert!(
        error
            .to_string()
            .contains("requested status acknowledgement")
    );
    assert!(core.io.actions.is_empty());
}

#[test]
fn every_fixed_write_matches_the_wire_contract_including_padding() {
    let prefixes: &[&[u8]] = &[
        &[0, 0xaa, 1, 0x8c, 5, 0, 0x3c],
        &[0, 0xaa, 1, 0x8d, 5, 3, 0x40],
        &[0, 0xaa, 1, 0x80, 6, 0, 0, 0x31],
        &[0, 0xaa, 1, 1, 4, 0xb0],
        &[0, 0xaa, 1, 0x7f, 5, 1, 0x30],
        &[0, 0xaa, 1, 0x7f, 5, 0, 0x2f],
        &[0, 0xaa, 1, 0x7f, 5, 2, 0x31],
        &[0, 0xaa, 1, 2, 4, 0xb1],
        &[0, 0xaa, 1, 0x40, 4, 0xef],
        &[0, 0xaa, 1, 0x47, 4, 0xf6],
        &[0, 0xaa, 1, 0x86, 6, 1, 1, 0x39],
        &[0, 0xaa, 1, 0x4a, 4, 0xf9],
        &[0, 0xaa, 1, 0x4e, 4, 0xfd],
        &[0, 0xaa, 1, 0x4f, 4, 0xfe],
        &[0, 0xaa, 1, 0x81, 8, 0xe8, 3, 0xc8, 0, 0xe7],
    ];
    for (index, (step, expected)) in protocol::STARTUP.iter().zip(prefixes).enumerate() {
        assert_eq!(step.report, padded(expected));
        let length = usize::from(step.report[4]);
        let sum = step.report[1..=length]
            .iter()
            .fold(0_u8, |a, b| a.wrapping_add(*b));
        assert_eq!(step.report[length + 1], sum);
        assert_eq!(
            step.timeout,
            Duration::from_millis(if index == 1 { 3500 } else { 1000 })
        );
    }
    assert_eq!(
        protocol::CURRENT_PROXIMITY,
        padded(&[0, 0xaa, 1, 0x4a, 4, 0xf9])
    );
}

#[test]
fn active_imu_skips_every_startup_write() {
    let mut script = Script::default();
    script.read(status(1));
    script.read(imu());
    let mut core = SessionCore::new(script);
    assert_eq!(
        core.initialize(StartupMode::Full).unwrap(),
        ConnectionInfo {
            startup_performed: false,
            startup_mode: StartupMode::Full,
        }
    );
    assert!(core.io.writes.is_empty());
    assert!(core.io.actions.is_empty());
    assert!(matches!(core.pending[0], Event::Status(_)));
    assert!(matches!(core.pending[1], Event::Imu(_)));
    assert!(core.io.read_timeouts[0] <= PASSIVE_WINDOW);
}

#[test]
fn stream_only_is_exactly_one_fixed_request_with_interleaved_status_ack() {
    let mut script = Script::default();
    script.timeout();
    script.write(protocol::STARTUP[0].report);
    script.read(imu());
    script.read(status(0x81));
    script.write(padded(&[0, 0xaa, 0x11, 0x0b, 5, 1, 0xcc]));
    script.read(response(0x8c, 0, &[]));
    let mut core = SessionCore::new(script);
    let info = core.initialize(StartupMode::StreamOnly).unwrap();
    assert!(info.startup_performed);
    assert_eq!(info.startup_mode, StartupMode::StreamOnly);
    assert_eq!(core.io.writes.len(), 2);
    assert!(core.io.actions.is_empty());
    assert!(
        core.pending
            .iter()
            .any(|event| matches!(event, Event::Imu(_)))
    );
    assert!(
        core.pending
            .iter()
            .any(|event| matches!(event, Event::Status(s) if s.proximity == Some(1)))
    );
}

#[test]
fn stream_only_still_skips_when_active_and_aborts_on_error_or_timeout() {
    let mut script = Script::default();
    script.read(imu());
    let mut core = SessionCore::new(script);
    assert!(
        !core
            .initialize(StartupMode::StreamOnly)
            .unwrap()
            .startup_performed
    );
    assert!(core.io.writes.is_empty());
    for timeout in [true, false] {
        let mut script = Script::default();
        script.timeout();
        script.write(protocol::STARTUP[0].report);
        if timeout {
            script.timeout();
        } else {
            script.read(response(0x8c, 4, &[]));
        }
        let mut core = SessionCore::new(script);
        assert!(core.initialize(StartupMode::StreamOnly).is_err());
        assert_eq!(core.io.writes.len(), 1);
        assert!(core.io.actions.is_empty());
    }
}

#[test]
fn first_two_sends_only_the_first_two_fixed_requests() {
    let mut script = Script::default();
    script.timeout();
    script.complete_step(0);
    script.complete_step(1);
    let mut core = SessionCore::new(script);
    let info = core.initialize(StartupMode::FirstTwo).unwrap();
    assert!(info.startup_performed);
    assert_eq!(info.startup_mode, StartupMode::FirstTwo);
    assert_eq!(
        core.io.writes,
        protocol::STARTUP[..2]
            .iter()
            .map(|step| step.report)
            .collect::<Vec<_>>()
    );
    assert!(core.io.actions.is_empty());
    assert_eq!(core.statistics.imu_reports, 0);
}

#[test]
fn timeout_reports_rejected_proximity_envelope_without_payload() {
    let mut script = Script::default();
    script.timeout();
    for index in 0..11 {
        script.complete_step(index);
    }
    script.write(protocol::CURRENT_PROXIMITY);
    // A response with no proximity data must stay rejected, but diagnostics
    // should distinguish it from receiving no response at all.
    script.read(response(0x4a, 0, &[]));
    script.timeout();
    let mut core = SessionCore::new(script);
    let error = core.initialize(StartupMode::Full).unwrap_err();
    let TransportError::StartupTimeout {
        step,
        invalid_reports,
        last_rejected_report,
        ..
    } = error
    else {
        panic!("wrong error: {error}");
    };
    assert_eq!(step, 12);
    assert_eq!(invalid_reports, 1);
    let rejected = last_rejected_report.unwrap();
    assert_eq!(rejected.report_type, Some(0x4a));
    assert_eq!(rejected.flags_version, Some(0x11));
    assert_eq!(rejected.protocol_length, Some(5));
    assert_eq!(
        rejected.error,
        ParseError::WrongProtocolLength {
            expected: 6,
            actual: 5
        }
    );
    assert!(core.io.actions.is_empty());
}

#[test]
fn length_seven_proximity_response_completes_full_startup_without_inventing_value() {
    let mut script = Script::default();
    script.timeout();
    for (index, step) in protocol::STARTUP.iter().enumerate() {
        script.write(step.report);
        script.read(response(
            step.report[3],
            0,
            if index == 11 { &[7, 99] } else { &[] },
        ));
    }
    let mut core = SessionCore::new(script);
    let info = core.initialize(StartupMode::Full).unwrap();
    assert!(info.startup_performed);
    assert_eq!(core.io.writes.len(), 15);
    assert_eq!(core.statistics.invalid_reports, 0);
    assert!(core.pending.iter().any(|event| {
        matches!(event, Event::CurrentProximity(p) if p.status == 0 && p.proximity.is_none() && p.uninterpreted_data == Some([7, 99]))
    }));
    assert!(core.io.actions.is_empty());
}

#[test]
fn early_stream_does_not_interrupt_full_startup_and_step12_emits_proximity() {
    let mut script = Script::default();
    script.timeout();
    for (index, step) in protocol::STARTUP.iter().enumerate() {
        script.write(step.report);
        // A mismatched response must not advance the startup state machine.
        script.read(response(0xf0, 9, b"ignored"));
        script.read(imu());
        script.read(status(0x81));
        script.write(padded(&[0, 0xaa, 0x11, 0x0b, 5, 1, 0xcc]));
        let data: &[u8] = if index == 11 {
            &[7]
        } else {
            b"PRIVATE_SYNTHETIC_DATA"
        };
        script.read(response(step.report[3], 0, data));
    }
    let mut core = SessionCore::new(script);
    assert_eq!(
        core.initialize(StartupMode::Full).unwrap(),
        ConnectionInfo {
            startup_performed: true,
            startup_mode: StartupMode::Full,
        }
    );
    assert_eq!(core.io.writes.len(), 30);
    assert!(core.io.actions.is_empty());
    assert_eq!(core.statistics.imu_reports, 15);
    assert!(
        core.pending
            .iter()
            .any(|event| matches!(event, Event::CurrentProximity(p) if p.proximity == Some(7)))
    );
    assert!(!format!("{:?}", core.pending).contains("PRIVATE_SYNTHETIC_DATA"));
}

#[test]
fn every_startup_failure_aborts_once_without_retry() {
    for failing_index in 0..15 {
        for timeout in [false, true] {
            let mut script = Script::default();
            script.timeout();
            for index in 0..failing_index {
                script.complete_step(index);
            }
            let step = &protocol::STARTUP[failing_index];
            script.write(step.report);
            if timeout {
                script.timeout();
            } else {
                script.read(response(
                    step.report[3],
                    7,
                    if failing_index == 11 { &[1] } else { &[] },
                ));
            }
            let mut core = SessionCore::new(script);
            let error = core.initialize(StartupMode::Full).unwrap_err();
            if timeout {
                assert!(
                    matches!(error, TransportError::StartupTimeout { step, .. } if step == failing_index + 1)
                );
            } else {
                assert!(
                    matches!(error, TransportError::CommandFailed { step, status: 7, .. } if step == failing_index + 1)
                );
            }
            assert!(core.io.actions.is_empty());
            assert_eq!(core.io.writes.len(), failing_index + 1);
        }
    }
}

#[test]
fn only_fully_valid_status_requests_are_acknowledged() {
    let mut script = Script::default();
    script.read(status(1));
    script.read(status(0x80));
    script.write(padded(&[0, 0xaa, 0x11, 0x0b, 5, 1, 0xcc]));
    let mut flagged = status(0x80);
    flagged[1] = 0x61;
    checksum(&mut flagged);
    script.read(flagged);
    script.write(padded(&[0, 0xaa, 0x71, 0x0b, 5, 1, 0x2c]));
    let mut corrupt = status(0x81);
    corrupt[9] ^= 1;
    script.read(corrupt);
    let mut bad_version = status(0x81);
    bad_version[1] = 2;
    checksum(&mut bad_version);
    script.read(bad_version);
    let mut wrong_length = status(0x81);
    wrong_length[3] = 8;
    checksum(&mut wrong_length);
    script.read(wrong_length);
    script.actions.push_back(Action::Read(status(0x81), 63));
    script.read(response(0x0b, 0, &[0x80, 1, 2, 3]));
    let mut core = SessionCore::new(script);
    let until = deadline(Duration::from_secs(1)).unwrap();
    assert!(
        matches!(core.receive(until).unwrap(), Some(Event::Status(s)) if s.proximity == Some(1))
    );
    for _ in 0..2 {
        assert!(
            matches!(core.receive(until).unwrap(), Some(Event::Status(s)) if s.proximity.is_none())
        );
    }
    for _ in 0..4 {
        assert!(matches!(
            core.receive(until),
            Err(TransportError::InvalidReport(_))
        ));
    }
    assert!(matches!(
        core.receive(until).unwrap(),
        Some(Event::UnsupportedResponse(_))
    ));
    assert!(core.io.actions.is_empty());
    assert_eq!(core.io.writes.len(), 2);
    assert_eq!(core.statistics.invalid_reports, 4);
}

#[test]
fn passive_window_acknowledges_and_skips_malformed_imu() {
    let mut script = Script::default();
    let mut malformed = imu();
    malformed[44] ^= 1;
    script.read(malformed);
    script.read(status(0x80));
    script.write(padded(&[0, 0xaa, 0x11, 0x0b, 5, 1, 0xcc]));
    script.read(imu());
    let mut core = SessionCore::new(script);
    assert!(
        !core
            .initialize(StartupMode::Full)
            .unwrap()
            .startup_performed
    );
    assert_eq!(core.statistics.invalid_reports, 1);
    assert_eq!(core.io.writes.len(), 1);
}

#[test]
fn invalid_matching_response_does_not_advance_startup() {
    let mut script = Script::default();
    script.timeout();
    script.write(protocol::STARTUP[0].report);
    let mut malformed = response(0x8c, 0, &[]);
    malformed[5] ^= 1;
    script.read(malformed);
    script.timeout();
    let mut core = SessionCore::new(script);
    assert!(matches!(
        core.initialize(StartupMode::Full),
        Err(TransportError::StartupTimeout { step: 1, .. })
    ));
    assert_eq!(core.statistics.invalid_reports, 1);
    assert_eq!(core.io.writes.len(), 1);
}

#[test]
fn bounded_startup_buffer_preserves_status_and_proximity() {
    let mut core = SessionCore::new(Script::default());
    core.enqueue(parse_report(&status(1)).unwrap()).unwrap();
    core.enqueue(parse_report(&response(0x4a, 0, &[9])).unwrap())
        .unwrap();
    for _ in 0..1000 {
        core.enqueue(parse_report(&imu()).unwrap()).unwrap();
    }
    assert_eq!(core.pending.len(), MAX_PENDING_EVENTS);
    assert_eq!(core.statistics.dropped_imu_reports, 746);
    assert!(matches!(core.pending[0], Event::Status(_)));
    assert!(matches!(core.pending[1], Event::CurrentProximity(_)));
    let mut core = SessionCore::new(Script::default());
    for _ in 0..MAX_PENDING_EVENTS {
        core.enqueue(parse_report(&status(1)).unwrap()).unwrap();
    }
    assert!(matches!(
        core.enqueue(parse_report(&status(1)).unwrap()),
        Err(TransportError::EventBufferFull)
    ));
}
