//! Hardware-free tests using only locally generated protocol inputs.

mod common;

use common::*;
use vrg9080_protocol::{
    Event, ImuChannel, ParseError, parse_imu_report, parse_report, parse_status_report,
};

#[test]
fn synthetic_conformance_vector() {
    let raw = imu();
    assert_eq!(raw[44], 0x86);
    let frame = parse_imu_report(&raw).unwrap();
    assert_eq!((frame.header0, frame.flags, frame.version), (0, 0, 1));
    assert_eq!(frame.acceleration_g, [1.0, 2.0, 3.0]);
    assert_eq!(frame.angular_velocity_deg_s, [90.0, -45.0, 0.0]);
    assert_eq!(frame.magnetic_field_raw, [4.0, 5.0, 6.0]);
    assert_eq!(frame.timestamp_ticks, 1234);
    assert_eq!(frame.temperature_tenths_c, 365);
    assert_eq!(frame.temperature_c(), 36.5);
    let radians = frame.angular_velocity_rad_s();
    assert!((radians[0] - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
    assert!((radians[1] + std::f32::consts::FRAC_PI_4).abs() < 1e-6);
    assert_eq!(radians[2], 0.0);
    assert_eq!(parse_report(&raw), Ok(Event::Imu(frame)));
}

#[test]
fn size_version_type_and_length_errors_are_distinct() {
    for size in [0, 4, 63, 65, 128] {
        assert_eq!(
            parse_imu_report(&vec![0; size]),
            Err(ParseError::WrongInputSize { actual: size })
        );
    }
    for version in [0, 2, 15] {
        let mut raw = imu();
        raw[1] = version;
        checksum(&mut raw);
        assert_eq!(
            parse_imu_report(&raw),
            Err(ParseError::UnsupportedVersion { version })
        );
    }
    let mut raw = imu();
    raw[2] = 0x0b;
    checksum(&mut raw);
    assert_eq!(
        parse_imu_report(&raw),
        Err(ParseError::WrongReportType {
            expected: 10,
            actual: 11
        })
    );
    raw[2] = 0x0a;
    raw[3] = 43;
    checksum(&mut raw);
    assert_eq!(
        parse_imu_report(&raw),
        Err(ParseError::WrongProtocolLength {
            expected: 44,
            actual: 43
        })
    );
    for length in [0, 3, 64, 255] {
        raw[3] = length;
        assert_eq!(
            parse_report(&raw),
            Err(ParseError::InvalidProtocolLength { actual: length })
        );
    }
}

#[test]
fn checksum_checked_and_padding_ignored() {
    let mut raw = imu();
    raw[5] ^= 0x01;
    assert!(matches!(
        parse_imu_report(&raw),
        Err(ParseError::ChecksumMismatch { .. })
    ));
    let mut raw = imu();
    raw[45..].fill(0xff);
    assert_eq!(parse_imu_report(&raw), parse_imu_report(&imu()));
}

#[test]
fn preserves_all_high_flags_and_header() {
    for flags in (0..=0xf0).step_by(16) {
        let mut raw = imu();
        raw[0] = 0x72;
        raw[1] = flags | 1;
        checksum(&mut raw);
        let frame = parse_imu_report(&raw).unwrap();
        assert_eq!(
            (frame.header0, frame.flags, frame.version),
            (0x72, flags, 1)
        );
    }
}

#[test]
fn validates_every_float_channel_and_boundary() {
    for (index, offset) in (4..40).step_by(4).enumerate() {
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 1.1e9, -1.1e9] {
            let mut raw = imu();
            raw[offset..offset + 4].copy_from_slice(&invalid.to_le_bytes());
            checksum(&mut raw);
            let channel = [
                ImuChannel::Acceleration,
                ImuChannel::AngularVelocity,
                ImuChannel::MagneticField,
            ][index / 3];
            assert_eq!(
                parse_imu_report(&raw),
                Err(ParseError::InvalidFloat {
                    channel,
                    axis: index % 3
                })
            );
        }
        for valid in [1e9_f32, -1e9, 0.0, -0.0] {
            let mut raw = imu();
            raw[offset..offset + 4].copy_from_slice(&valid.to_le_bytes());
            checksum(&mut raw);
            assert!(parse_imu_report(&raw).is_ok());
        }
    }
}

#[test]
fn unsigned_little_endian_fields_do_not_track_wraparound() {
    let mut raw = imu();
    raw[40..44].copy_from_slice(&[0xfe, 0xff, 0x80, 0xab]);
    checksum(&mut raw);
    let frame = parse_imu_report(&raw).unwrap();
    assert_eq!(frame.timestamp_ticks, 65534);
    assert_eq!(frame.temperature_tenths_c, 0xab80);
    raw[40..42].copy_from_slice(&[1, 0]);
    checksum(&mut raw);
    assert_eq!(parse_imu_report(&raw).unwrap().timestamp_ticks, 1);
}

#[test]
fn all_status_masks_select_only_their_fields() {
    for mask in 0..=255 {
        let parsed = parse_status_report(&status(mask)).unwrap();
        assert_eq!(parsed.mask, mask);
        assert_eq!(parsed.proximity, (mask & 1 != 0).then_some(1));
        assert_eq!(parsed.status_a, (mask & 2 != 0).then_some(30));
        assert_eq!(parsed.status_b, (mask & 4 != 0).then_some(31));
        assert_eq!(parsed.event, (mask & 8 != 0).then_some(2));
        assert_eq!(parsed.acknowledgement_requested, mask & 0x80 != 0);
        assert_eq!(parse_report(&status(mask)), Ok(Event::Status(parsed)));
    }
}

#[test]
fn published_status_and_response_vectors() {
    let vectors: &[&[u8]] = &[
        &[0, 1, 0x0b, 9, 1, 1, 0x1e, 0x1e, 2, 0x55],
        &[0, 1, 0x0b, 9, 0x80, 0, 0x1e, 0x1e, 2, 0xd3],
        &[0xaa, 0x11, 0x8c, 5, 0, 0x4c],
        &[0xaa, 0x11, 0x4a, 6, 1, 0, 0x0c],
    ];
    for prefix in vectors {
        let mut raw = [0; 64];
        raw[..prefix.len()].copy_from_slice(prefix);
        assert!(parse_report(&raw).is_ok());
    }
}

#[test]
fn startup_success_failure_and_payload_redaction() {
    let commands = [
        0x8c, 0x8d, 0x80, 1, 0x7f, 2, 0x40, 0x47, 0x86, 0x4e, 0x4f, 0x81,
    ];
    for command in commands {
        for status in [0, 7, 255] {
            let data = b"SYNTHETIC_IDENTIFICATION_DO_NOT_EXPOSE";
            let event = parse_report(&response(command, status, data)).unwrap();
            let Event::StartupResponse(parsed) = event else {
                panic!("wrong event")
            };
            assert_eq!((parsed.command_id, parsed.status), (command, status));
            // Distinct identification payloads must yield identical public values,
            // including Debug. No raw bytes survive parsing.
            assert_eq!(
                event,
                parse_report(&response(command, status, b"other data")).unwrap()
            );
            assert!(!format!("{event:?}").contains("SYNTHETIC"));
        }
    }
}

#[test]
fn proximity_response_value_exists_only_on_success() {
    for status in [0, 1, 255] {
        let Event::CurrentProximity(parsed) = parse_report(&response(0x4a, status, &[42])).unwrap()
        else {
            panic!("wrong event")
        };
        assert_eq!(parsed.status, status);
        assert_eq!(parsed.proximity, (status == 0).then_some(42));
        assert_eq!(parsed.uninterpreted_data, None);
    }
    for status in [0, 1, 255] {
        let Event::CurrentProximity(parsed) =
            parse_report(&response(0x4a, status, &[42, 91])).unwrap()
        else {
            panic!("wrong event")
        };
        assert_eq!(parsed.status, status);
        assert_eq!(parsed.proximity, None);
        assert_eq!(parsed.uninterpreted_data, (status == 0).then_some([42, 91]));
    }
    for data in [&[][..], &[1, 2, 3][..]] {
        assert!(matches!(
            parse_report(&response(0x4a, 0, data)),
            Err(ParseError::WrongProtocolLength { expected: 6, .. })
        ));
    }
}

#[test]
fn response_flag_precedes_type_and_unknown_data_is_discarded() {
    let Event::UnsupportedResponse(parsed) =
        parse_report(&response(0x0b, 0, &[0x80, 1, 2, 3])).unwrap()
    else {
        panic!("response interpreted as status")
    };
    assert_eq!(parsed.command_id, 0x0b);
    let mut raw = [0; 64];
    raw[..4].copy_from_slice(&[0, 1, 0xf1, 4]);
    checksum(&mut raw);
    assert!(matches!(
        parse_report(&raw),
        Ok(Event::UnsupportedReport {
            report_type: 0xf1,
            ..
        })
    ));
    raw[1] |= 0x10;
    checksum(&mut raw);
    assert_eq!(
        parse_report(&raw),
        Err(ParseError::ResponseTooShort { actual: 4 })
    );
}

#[test]
fn arbitrary_inputs_never_panic() {
    let mut seed = 0x9060_u32;
    for case in 0..10_000 {
        let mut raw = [0; 64];
        for byte in &mut raw {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            *byte = (seed >> 24) as u8;
        }
        if case % 2 == 0 {
            raw[1] = (raw[1] & 0xf0) | 1;
            raw[3] = 4 + raw[3] % 60;
            checksum(&mut raw);
        }
        let _ = parse_report(&raw);
        let _ = parse_imu_report(&raw);
        let _ = parse_status_report(&raw);
    }
}
