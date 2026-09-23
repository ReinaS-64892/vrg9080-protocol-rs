use std::time::Duration;

use crate::codec::{StatusUpdate, sum8};

pub(super) struct StartupStep {
    pub(super) report: [u8; 65],
    pub(super) timeout: Duration,
}

// This helper only pads complete literal wire prefixes. It is never public and
// accepts neither caller-supplied command IDs nor payloads.
const fn padded(prefix: &[u8]) -> [u8; 65] {
    let mut report = [0; 65];
    let mut i = 0;
    while i < prefix.len() {
        report[i] = prefix[i];
        i += 1;
    }
    report
}

pub(super) const CURRENT_PROXIMITY: [u8; 65] = padded(&[0, 0xaa, 1, 0x4a, 4, 0xf9]);

const fn step(prefix: &[u8], millis: u64) -> StartupStep {
    StartupStep {
        report: padded(prefix),
        timeout: Duration::from_millis(millis),
    }
}

pub(super) const STARTUP: [StartupStep; 15] = [
    step(&[0, 0xaa, 1, 0x8c, 5, 0, 0x3c], 1000),
    step(&[0, 0xaa, 1, 0x8d, 5, 3, 0x40], 3500),
    step(&[0, 0xaa, 1, 0x80, 6, 0, 0, 0x31], 1000),
    step(&[0, 0xaa, 1, 0x01, 4, 0xb0], 1000),
    step(&[0, 0xaa, 1, 0x7f, 5, 1, 0x30], 1000),
    step(&[0, 0xaa, 1, 0x7f, 5, 0, 0x2f], 1000),
    step(&[0, 0xaa, 1, 0x7f, 5, 2, 0x31], 1000),
    step(&[0, 0xaa, 1, 0x02, 4, 0xb1], 1000),
    step(&[0, 0xaa, 1, 0x40, 4, 0xef], 1000),
    step(&[0, 0xaa, 1, 0x47, 4, 0xf6], 1000),
    step(&[0, 0xaa, 1, 0x86, 6, 1, 1, 0x39], 1000),
    step(&[0, 0xaa, 1, 0x4a, 4, 0xf9], 1000),
    step(&[0, 0xaa, 1, 0x4e, 4, 0xfd], 1000),
    step(&[0, 0xaa, 1, 0x4f, 4, 0xfe], 1000),
    step(&[0, 0xaa, 1, 0x81, 8, 0xe8, 3, 0xc8, 0, 0xe7], 1000),
];

// Only called with a fully parsed incoming status requesting acknowledgement.
pub(super) fn acknowledgement(status: &StatusUpdate) -> [u8; 65] {
    let mut report = padded(&[0, 0xaa, status.flags | status.version | 0x10, 0x0b, 5, 1]);
    report[6] = sum8(&report[1..6]);
    report
}
