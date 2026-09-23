# Repository notes for agents

## Goal and implementation

- This crate reads VRG9080 sensor values. Keep the library API primarily synchronous and independent of an async runtime. The pure 64-byte report codec works without the Linux `hidraw` feature.
- `src/codec.rs` parses reports; `src/transport/` implements Linux `hidraw`, fixed startup requests, and synchronous event delivery. `examples/read_sensors.rs` is the diagnostic CLI. See `README.md` for the public API, wire behavior, and usage.
- Preserve the device's original axis order and raw or unspecified units. Do not invent calibration, orientation, or a meaning for unknown fields.
- Keep outgoing device writes restricted to the documented fixed startup sequence, explicit fixed proximity query, and required ACK. Do not add arbitrary command or raw write APIs without a specific need and hardware validation.

## Hardware findings and follow-up

- Target USB identity is VID `0x2C30`, PID `0x1050`. Linux `hidraw write()` with a 65-byte buffer is the confirmed sending method. No device is assumed to be attached in this workspace; the user runs physical tests in a separate environment.
- `StartupMode::Full` (15 fixed steps) is the default. Two cold-start 30-second runs produced about 28,000 IMU events and proximity updates alternating `1/0`; an ACK was observed. An earlier run timed out at step 12 because the real `0x4A` response used protocol length 7, which the codec now accepts.
- `StartupMode::StreamOnly` (step 1) is insufficient for IMU: one cold attempt timed out, and another produced proximity updates but zero IMU events over 30 seconds.
- `StartupMode::FirstTwo` (steps 1–2) produced about 28,800–28,900 IMU events in each of two cold-start 30-second runs. Proximity updates were 4 in one run and 0 in the other, despite covering and uncovering the sensor. Keep Full as the default until shortened startup is shown to meet the complete sensor goal reliably.
- The length-7 `0x4A` response returned data `[0, 0]` both with the sensor uncovered and covered while asynchronous proximity updates changed `1/0`. Keep those two bytes uninterpreted; do not expose either as a confirmed current proximity value. Length-6 responses retain the documented one-byte value.
- For cold-start comparisons, ask the user to unplug/replug USB immediately before **each** run, ensure other readers are stopped, and check `startup_performed=true`. A subsequent run without replugging may use the already active stream and report `startup_performed=false`; that does not validate a startup mode. Request only concise `connected:`, `finished:`, proximity lines, and errors unless fuller logs are necessary.

## Verification

- Run `cargo fmt --all -- --check`, `cargo test --offline --all-features`, `cargo test --offline --no-default-features`, and `cargo clippy --offline --all-targets --all-features -- -D warnings` for relevant code changes.
- The opt-in physical test is `cargo test --test hardware -- --ignored --nocapture`; `VRG9080_HIDRAW=/dev/hidrawN` selects a device. Do not run it unless the user has explicitly arranged an attached device and exclusive access.
- Do not turn uncertain hardware observations into protocol facts. Record conditions and results in `README.md` when behavior or startup recommendations change.

## Git and private material

- `Private/imu-parser-implementation-brief.md` is local reference material. `.gitignore` excludes `/Private/`; never stage or commit it, and verify the staged file list before each commit. Do not copy private source material or identifying payloads into committed documentation or tests.
- Use Conventional Commits. When committing, use the user's exact command form:

  ```sh
  git -c user.name='OpenAI Codex' -c user.email='codex@openai.com' -c commit.gpgSign=false commit --author='OpenAI Codex <codex@openai.com>' -m 'type: description'
  ```

- The initial crate is commit `4a04628` (`feat: add synchronous VRG9080 sensor crate`). The license is unset and `publish = false`; do not infer a license or publish the crate without a user decision.
