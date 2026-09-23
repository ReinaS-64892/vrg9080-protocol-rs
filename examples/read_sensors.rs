//! Print decoded sensor events and the as-yet uninterpreted two-byte proximity response.

#[cfg(target_os = "linux")]
fn main() {
    if let Err(error) = run() {
        eprintln!("Error: {error}");
        std::process::exit(1);
    }
}

#[cfg(target_os = "linux")]
fn run() -> Result<(), Box<dyn std::error::Error>> {
    use std::{
        io::{self, Write},
        path::PathBuf,
        time::{Duration, Instant},
    };
    use vrg9080_protocol::{DeviceSession, Event, StartupMode, TransportError, discover_devices};

    let mut path = None::<PathBuf>;
    let mut seconds = 30_u64;
    let mut imu_every = 100_u64;
    let mut query = false;
    let mut list = false;
    let mut startup = StartupMode::Full;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--device" => path = Some(args.next().ok_or("--device needs /dev/hidrawN")?.into()),
            "--seconds" => {
                seconds = args
                    .next()
                    .ok_or("--seconds needs a positive integer")?
                    .parse()?
            }
            "--imu-every" => {
                imu_every = args
                    .next()
                    .ok_or("--imu-every needs a positive integer")?
                    .parse()?
            }
            "--query-proximity" => query = true,
            "--list" => list = true,
            "--startup" => {
                startup = match args.next().as_deref() {
                    Some("full") => StartupMode::Full,
                    Some("stream-only") => StartupMode::StreamOnly,
                    Some("first-two") => StartupMode::FirstTwo,
                    _ => return Err("--startup requires full, stream-only or first-two".into()),
                };
            }
            "--help" | "-h" => {
                println!(
                    "read_sensors [--list] [--device /dev/hidrawN] [--seconds 30] [--imu-every 100] [--query-proximity] [--startup full|stream-only|first-two]"
                );
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if seconds == 0 || imu_every == 0 {
        return Err("--seconds and --imu-every must be positive".into());
    }
    if list {
        let devices = discover_devices()?;
        if devices.is_empty() {
            println!("No USB VRG9080 (2c30:1050) found.");
        }
        for device in devices {
            println!("{} (USB 2c30:1050)", device.path().display());
        }
        return Ok(());
    }

    let mut session = match path {
        Some(path) => DeviceSession::open_with_startup(path, startup)?,
        None => DeviceSession::connect_with_startup(startup)?,
    };
    eprintln!(
        "connected: {} startup_performed={} startup_mode={:?}",
        session.device().path().display(),
        session.connection_info().startup_performed,
        session.connection_info().startup_mode
    );
    if query {
        session.request_current_proximity()?;
    }
    let end = Instant::now()
        .checked_add(Duration::from_secs(seconds))
        .ok_or("--seconds is too large")?;
    let start_stats = session.statistics();
    let mut imu_count = 0_u64;
    let mut proximity_updates = 0_u64;
    let mut invalid_count = 0_u64;
    let stdout = io::stdout();
    let mut output = io::BufWriter::new(stdout.lock());
    while Instant::now() < end {
        let remaining = end.saturating_duration_since(Instant::now());
        let event = match session.next_event(remaining.min(Duration::from_millis(500))) {
            Ok(Some(event)) => event,
            Ok(None) => continue,
            Err(TransportError::InvalidReport(_)) => {
                invalid_count += 1;
                continue;
            }
            Err(error) => return Err(error.into()),
        };
        match event {
            Event::Imu(frame) => {
                imu_count += 1;
                if imu_count == 1 || imu_count % imu_every == 0 {
                    writeln!(
                        output,
                        "imu ticks={} accel_g={:?} gyro_deg_s={:?} magnetic_raw={:?} temp_c={:.1}",
                        frame.timestamp_ticks,
                        frame.acceleration_g,
                        frame.angular_velocity_deg_s,
                        frame.magnetic_field_raw,
                        frame.temperature_c()
                    )?;
                    output.flush()?;
                }
            }
            Event::Status(status) => {
                if let Some(value) = status.proximity {
                    proximity_updates += 1;
                    writeln!(output, "proximity update={value}")?;
                    output.flush()?;
                }
            }
            Event::CurrentProximity(response) => {
                if let Some(value) = response.proximity {
                    writeln!(output, "proximity current={value}")?;
                } else if let Some(bytes) = response.uninterpreted_data {
                    writeln!(
                        output,
                        "proximity response data={bytes:?} (interpretation unverified)"
                    )?;
                } else {
                    writeln!(
                        output,
                        "proximity query failed: status={:#04x}",
                        response.status
                    )?;
                }
                output.flush()?;
            }
            _ => {}
        }
    }
    output.flush()?;
    let stats = session.statistics();
    eprintln!(
        "finished: imu_events={imu_count} proximity_updates={proximity_updates} invalid_after_connect={invalid_count} imu_during_connect={} status_total={} acks_sent={} invalid_total={} dropped_imu_during_connect={}",
        start_stats.imu_reports,
        stats.status_reports,
        stats.acknowledgements_sent,
        stats.invalid_reports,
        stats.dropped_imu_reports
    );
    if let Some(rejected) = stats.last_rejected_report {
        eprintln!("last rejected report: {rejected}");
    }
    if imu_count == 0 {
        return Err("no IMU events received during the observation window".into());
    }
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("The hidraw example requires Linux; the pure codec is portable.");
    std::process::exit(1);
}
