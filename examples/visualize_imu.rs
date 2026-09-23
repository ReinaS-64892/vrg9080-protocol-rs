//! Display the latest IMU sample as simple axis bars.

#[cfg(target_os = "linux")]
fn main() {
    if let Err(error) = run() {
        eprintln!("Error: {error}");
        std::process::exit(1);
    }
}

#[cfg(target_os = "linux")]
fn run() -> Result<(), Box<dyn std::error::Error>> {
    use minifb::{Key, Window, WindowOptions};
    use std::{
        sync::{
            Arc, Mutex,
            atomic::{AtomicBool, Ordering},
        },
        thread,
        time::Duration,
    };
    use vrg9080_protocol::{DeviceSession, Event, ImuFrame, TransportError};

    const WIDTH: usize = 1200;
    const HEIGHT: usize = 540;
    let latest = Arc::new(Mutex::new((None::<ImuFrame>, None::<u8>)));
    let running = Arc::new(AtomicBool::new(true));
    let reader_latest = Arc::clone(&latest);
    let reader_running = Arc::clone(&running);
    let reader = thread::spawn(move || -> Result<(), String> {
        let mut session = DeviceSession::connect().map_err(|error| error.to_string())?;
        while reader_running.load(Ordering::Relaxed) {
            match session.next_event(Duration::from_millis(100)) {
                Ok(Some(Event::Imu(frame))) => reader_latest.lock().unwrap().0 = Some(frame),
                Ok(Some(Event::Status(status))) => {
                    if let Some(value) = status.proximity {
                        reader_latest.lock().unwrap().1 = Some(value);
                    }
                }
                Ok(Some(_)) | Ok(None) => {}
                Err(TransportError::InvalidReport(_)) => {}
                Err(error) => return Err(error.to_string()),
            }
        }
        Ok(())
    });

    let mut window = Window::new(
        "VRG9080 IMU — Esc to close",
        WIDTH,
        HEIGHT,
        WindowOptions::default(),
    )?;
    window.set_target_fps(30);
    let mut buffer = vec![0_u32; WIDTH * HEIGHT];
    let mut rotation = [0.0_f32; 3];
    let mut previous_ticks = None::<u16>;
    while window.is_open() && !window.is_key_down(Key::Escape) {
        buffer.fill(0x101820);
        text(&mut buffer, WIDTH, 20, 16, "VRG9080  IMU", 0xffffff, 3);
        let (sample, proximity) = *latest.lock().unwrap();
        let proximity_label = proximity
            .map(|value| format!("PROXIMITY {value}"))
            .unwrap_or_else(|| "PROXIMITY --".to_owned());
        text(&mut buffer, WIDTH, 850, 470, &proximity_label, 0xfbbf24, 2);
        if let Some(frame) = sample {
            if let Some(old_ticks) = previous_ticks {
                let elapsed = frame.timestamp_ticks.wrapping_sub(old_ticks) as f32 * 0.0001;
                if elapsed < 1.0 {
                    for axis in 0..3 {
                        rotation[axis] += frame.angular_velocity_deg_s[axis].to_radians() * elapsed;
                    }
                }
            }
            previous_ticks = Some(frame.timestamp_ticks);
            text(&mut buffer, WIDTH, 20, 58, "ACCELERATION (g)", 0xcbd5e1, 2);
            text(
                &mut buffer,
                WIDTH,
                20,
                222,
                "ANGULAR VELOCITY (deg/s)",
                0xcbd5e1,
                2,
            );
            draw_channels(&mut buffer, WIDTH, 94, frame.acceleration_g, 2.0);
            draw_channels(&mut buffer, WIDTH, 258, frame.angular_velocity_deg_s, 500.0);
            text(&mut buffer, WIDTH, 20, 386, "MAGNETIC (raw)", 0xcbd5e1, 2);
            draw_channels(&mut buffer, WIDTH, 422, frame.magnetic_field_raw, 100.0);
            let ticks = format!(
                "TICKS {}   TEMP {:.1} C",
                frame.timestamp_ticks,
                frame.temperature_c()
            );
            text(&mut buffer, WIDTH, 450, 20, &ticks, 0x94a3b8, 2);
            text(
                &mut buffer,
                WIDTH,
                850,
                72,
                "GYRO INTEGRATED BOX",
                0xcbd5e1,
                2,
            );
            draw_box(&mut buffer, WIDTH, 970, 260, rotation);
            text(
                &mut buffer,
                WIDTH,
                850,
                430,
                "RELATIVE VIEW  NO FUSION",
                0x94a3b8,
                2,
            );
        } else {
            text(
                &mut buffer,
                WIDTH,
                20,
                100,
                "Waiting for IMU samples...",
                0xfbbf24,
                2,
            );
        }
        window.update_with_buffer(&buffer, WIDTH, HEIGHT)?;
    }
    running.store(false, Ordering::Relaxed);
    match reader.join() {
        Ok(Ok(())) => Ok(()),
        Ok(Err(error)) => Err(error.into()),
        Err(_) => Err("sensor reader thread panicked".into()),
    }
}

#[cfg(target_os = "linux")]
fn draw_box(buffer: &mut [u32], width: usize, center_x: i32, center_y: i32, rotation: [f32; 3]) {
    let vertices = [
        [-1.0_f32, -1.0, -1.0],
        [1.0, -1.0, -1.0],
        [1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0],
        [-1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
    ];
    let mut projected = [(0_i32, 0_i32); 8];
    for (index, [x, y, z]) in vertices.into_iter().enumerate() {
        let (sx, cx) = rotation[0].sin_cos();
        let (sy, cy) = rotation[1].sin_cos();
        let (sz, cz) = rotation[2].sin_cos();
        let y1 = y * cx - z * sx;
        let z1 = y * sx + z * cx;
        let x2 = x * cy + z1 * sy;
        let z2 = -x * sy + z1 * cy;
        let x3 = x2 * cz - y1 * sz;
        let y3 = x2 * sz + y1 * cz;
        projected[index] = (
            center_x + ((x3 - y3) * 64.0) as i32,
            center_y + ((x3 + y3) * 32.0 - z2 * 64.0) as i32,
        );
    }
    let edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];
    for (a, b) in edges {
        line(buffer, width, projected[a], projected[b], 0x38bdf8);
    }
    let origin = project_point([0.0, 0.0, 0.0], rotation, center_x, center_y);
    for (endpoint, color, label) in [
        ([1.45, 0.0, 0.0], 0xf87171, "X"),
        ([0.0, 1.45, 0.0], 0x4ade80, "Y"),
        ([0.0, 0.0, 1.45], 0x60a5fa, "Z"),
    ] {
        let tip = project_point(endpoint, rotation, center_x, center_y);
        line(buffer, width, origin, tip, color);
        text(
            buffer,
            width,
            tip.0.max(0) as usize,
            tip.1.max(0) as usize,
            label,
            color,
            2,
        );
    }
}

#[cfg(target_os = "linux")]
fn project_point(point: [f32; 3], rotation: [f32; 3], cx: i32, cy: i32) -> (i32, i32) {
    let (sx, cosx) = rotation[0].sin_cos();
    let (sy, cosy) = rotation[1].sin_cos();
    let (sz, cosz) = rotation[2].sin_cos();
    let [x, y, z] = point;
    let y1 = y * cosx - z * sx;
    let z1 = y * sx + z * cosx;
    let x2 = x * cosy + z1 * sy;
    let z2 = -x * sy + z1 * cosy;
    let x3 = x2 * cosz - y1 * sz;
    let y3 = x2 * sz + y1 * cosz;
    (
        cx + ((x3 - y3) * 64.0) as i32,
        cy + ((x3 + y3) * 32.0 - z2 * 64.0) as i32,
    )
}

#[cfg(target_os = "linux")]
fn line(buffer: &mut [u32], width: usize, from: (i32, i32), to: (i32, i32), color: u32) {
    let (mut x0, mut y0) = from;
    let (x1, y1) = to;
    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut error = dx + dy;
    loop {
        if x0 >= 0 && y0 >= 0 && (x0 as usize) < width && (y0 as usize) < buffer.len() / width {
            buffer[y0 as usize * width + x0 as usize] = color;
        }
        if x0 == x1 && y0 == y1 {
            break;
        }
        let twice = 2 * error;
        if twice >= dy {
            error += dy;
            x0 += sx;
        }
        if twice <= dx {
            error += dx;
            y0 += sy;
        }
    }
}

#[cfg(target_os = "linux")]
fn draw_channels(buffer: &mut [u32], width: usize, y: usize, values: [f32; 3], scale: f32) {
    let colors = [0xf87171, 0x4ade80, 0x60a5fa];
    let names = ["X", "Y", "Z"];
    for axis in 0..3 {
        let row = y + axis * 40;
        text(buffer, width, 24, row, names[axis], colors[axis], 2);
        let center = 400_i32;
        let extent = ((values[axis] / scale).clamp(-1.0, 1.0) * 300.0) as i32;
        rect(buffer, width, center as usize, row + 2, 1, 24, 0x475569);
        if extent >= 0 {
            rect(
                buffer,
                width,
                center as usize,
                row + 5,
                extent as usize,
                18,
                colors[axis],
            );
        } else {
            rect(
                buffer,
                width,
                (center + extent) as usize,
                row + 5,
                (-extent) as usize,
                18,
                colors[axis],
            );
        }
        let label = format!("{:+.3}", values[axis]);
        text(buffer, width, 720, row, &label, 0xe2e8f0, 2);
    }
}

#[cfg(target_os = "linux")]
fn rect(buffer: &mut [u32], width: usize, x: usize, y: usize, w: usize, h: usize, color: u32) {
    let height = buffer.len() / width;
    for row in y.min(height)..(y + h).min(height) {
        for col in x.min(width)..(x + w).min(width) {
            buffer[row * width + col] = color;
        }
    }
}

#[cfg(target_os = "linux")]
fn text(
    buffer: &mut [u32],
    width: usize,
    x: usize,
    y: usize,
    value: &str,
    color: u32,
    scale: usize,
) {
    for (index, ch) in value.chars().enumerate() {
        let rows = glyph(ch.to_ascii_uppercase());
        for (gy, bits) in rows.iter().enumerate() {
            for gx in 0..5 {
                if bits & (1 << (4 - gx)) != 0 {
                    rect(
                        buffer,
                        width,
                        x + (index * 6 + gx) * scale,
                        y + gy * scale,
                        scale,
                        scale,
                        color,
                    );
                }
            }
        }
    }
}

#[cfg(target_os = "linux")]
fn glyph(ch: char) -> [u8; 7] {
    match ch {
        'A' => [14, 17, 17, 31, 17, 17, 17],
        'B' => [30, 17, 17, 30, 17, 17, 30],
        'C' => [14, 17, 16, 16, 16, 17, 14],
        'D' => [30, 17, 17, 17, 17, 17, 30],
        'E' => [31, 16, 16, 30, 16, 16, 31],
        'G' => [14, 17, 16, 23, 17, 17, 15],
        'H' => [17, 17, 17, 31, 17, 17, 17],
        'I' => [14, 4, 4, 4, 4, 4, 14],
        'K' => [17, 18, 20, 24, 20, 18, 17],
        'L' => [16, 16, 16, 16, 16, 16, 31],
        'M' => [17, 27, 21, 21, 17, 17, 17],
        'N' => [17, 25, 21, 19, 17, 17, 17],
        'O' => [14, 17, 17, 17, 17, 17, 14],
        'P' => [30, 17, 17, 30, 16, 16, 16],
        'R' => [30, 17, 17, 30, 20, 18, 17],
        'S' => [15, 16, 16, 14, 1, 1, 30],
        'T' => [31, 4, 4, 4, 4, 4, 4],
        'U' => [17, 17, 17, 17, 17, 17, 14],
        'V' => [17, 17, 17, 17, 17, 10, 4],
        'W' => [17, 17, 17, 21, 21, 21, 10],
        'X' => [17, 17, 10, 4, 10, 17, 17],
        'Y' => [17, 17, 10, 4, 4, 4, 4],
        '0' => [14, 17, 19, 21, 25, 17, 14],
        '1' => [4, 12, 4, 4, 4, 4, 14],
        '2' => [14, 17, 1, 2, 4, 8, 31],
        '3' => [30, 1, 1, 14, 1, 1, 30],
        '4' => [2, 6, 10, 18, 31, 2, 2],
        '5' => [31, 16, 16, 30, 1, 1, 30],
        '6' => [14, 16, 16, 30, 17, 17, 14],
        '7' => [31, 1, 2, 4, 8, 8, 8],
        '8' => [14, 17, 17, 14, 17, 17, 14],
        '9' => [14, 17, 17, 15, 1, 1, 14],
        '-' => [0, 0, 0, 31, 0, 0, 0],
        '+' => [0, 4, 4, 31, 4, 4, 0],
        '.' => [0, 0, 0, 0, 0, 12, 12],
        ':' => [0, 12, 12, 0, 12, 12, 0],
        '(' => [2, 4, 8, 8, 8, 4, 2],
        ')' => [8, 4, 2, 2, 2, 4, 8],
        '/' => [1, 2, 2, 4, 8, 8, 16],
        _ => [0; 7],
    }
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("The IMU visualizer requires Linux hidraw.");
    std::process::exit(1);
}
