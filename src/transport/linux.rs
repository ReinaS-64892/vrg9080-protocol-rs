use std::{
    fs::{self, File, OpenOptions},
    io::{self, Read, Write},
    os::{
        fd::AsRawFd,
        unix::{
            ffi::OsStrExt,
            fs::{FileTypeExt, OpenOptionsExt},
        },
    },
    path::{Path, PathBuf},
    time::Instant,
};

use super::{DeviceInfo, Incoming, ReportIo, TransportError};
use crate::codec::REPORT_SIZE;

const SYSFS_ROOT: &str = "/sys/class/hidraw";
const DEV_ROOT: &str = "/dev";

fn io_error(operation: &'static str, path: &Path, source: io::Error) -> TransportError {
    TransportError::Io {
        operation,
        path: path.to_owned(),
        source,
    }
}

fn valid_node_name(name: &std::ffi::OsStr) -> bool {
    name.as_bytes()
        .strip_prefix(b"hidraw")
        .is_some_and(|suffix| !suffix.is_empty() && suffix.iter().all(u8::is_ascii_digit))
}

fn matching_identity(uevent: &str) -> bool {
    let mut ids = uevent
        .lines()
        .filter_map(|line| line.strip_prefix("HID_ID="));
    let Some(id) = ids.next() else { return false };
    if ids.next().is_some() {
        return false;
    }
    let mut fields = id.split(':');
    let mut next = || {
        fields.next().and_then(|field| {
            if field.is_empty() || !field.bytes().all(|byte| byte.is_ascii_hexdigit()) {
                return None;
            }
            u32::from_str_radix(field, 16).ok()
        })
    };
    let identity = (next(), next(), next());
    identity == (Some(3), Some(0x2c30), Some(0x1050)) && fields.next().is_none()
}

fn identity_at(node: &Path, sysfs: &Path) -> Result<bool, TransportError> {
    let Some(name) = node.file_name().filter(|name| valid_node_name(name)) else {
        return Err(TransportError::InvalidPath(node.to_owned()));
    };
    let uevent = sysfs.join(name).join("device/uevent");
    let text = fs::read_to_string(&uevent)
        .map_err(|error| io_error("read HID identity", &uevent, error))?;
    Ok(matching_identity(&text))
}

fn verify_path(node: &Path, sysfs: &Path, dev: &Path) -> Result<(), TransportError> {
    let Some(name) = node.file_name().filter(|name| valid_node_name(name)) else {
        return Err(TransportError::InvalidPath(node.to_owned()));
    };
    if node.as_os_str() != dev.join(name).as_os_str() {
        return Err(TransportError::InvalidPath(node.to_owned()));
    }
    if !identity_at(node, sysfs)? {
        return Err(TransportError::IdentityMismatch(node.to_owned()));
    }
    Ok(())
}

/// Enumerate only USB VID `0x2c30`, PID `0x1050` nodes using sysfs `HID_ID`.
///
/// Display names and path order are never used to identify a device. This
/// operation does not open hidraw nodes or send requests.
pub fn discover_devices() -> Result<Vec<DeviceInfo>, TransportError> {
    discover_at(Path::new(SYSFS_ROOT), Path::new(DEV_ROOT))
}

fn discover_at(sysfs: &Path, dev: &Path) -> Result<Vec<DeviceInfo>, TransportError> {
    let entries = match fs::read_dir(sysfs) {
        Ok(entries) => entries,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(error) => return Err(io_error("enumerate hidraw nodes", sysfs, error)),
    };
    let mut devices = Vec::new();
    for entry in entries {
        let entry = entry.map_err(|error| io_error("enumerate hidraw node", sysfs, error))?;
        if !valid_node_name(&entry.file_name()) {
            continue;
        }
        let node = dev.join(entry.file_name());
        match identity_at(&node, sysfs) {
            Ok(true) => devices.push(DeviceInfo { path: node }),
            Ok(false) => {}
            // Hot-unplug while walking sysfs is normal.
            Err(TransportError::Io { source, .. }) if source.kind() == io::ErrorKind::NotFound => {}
            Err(error) => return Err(error),
        }
    }
    devices.sort_by(|a, b| a.path.cmp(&b.path));
    Ok(devices)
}

pub(super) struct Hidraw {
    file: File,
    path: PathBuf,
}

impl Hidraw {
    pub(super) fn open(path: &Path) -> Result<Self, TransportError> {
        verify_path(path, Path::new(SYSFS_ROOT), Path::new(DEV_ROOT))?;
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK | libc::O_CLOEXEC | libc::O_NOFOLLOW)
            .open(path)
            .map_err(|error| io_error("open hidraw node", path, error))?;
        if !file
            .metadata()
            .map_err(|error| io_error("stat hidraw node", path, error))?
            .file_type()
            .is_char_device()
        {
            return Err(TransportError::IdentityMismatch(path.to_owned()));
        }
        // Check the actual descriptor as well, so a hotplug/path replacement
        // between sysfs validation and open cannot redirect our fixed writes.
        #[repr(C)]
        #[derive(Default)]
        struct RawInfo {
            bus: u32,
            vendor: i16,
            product: i16,
        }
        let mut info = RawInfo::default();
        // SAFETY: file owns a live fd, and HIDIOCGRAWINFO writes one initialized,
        // correctly aligned RawInfo matching Linux's struct hidraw_devinfo.
        let result = unsafe {
            libc::ioctl(
                file.as_raw_fd(),
                libc::_IOR::<RawInfo>(u32::from(b'H'), 3),
                &mut info,
            )
        };
        if result < 0 {
            return Err(io_error(
                "verify opened HID identity",
                path,
                io::Error::last_os_error(),
            ));
        }
        if (info.bus, info.vendor as u16, info.product as u16) != (3, 0x2c30, 0x1050) {
            return Err(TransportError::IdentityMismatch(path.to_owned()));
        }
        Ok(Self {
            file,
            path: path.to_owned(),
        })
    }

    fn poll(&self, events: libc::c_short, deadline: Instant) -> Result<bool, TransportError> {
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            // Round up sub-millisecond waits, and avoid overflowing poll's i32.
            let millis =
                remaining.as_millis() + u128::from(remaining.subsec_nanos() % 1_000_000 != 0);
            let timeout = millis.min(i32::MAX as u128) as i32;
            let mut fd = libc::pollfd {
                fd: self.file.as_raw_fd(),
                events,
                revents: 0,
            };
            // SAFETY: fd points to one initialized pollfd for the duration of
            // this call; the owned File remains alive and exclusively borrowed.
            let count = unsafe { libc::poll(&mut fd, 1, timeout) };
            if count < 0 {
                let error = io::Error::last_os_error();
                if error.kind() == io::ErrorKind::Interrupted {
                    if Instant::now() >= deadline {
                        return Ok(false);
                    }
                    continue;
                }
                return Err(io_error("poll hidraw node", &self.path, error));
            }
            if count == 0 {
                if Instant::now() >= deadline {
                    return Ok(false);
                }
                continue;
            }
            if fd.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 {
                return Err(TransportError::Disconnected);
            }
            if fd.revents & events != 0 {
                return Ok(true);
            }
            if Instant::now() >= deadline {
                return Ok(false);
            }
        }
    }
}

impl ReportIo for Hidraw {
    fn read_report(&mut self, deadline: Instant) -> Result<Option<Incoming>, TransportError> {
        loop {
            if !self.poll(libc::POLLIN, deadline)? {
                return Ok(None);
            }
            let mut bytes = [0; REPORT_SIZE];
            match self.file.read(&mut bytes) {
                Ok(0) => return Err(TransportError::Disconnected),
                Ok(len) => return Ok(Some(Incoming { bytes, len })),
                Err(error)
                    if matches!(
                        error.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted
                    ) =>
                {
                    if Instant::now() >= deadline {
                        return Ok(None);
                    }
                }
                Err(error) => return Err(io_error("read hidraw report", &self.path, error)),
            }
        }
    }

    fn write_fixed(&mut self, report: &[u8; 65], deadline: Instant) -> Result<(), TransportError> {
        loop {
            match self.file.write(report) {
                Ok(65) => return Ok(()),
                Ok(actual) => return Err(TransportError::ShortWrite { actual }),
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                    if Instant::now() >= deadline || !self.poll(libc::POLLOUT, deadline)? {
                        return Err(TransportError::WriteTimeout);
                    }
                }
                Err(error) if error.kind() == io::ErrorKind::Interrupted => {
                    if Instant::now() >= deadline {
                        return Err(TransportError::WriteTimeout);
                    }
                }
                Err(error) => {
                    return Err(io_error("write fixed hidraw request", &self.path, error));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    fn io_pair() -> (Hidraw, std::os::unix::net::UnixDatagram) {
        let (reader, peer) = std::os::unix::net::UnixDatagram::pair().unwrap();
        reader.set_nonblocking(true).unwrap();
        peer.set_nonblocking(true).unwrap();
        let fd: std::os::fd::OwnedFd = reader.into();
        (
            Hidraw {
                file: File::from(fd),
                path: PathBuf::from("<test socket>"),
            },
            peer,
        )
    }

    #[test]
    fn poll_reads_separate_packets_and_zero_timeout_does_not_wait() {
        let (mut hid, peer) = io_pair();
        assert!(hid.read_report(Instant::now()).unwrap().is_none());
        peer.send(&[1; 63]).unwrap();
        peer.send(&[2; 64]).unwrap();
        let first = hid.read_report(Instant::now()).unwrap().unwrap();
        assert_eq!(first.len, 63);
        assert_eq!(&first.bytes[..63], &[1; 63]);
        let second = hid.read_report(Instant::now()).unwrap().unwrap();
        assert_eq!(second.len, 64);
        assert_eq!(second.bytes, [2; 64]);
        assert!(hid.read_report(Instant::now()).unwrap().is_none());
    }

    #[test]
    fn os_write_is_one_complete_report_and_drop_sends_nothing() {
        let (mut hid, peer) = io_pair();
        let request = super::super::protocol::CURRENT_PROXIMITY;
        hid.write_fixed(&request, Instant::now()).unwrap();
        let mut received = [0; 100];
        assert_eq!(peer.recv(&mut received).unwrap(), 65);
        assert_eq!(&received[..65], &request);
        drop(hid);
        assert_eq!(
            peer.recv(&mut received).unwrap_err().kind(),
            io::ErrorKind::WouldBlock
        );
    }

    #[test]
    fn full_output_buffer_respects_write_deadline() {
        let (mut hid, _peer) = io_pair();
        let request = super::super::protocol::CURRENT_PROXIMITY;
        // Fill the local datagram buffer without an external network or sleeps.
        loop {
            match hid.file.write(&request) {
                Ok(65) => {}
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => break,
                result => panic!("unexpected fill result: {result:?}"),
            }
        }
        assert!(matches!(
            hid.write_fixed(&request, Instant::now()),
            Err(TransportError::WriteTimeout)
        ));
    }

    #[test]
    fn identity_requires_exact_hex_usb_vendor_product() {
        for text in [
            "HID_ID=0003:00002C30:00001050",
            "HID_NAME=untrusted\nHID_ID=3:2c30:1050\n",
        ] {
            assert!(matching_identity(text));
        }
        for text in [
            "",
            "HID_ID=3:2c30:1051",
            "HID_ID=3:2c31:1050",
            "HID_ID=5:2c30:1050",
            "HID_ID=3:2c30:1050:0",
            "HID_ID=+3:2c30:1050",
            "HID_NAME=VRG9080",
            "HID_ID=3:2c30:1050\nHID_ID=3:2c30:1050",
        ] {
            assert!(!matching_identity(text), "{text}");
        }
    }

    struct Fixture(PathBuf);
    impl Fixture {
        fn new() -> Self {
            static COUNTER: AtomicU64 = AtomicU64::new(0);
            let path = std::env::temp_dir().join(format!(
                "vrg9080-sysfs-{}-{}",
                std::process::id(),
                COUNTER.fetch_add(1, Ordering::Relaxed)
            ));
            fs::create_dir(&path).unwrap();
            Self(path)
        }
        fn node(&self, name: &str, id: &str) {
            let dir = self.0.join(name).join("device");
            fs::create_dir_all(&dir).unwrap();
            fs::write(dir.join("uevent"), id).unwrap();
        }
    }
    impl Drop for Fixture {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn discovery_and_explicit_path_refuse_mismatched_sysfs_identity() {
        let f = Fixture::new();
        f.node("hidraw9", "HID_ID=0003:00002C30:00001050\n");
        f.node(
            "hidraw0",
            "HID_ID=0003:00001234:00001050\nHID_NAME=VRG9080\n",
        );
        f.node("hidraw1", "HID_ID=0003:00002C30:00001051\n");
        f.node("hidraw2", "HID_ID=0005:00002C30:00001050\n");
        let devices = discover_at(&f.0, Path::new("/dev")).unwrap();
        assert_eq!(
            devices,
            [DeviceInfo {
                path: PathBuf::from("/dev/hidraw9")
            }]
        );
        assert!(verify_path(Path::new("/dev/hidraw9"), &f.0, Path::new("/dev")).is_ok());
        for node in ["/dev/hidraw0", "/dev/hidraw1", "/dev/hidraw2"] {
            assert!(matches!(
                verify_path(Path::new(node), &f.0, Path::new("/dev")),
                Err(TransportError::IdentityMismatch(_))
            ));
        }
        for node in [
            "hidraw9",
            "/tmp/hidraw9",
            "/dev/../dev/hidraw9",
            "/dev/hidraw",
            "/dev/hidrawx",
        ] {
            assert!(matches!(
                verify_path(Path::new(node), &f.0, Path::new("/dev")),
                Err(TransportError::InvalidPath(_))
            ));
        }
    }
}
