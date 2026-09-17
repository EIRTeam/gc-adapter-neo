#![cfg_attr(not(feature = "libusb"), no_std)]
//! A library for working with the Nintendo Gamecube controller adapter.
//!
//! **Supports:**
//!
//! * Official Nintendo Gamecube Controller Adapter for Wii U and Switch
//! * Mayflash Gamecube Controller Adapter (in "Wii U/Switch" mode)
//! * Other 3rd party adapters (untested)
//!
//! ## Example
//!
//! ```rust,ignore
//! use gc_adapter_neo::GcAdapter;
//!
//! // get adapter from global context
//! let mut adapter = GcAdapter::from_usb().unwrap().expect("no adapter plugged in");
//!
//! // refresh inputs to ensure they are up to date
//! adapter.refresh_inputs().unwrap();
//!
//! // read and display all controller ports
//! dbg!(adapter.read_controllers().unwrap());
//!
//! // enable rumble for only ports 4
//! adapter.set_rumble([false, false, false, true]).unwrap();
//!
//! std::thread::sleep(std::time::Duration::from_millis(100));
//!
//! // on drop all rumble will be disabled and the USB connection
//! // will be cleaned up
//! let _ = adapter;
//! ```

#[cfg(feature = "libusb")]
pub use rusb;

/// Vendor and Product IDs for adapter
pub mod constants {
    pub const ADAPTER_VID: u16 = 0x057e;
    pub const ADAPTER_PID: u16 = 0x0337;
}

/// Packet parsing code
mod parsing;
pub use parsing::*;

/// Types for represent various axis types
mod axis;
pub use axis::{SignedAxis, StickCalibration, UnsignedAxis};

/// Types/Traits for handling USB connections
mod usb;
pub use usb::*;

/// Errors that can occur while interacting with a [`GcAdapter`].
#[derive(Debug)]
pub enum AdapterError {
    /// A USB-level error occurred (disconnect, timeout, stall, etc).
    Usb(UsbError),
    /// The adapter returned a packet that didn't parse as expected. This can legitimately
    /// happen for a few frames right after the adapter is opened.
    Parse(binread::Error),
    /// A read returned fewer bytes than a full input payload; the data should be discarded and
    /// the read retried.
    ShortRead(usize),
}

impl From<UsbError> for AdapterError {
    fn from(err: UsbError) -> Self {
        AdapterError::Usb(err)
    }
}

impl core::fmt::Display for AdapterError {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        match self {
            AdapterError::Usb(err) => write!(f, "USB error: {:?}", err),
            AdapterError::Parse(err) => write!(f, "failed to parse adapter packet: {:?}", err),
            AdapterError::ShortRead(n) => write!(f, "short read from adapter: {} bytes", n),
        }
    }
}

#[cfg(feature = "libusb")]
impl std::error::Error for AdapterError {}

/// Per-port stick origins used for Melee-style normalization.
#[derive(Debug, Clone, Copy, Default)]
pub struct PortCalibration {
    pub left_stick: StickCalibration,
    pub right_stick: StickCalibration,
}

impl PortCalibration {
    /// Get the normalized left stick coordinates for a controller read from this port.
    pub fn left_stick_coords(&self, controller: &Controller) -> (f32, f32) {
        controller.left_stick.coords_calibrated(&self.left_stick)
    }

    /// Get the normalized right stick coordinates for a controller read from this port.
    pub fn right_stick_coords(&self, controller: &Controller) -> (f32, f32) {
        controller.right_stick.coords_calibrated(&self.right_stick)
    }
}

/// A connection to a gamecube adapter
pub struct GcAdapter<T: AdapterHardware> {
    usb: T,
    was_connected: [bool; 4],
    calibration: [PortCalibration; 4],
}

impl<T: AdapterHardware> GcAdapter<T> {
    /// Set rumble for all 4 ports at once
    pub fn set_rumble(&mut self, ports: [bool; 4]) -> Result<(), UsbError> {
        let payload = [
            0x11,
            ports[0] as u8,
            ports[1] as u8,
            ports[2] as u8,
            ports[3] as u8,
        ];

        self.usb.write_interrupt(&payload[..])?;
        let mut buf = [0u8; 37];
        let n = self.usb.read_interrupt(&mut buf)?;
        if n == buf.len() {
            self.process_report(buf);
        }
        Ok(())
    }

    /// Refresh the set of inputs by polling the adapter 10 times, this ensures the results of
    /// [`read_controllers`](GcAdapter::read_controllers) is current.
    pub fn refresh_inputs(&mut self) -> Result<(), UsbError> {
        for _ in 0..10 {
            let mut buf = [0u8; 37];
            let n = self.usb.read_interrupt(&mut buf)?;
            if n == buf.len() {
                self.process_report(buf);
            }
        }
        Ok(())
    }

    /// Read the current state of all the controllers plugged into the adapter.
    ///
    /// Stick/axis calibration is automatically updated based on the result: when a controller is
    /// newly detected, its origin is recaptured (mirroring the GameCube's own origin behavior),
    /// and fixed radius-80 normalization remains stable as the stick is moved. Use
    /// [`calibration`](GcAdapter::calibration) to retrieve it for converting raw stick bytes into
    /// a normalized [-1.0, 1.0] range via [`PortCalibration::left_stick_coords`]/
    /// [`PortCalibration::right_stick_coords`]. No deadzone is applied.
    pub fn read_controllers(&mut self) -> Result<[Controller; 4], AdapterError> {
        let mut buf = [0u8; 37];
        let n = self.usb.read_interrupt(&mut buf)?;
        if n != buf.len() {
            return Err(AdapterError::ShortRead(n));
        }

        let ports = match Packet::parse(buf).map_err(AdapterError::Parse)? {
            Packet::ControllerInfo { ports } => ports,
            Packet::Unknown(_) => Default::default(),
        };

        self.update_calibration(&ports);

        Ok(ports)
    }

    /// Get the automatically-tracked calibration data for a given port (0-3). Panics if `chan`
    /// is out of range.
    pub fn calibration(&self, chan: usize) -> &PortCalibration {
        &self.calibration[chan]
    }

    fn process_report(&mut self, buffer: [u8; 37]) {
        if let Ok(Packet::ControllerInfo { ports }) = Packet::parse(buffer) {
            self.update_calibration(&ports);
        }
    }

    fn update_calibration(&mut self, ports: &[Controller; 4]) {
        for (i, controller) in ports.iter().enumerate() {
            let connected = controller.connected();
            let cal = &mut self.calibration[i];

            if connected {
                let left = controller.left_stick.raw();
                let right = controller.right_stick.raw();

                if !self.was_connected[i] {
                    // Newly connected: capture the origin, mirroring GameCube PAD origin behavior.
                    cal.left_stick.recenter(left);
                    cal.right_stick.recenter(right);
                }
            }

            self.was_connected[i] = connected;
        }
    }

    /// Creates a new `GcAdapter` from a USB connection that implements
    /// [`AdapterHardware`](AdapterHardware).
    pub fn new(usb: T) -> Self {
        Self {
            usb,
            was_connected: [false; 4],
            calibration: [PortCalibration::default(); 4],
        }
    }
}

#[cfg(feature = "libusb")]
impl GcAdapter<LibUsbAdapter<rusb::GlobalContext>> {
    /// Get an adapter from the libusb global context.
    ///
    /// Returns `Ok(None)` if no adapter is plugged in, and `Err` if a matching device was found
    /// but could not be opened/claimed (e.g. permissions, or already in use).
    pub fn from_usb() -> Result<Option<Self>, UsbError> {
        Ok(LibUsbAdapter::from_usb()?.map(Self::new))
    }
}

impl<T: AdapterHardware> Drop for GcAdapter<T> {
    fn drop(&mut self) {
        // clear rumble on drop; errors are ignored since there's nothing useful to do with them
        // during a drop, and we don't want to panic while unwinding/exiting.
        let _ = self.set_rumble([false; 4]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn origin_is_captured_once_per_connection() {
        let mut adapter = GcAdapter::new(TestHardware);
        let connected = Controller {
            status: ControllerStatus::new().with_controller_type(ControllerType::Normal),
            left_stick: Stick {
                x: SignedAxis::from_raw(140),
                y: SignedAxis::from_raw(120),
            },
            ..Default::default()
        };

        adapter.update_calibration(&[
            connected,
            Controller::default(),
            Controller::default(),
            Controller::default(),
        ]);
        assert_eq!(adapter.calibration(0).left_stick.center(), (140, 120));

        let moved = Controller {
            status: connected.status,
            left_stick: Stick {
                x: SignedAxis::from_raw(200),
                y: SignedAxis::from_raw(60),
            },
            ..Default::default()
        };
        adapter.update_calibration(&[
            moved,
            Controller::default(),
            Controller::default(),
            Controller::default(),
        ]);
        assert_eq!(adapter.calibration(0).left_stick.center(), (140, 120));

        adapter.update_calibration(&[
            Controller::default(),
            Controller::default(),
            Controller::default(),
            Controller::default(),
        ]);
        adapter.update_calibration(&[
            moved,
            Controller::default(),
            Controller::default(),
            Controller::default(),
        ]);
        assert_eq!(adapter.calibration(0).left_stick.center(), (200, 60));
    }

    struct TestHardware;

    impl AdapterHardware for TestHardware {
        fn write_interrupt(&mut self, _: &[u8]) -> Result<(), UsbError> {
            Ok(())
        }

        fn read_interrupt(&mut self, _: &mut [u8]) -> Result<usize, UsbError> {
            Ok(0)
        }
    }

    #[test]
    #[cfg(feature = "libusb")]
    fn test_display_controllers() {
        // get adapter from global context
        let mut adapter = GcAdapter::from_usb()
            .unwrap()
            .expect("no adapter plugged in");

        // refresh inputs to ensure they are up to date
        adapter.refresh_inputs().unwrap();

        // read and display all controller ports
        let controllers = adapter.read_controllers().unwrap();
        dbg!(&controllers);

        dbg!(adapter.calibration(3).left_stick_coords(&controllers[3]));
    }
}
