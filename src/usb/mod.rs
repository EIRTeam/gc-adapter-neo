/// A libusb implementation of the adapter hardware interface
#[cfg(feature = "libusb")]
mod libusb;

#[cfg(feature = "libusb")]
pub use libusb::*;

/// Errors that can occur while communicating with the adapter over USB.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsbError {
    /// The adapter is no longer connected (unplugged, or claim/open failed).
    Disconnected,
    /// The operation timed out before completing. This is often recoverable by retrying.
    Timeout,
    /// The endpoint stalled/returned a protocol error (e.g. a rumble command to a wireless
    /// controller, or an adapter that doesn't support a given control transfer).
    Stall,
    /// Some other, less common USB error occurred.
    Other,
}

/// A trait representing a struct which provides access to a limited set of
/// USB operations
pub trait AdapterHardware {
    /// Write `data` to the adapter's output endpoint.
    fn write_interrupt(&mut self, data: &[u8]) -> Result<(), UsbError>;

    /// Read into `data` from the adapter's input endpoint, returning the number of bytes
    /// actually read.
    fn read_interrupt(&mut self, data: &mut [u8]) -> Result<usize, UsbError>;
}
