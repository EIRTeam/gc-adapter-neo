use rusb::{DeviceHandle, GlobalContext, UsbContext};
use std::time::Duration;

use super::{AdapterHardware, UsbError};
use crate::constants::*;

#[cfg(doc)]
use crate::GcAdapter;

/// Matches Dolphin's `USB_TIMEOUT_MS`: interrupt transfers use a finite timeout rather than
/// blocking forever, so a disconnect/unplug surfaces as a recoverable [`UsbError`] instead of
/// hanging the calling thread indefinitely.
const USB_TIMEOUT: Duration = Duration::from_millis(100);

fn map_rusb_error(err: rusb::Error) -> UsbError {
    match err {
        rusb::Error::Timeout => UsbError::Timeout,
        rusb::Error::Pipe => UsbError::Stall,
        rusb::Error::NoDevice | rusb::Error::NotFound => UsbError::Disconnected,
        _ => UsbError::Other,
    }
}

/// Adapter interface for libusb to provide USB access for the gamecube adapter.
///
/// The suggested interface for using this is [`GcAdapter::from_usb`](GcAdapter::from_usb). This
/// should only be used if you want to provide your own [`rusb::UsbContext`](UsbContext)
pub struct LibUsbAdapter<Context: UsbContext> {
    handle: DeviceHandle<Context>,
    has_kernel_driver: bool,
}

impl<Context: UsbContext> LibUsbAdapter<Context> {
    pub fn from_usb_context(context: Context) -> Result<Option<Self>, UsbError> {
        match context.open_device_with_vid_pid(ADAPTER_VID, ADAPTER_PID) {
            Some(handle) => Self::from_handle(handle).map(Some),
            None => Ok(None),
        }
    }

    pub fn from_handle(mut handle: DeviceHandle<Context>) -> Result<Self, UsbError> {
        let endpoint = 0;
        let has_kernel_driver = handle.kernel_driver_active(endpoint).unwrap_or(false);

        if has_kernel_driver {
            handle
                .detach_kernel_driver(endpoint)
                .map_err(map_rusb_error)?;
        }

        handle.claim_interface(endpoint).map_err(map_rusb_error)?;
        let mut adapter = Self {
            handle,
            has_kernel_driver,
        };

        // Mirrors Dolphin's CheckDeviceAccess: this HID SET_IDLE-style control transfer is
        // required for some 3rd party adapters (e.g. Nyko) to start reporting input, but is
        // expected to fail with a pipe error on others (e.g. Mayflash)
        let _ = adapter
            .handle
            .write_control(0x21, 11, 0x0001, 0, &[], USB_TIMEOUT);

        // Init payload that puts the adapter into "polling" mode.
        adapter.write_interrupt(&[0x13])?;

        Ok(adapter)
    }
}

impl<Context: UsbContext> Drop for LibUsbAdapter<Context> {
    fn drop(&mut self) {
        let endpoint = 0;
        let _ = self.handle.release_interface(endpoint);
        if self.has_kernel_driver {
            let _ = self.handle.attach_kernel_driver(endpoint);
        }
    }
}

impl LibUsbAdapter<GlobalContext> {
    pub fn from_usb() -> Result<Option<Self>, UsbError> {
        match rusb::open_device_with_vid_pid(ADAPTER_VID, ADAPTER_PID) {
            Some(handle) => Self::from_handle(handle).map(Some),
            None => Ok(None),
        }
    }
}

impl<Context: UsbContext> super::AdapterHardware for LibUsbAdapter<Context> {
    fn read_interrupt(&mut self, data: &mut [u8]) -> Result<usize, UsbError> {
        self.handle
            .read_interrupt(0x81, data, USB_TIMEOUT)
            .map_err(map_rusb_error)
    }

    fn write_interrupt(&mut self, data: &[u8]) -> Result<(), UsbError> {
        self.handle
            .write_interrupt(2, data, USB_TIMEOUT)
            .map(|_| ())
            .map_err(map_rusb_error)
    }
}
