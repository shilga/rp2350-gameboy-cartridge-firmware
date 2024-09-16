use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{driver::Driver, Builder, Handler};

use embassy_rp::rom_data::reboot;

use core::marker::PhantomData;
use core::mem::MaybeUninit;

use defmt::info;

/// Handle CONTROL endpoint requests and responses. For many simple requests and responses
/// you can get away with only using the control endpoint.
struct Control<'d> {
    if_num: InterfaceNumber,
    _lifetime: PhantomData<&'d ()>, // NOTE: there's no generic type here!
}

impl<'d> Handler for Control<'d> {
    /// Respond to HostToDevice control messages, where the host sends us a command and
    /// optionally some data, and we can only acknowledge or reject it.
    fn control_out<'a>(&'a mut self, req: Request, buf: &'a [u8]) -> Option<OutResponse> {
        // Log the request before filtering to help with debugging.
        info!("Got control_out, request={}, buf={:a}", req, buf);

        // Only handle Vendor request types to an Interface.
        if req.request_type != RequestType::Class || req.recipient != Recipient::Interface {
            return None;
        }

        // Ignore requests to other interfaces.
        if req.index != self.if_num.0 as u16 {
            return None;
        }

        match req.request {
            0x01 => {
                // reset
                reboot(0x102, 10, 0, 0);
                unreachable!()
            }
            _ => Some(OutResponse::Rejected),
        }
    }

    /// Respond to DeviceToHost control messages, where the host requests some data from us.
    fn control_in<'a>(&'a mut self, req: Request, _buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        info!("Got control_in, request={}", req);

        // Only handle Vendor request types to an Interface.
        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
            return None;
        }

        // Ignore requests to other interfaces.
        if req.index != self.if_num.0 as u16 {
            return None;
        }

        // we are not expecting any USB IN requests
        Some(InResponse::Rejected)
    }
}

pub struct PicotoolReset<'d> {
    control: MaybeUninit<Control<'d>>,
}

impl<'d> PicotoolReset<'d> {
    pub const fn new() -> Self {
        PicotoolReset {
            control: MaybeUninit::uninit(),
        }
    }

    /// Builder for the PicotoolReset capability implementation.
    ///
    /// Pass in a USB `Builder`, a `State`, which holds the the control endpoint state, and a `Config` for the WebUSB configuration.
    pub fn configure<D: Driver<'d>>(&'d mut self, builder: &mut Builder<'d, D>) {
        // Add a vendor-specific function (class 0xFF), and corresponding interface,
        // that uses our custom handler.
        let mut function = builder.function(0xFF, 0, 0);
        let mut interface = function.interface();
        let _alt = interface.alt_setting(0xFF, 0, 1, None);

        let control = self.control.write(Control {
            if_num: interface.interface_number(),
            _lifetime: PhantomData,
        });

        drop(function);
        builder.handler(control);
    }
}
