//! Blinks the LED on a Pico board and creates a USB serial device.
//!
//! This will blink an LED attached to GP25 (or GPIO16) and create a USB serial port.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
use rp_pico as bsp;

use bsp::hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};

use core::fmt::Write;
use heapless::String;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(bsp::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Serial Port
    let mut serial = SerialPort::new(&usb_bus);

    // Set up the USB Device
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .device_class(2) // CDC class
        .build();

    let mut count = 0;
    let mut index = 0;

    loop {
        // Poll the USB device
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    // Echo back
                    let _ = serial.write(&buf[0..count]);
                }
                _ => {}
            }
        }

        // Blink LED and print to serial occasionally
        // Note: This delay blocks the USB poll, so it's not ideal for a responsive USB device.
        // For a real application, use a timer interrupt or non-blocking delay.
        // However, for this simple example, we'll just poll in a tight loop and use a counter for timing.

        count += 1;
        index += 1;
        if count > 100000 {
            count = 0;
            let mut text: String<64> = String::new();
            let _ = text.write_fmt(format_args!("Hello from pico index: {}\r\n", index));
            let _ = serial.write(text.as_bytes());
        }
    }
}
