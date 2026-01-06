#![no_std]
#![no_main]

use cortex_m_rt::entry;

use panic_halt as _;
use rotary_encoder_embedded::{Direction, RotaryEncoder};
use rp_pico::hal::{clocks::init_clocks_and_plls, pac, sio::Sio, usb::UsbBus, watchdog::Watchdog};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // 1. Clock Setup (Essential for USB)
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

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // 2. Encoder Setup (DT=GP0, CLK=GP1)
    let rotary_dt = pins.gpio0.into_pull_up_input();
    let rotary_clk = pins.gpio1.into_pull_up_input();
    let mut encoder = RotaryEncoder::new(rotary_dt, rotary_clk).into_standard_mode();

    // 3. USB Serial Setup
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake Corp")
            .product("Serial Port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // CDC
        .build();

    let mut value = 0i32;

    loop {
        // 1. Always poll USB (to keep the connection alive)
        // We don't care if it returns true or false here, just let it do its housekeeping.
        let _ = usb_dev.poll(&mut [&mut serial]);

        // 2. Always poll the encoder
        // This needs to run as fast as possible to catch the rotation.
        match encoder.update() {
            Direction::Clockwise => {
                value += 1;
                // Optional: Check if we can write to avoid blocking,
                // but for simple debugging this is fine.
                let _ = serial.write(b"CW: ");
                write_num(&mut serial, value);
                let _ = serial.write(b"\r\n");
            }
            Direction::Anticlockwise => {
                value -= 1;
                let _ = serial.write(b"CCW: ");
                write_num(&mut serial, value);
                let _ = serial.write(b"\r\n");
            }
            Direction::None => {}
        }
    }
}

// Helper to write integers to serial
fn write_num<B: usb_device::bus::UsbBus>(serial: &mut SerialPort<B>, num: i32) {
    let mut buffer = [0u8; 20];
    let mut n = num;
    let mut i = 0;
    let is_neg = n < 0;
    if is_neg {
        n = -n;
    }

    if n == 0 {
        let _ = serial.write(b"0");
        return;
    }

    while n > 0 {
        buffer[i] = (n % 10) as u8 + b'0';
        n /= 10;
        i += 1;
    }
    if is_neg {
        let _ = serial.write(b"-");
    }
    for j in (0..i).rev() {
        let _ = serial.write(&[buffer[j]]);
    }
}
