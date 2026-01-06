#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use rotary_encoder_embedded::{Direction, RotaryEncoder};
use rp_pico::hal::{
    clocks::init_clocks_and_plls,
    pac,
    sio::Sio,
    usb::UsbBus,
    watchdog::Watchdog,
    Timer, // <--- Import Timer
};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // 1. Clock Setup
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

    // 2. Setup the Timer for debouncing
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // 3. Encoder Setup
    let rotary_dt = pins.gpio0.into_pull_up_input();
    let rotary_clk = pins.gpio1.into_pull_up_input();
    let mut encoder = RotaryEncoder::new(rotary_dt, rotary_clk).into_standard_mode();

    // 4. USB Setup
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
        .device_class(2)
        .build();

    let mut value = 0i32;
    let mut last_turn_time = 0u64; // Track when the last valid turn happened
    const DEBOUNCE_TIME_US: u64 = 50000;

    loop {
        let _ = usb_dev.poll(&mut [&mut serial]);

        // Get the current direction
        let dir = encoder.update();

        // Check the current time (in microseconds)
        let now = timer.get_counter().ticks();

        match dir {
            Direction::Clockwise => {
                // DEBOUNCE: Only accept if 5ms (5000us) have passed since the last turn
                if (now - last_turn_time) > DEBOUNCE_TIME_US {
                    value += 1;
                    let _ = serial.write(b"CW: ");
                    write_num(&mut serial, value);
                    let _ = serial.write(b"\r\n");

                    last_turn_time = now; // Update timestamp
                }
            }
            Direction::Anticlockwise => {
                if (now - last_turn_time) > DEBOUNCE_TIME_US {
                    value -= 1;
                    let _ = serial.write(b"CCW: ");
                    write_num(&mut serial, value);
                    let _ = serial.write(b"\r\n");

                    last_turn_time = now;
                }
            }
            Direction::None => {}
        }
    }
}

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
