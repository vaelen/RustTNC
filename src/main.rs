#![no_std]
#![no_main]

use bsp::hal::uart::{Enabled, UartDevice, ValidUartPinout};
use core::fmt::Write;
use cortex_m::delay::Delay;
use defmt_rtt as _;
//use embedded_hal::digital::v2::OutputPin;
use fugit::RateExtU32; // Provides u32.MHz() and u32.kHz()
use heapless::String;
use panic_probe as _;

use bsp::entry;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::FunctionUart,
    i2c::I2C,
    pac,
    sio::Sio,
    uart::{self, UartPeripheral},
    watchdog::Watchdog,
};
use rp_pico as bsp;

const LCD_ADDRESS: u8 = 0x27;

#[entry]
fn main() -> ! {
    // info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

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

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let gps = UartPeripheral::new(
        pac.UART0,
        (
            pins.gpio0.into_mode::<FunctionUart>(),
            pins.gpio1.into_mode::<FunctionUart>(),
        ),
        &mut pac.RESETS,
    )
    .enable(
        uart::common_configs::_9600_8_N_1,
        clocks.peripheral_clock.freq(),
    )
    .unwrap();

    let mut console = UartPeripheral::new(
        pac.UART1,
        (
            pins.gpio4.into_mode::<FunctionUart>(),
            pins.gpio5.into_mode::<FunctionUart>(),
        ),
        &mut pac.RESETS,
    )
    .enable(
        uart::common_configs::_9600_8_N_1,
        clocks.peripheral_clock.freq(),
    )
    .unwrap();

    console.write_str("\r\nStarting.\r\n").ok();

    let mut i2c = I2C::i2c0(
        pac.I2C0,
        pins.gpio8.into_mode(), // sda
        pins.gpio9.into_mode(), // scl
        400.kHz(),
        &mut pac.RESETS,
        125.MHz(),
    );

    // Scan for devices on the bus by attempting to read from them
    // use embedded_hal::prelude::_embedded_hal_blocking_i2c_Read;
    // loop {
    //     write!(uart0, "\r\nScanning\r\n--------\r\n");
    //     for i in 0..=127 {
    //         let mut readbuf: [u8; 1] = [0; 1];
    //         write!(uart0, "{:02x?} ", i).ok();
    //         let result = i2c.read(i, &mut readbuf);
    //         if let Ok(_d) = result {
    //             // Do whatever work you want to do with found devices
    //             write!(uart0, "\r\nDevice found: {:02x?}\r\n", i).ok();
    //         }
    //     }
    //     delay.delay_ms(10000);
    // }

    let mut lcd = lcd_lcm1602_i2c::Lcd::new(&mut i2c, &mut delay)
        .address(LCD_ADDRESS)
        .cursor_on(false)
        .rows(4)
        .init()
        .unwrap();

    lcd.clear().ok();
    lcd.return_home().ok();
    lcd.write_str("Hello world!").ok();

    //let mut led_pin = pins.led.into_push_pull_output();

    //use cortex_m::prelude::_embedded_hal_serial_Read;
    //use cortex_m::prelude::_embedded_hal_serial_Write;

    // Create storage space for each NMEA sentence from the GPS.
    let mut nmea_data: String<256> = String::new();
    let mut nmea_left: String<256> = String::new();
    let mut buffer: [u8; 32] = [0; 32];

    loop {
        match gps.read_raw(&mut buffer) {
            Ok(bytes_read) => {
                let read = &buffer[0..bytes_read];
                for b in read {
                    nmea_data.push(char::from(*b)).unwrap();
                }
                if read.contains(&b'\n') {
                    let lines = nmea_data.lines();
                    for line in lines {
                        nmea_left.clear();
                        let l = line.trim();
                        if !parse_nmea(l, &mut console) {
                            nmea_left.push_str(l).ok();
                        }
                    }
                    if nmea_data.ends_with("\n") {
                        nmea_left.push('\r').ok();
                        nmea_left.push('\n').ok();
                    }
                    nmea_data.clear();
                    if !nmea_left.is_empty() {
                        nmea_data.push_str(&nmea_left[..]).ok();
                    }
                }
            }
            _ => {}
        }
    }
}

fn parse_nmea<D, P>(s: &str, console: &mut UartPeripheral<Enabled, D, P>) -> bool
where
    D: UartDevice,
    P: ValidUartPinout<D>,
{
    match s.find('*') {
        None => false,
        Some(end) => {
            let remaining = &s[0..end];
            if !remaining.len() >= 8 && (remaining.starts_with("$") || remaining.starts_with("!")) {
                let speaker = &remaining[1..3];
                let sentence_type = &remaining[3..6];
                let remaining = &remaining[7..];
                write!(
                    console,
                    "Speaker: {}, Type: {}, Remainder: {}\r\n",
                    speaker, sentence_type, remaining
                )
                .ok();
            }
            true
        }
    }
}

/*
$GPTXT,01,01,01,ANTENNA OPEN*25
$GNGGA,,,,,,0,00,25.5,,,,,,*64
$GNGLL,,,,,,V,N*7A
$GPGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5*02
$BDGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5*13
$GPGSV,1,1,01,07,,,12*7C
$BDGSV,1,1,00*68
$GNRMC,,V,,,,,,,,,,N*4D
$GNVTG,,,,,,,,,N*2E
$GNZDA,,,,,,*56

 */

struct _Position {
    latitude: String<9>,
    ns: char,
    longitude: String<10>,
    ew: char,
    utc: String<10>,
    status: bool,
    checksum: String<2>,
}
