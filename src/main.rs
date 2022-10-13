#![no_std]
#![no_main]

use panic_halt as _;
use rp2040_hal as hal;
use hal::pac;
use embedded_hal::digital::v2::ToggleableOutputPin;
use hal::pac::interrupt;
use core::cell::RefCell;
use critical_section::Mutex;
use rp2040_hal::gpio;
use rp2040_hal::gpio::Interrupt::EdgeHigh;
use rp2040_hal::entry;

type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::PushPullOutput>;
type ButtonPin = gpio::Pin<gpio::bank0::Gpio20, gpio::PullUpInput>;
type LedAndButton = (LedPin, ButtonPin);

static GLOBAL_PINS: Mutex<RefCell<Option<LedAndButton>>> = Mutex::new(RefCell::new(None));


#[interrupt]
fn IO_IRQ_BANK0() {
    static mut LED_AND_BUTTON: Option<LedAndButton> = None;

    if LED_AND_BUTTON.is_none() {
        critical_section::with(|cs| {
            *LED_AND_BUTTON = GLOBAL_PINS.borrow(cs).take();
        });
    }

    if let Some(gpios) = LED_AND_BUTTON {
        let (led, button) = gpios;
        if button.interrupt_status(EdgeHigh) {
            let _ = led.toggle();
            button.clear_interrupt(EdgeHigh);
        }
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    ).ok().unwrap();

    let led = pins.gpio25.into_mode();
    let in_pin = pins.gpio20.into_mode();

    in_pin.set_interrupt_enabled(EdgeHigh, true);
    critical_section::with(|cs| {
        GLOBAL_PINS.borrow(cs).replace(Some((led, in_pin)));
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    loop {
        cortex_m::asm::wfi();
    }
}
