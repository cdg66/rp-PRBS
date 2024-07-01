//! Send a PRBS sequence to pin 21 using the PIO state machine
//! Also set high the pin 20 to enable your differntial transmitter
//! Usefull when you need to do a eye diagram.
//! This code use the rp-pico BSP.
//!
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

use rp_pico::hal::pio::PIOExt;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;
use crate::hal::pio::Buffers;
use bsp::hal;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionPio0, Pin},
    sio::Sio,
    watchdog::Watchdog,
};
use rp_pico::hal::pac;

use rand_core::RngCore;
use wyhash::WyRng;

#[entry]
fn main() -> ! {
    info!("Program start");
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

    let mut _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let tx_pin = pins.gpio21.into_function::<FunctionPio0>();
    let mut tx_en = pins.gpio20.into_push_pull_output();

    let program_tx = pio_proc::pio_asm!(
        ".wrap_target",
        "out pins 1", // the txbuffer need to be always full
        "nop",// remove nop for faster speed
        ".wrap"
        options(max_program_size = 32)
    );

    // Initialize and start tx machine
    let (mut pio, tx0, _, _, _) = pac.PIO0.split(&mut pac.RESETS); //split(&mut pac.RESETS);
    let installed = pio.install(&program_tx.program).unwrap();
    let (int, frac) = (0, 0); // as slow as possible (0 is interpreted as 65536)
    let (mut tx0, _, mut tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .set_pins(tx_pin.id().num, 1)
        .autopull(true)
        .buffers(Buffers::OnlyTx)
        .clock_divisor_fixed_point(int, frac)
        .pull_threshold(32)
        .out_pins(tx_pin.id().num, 1)
        .out_shift_direction(hal::pio::ShiftDirection::Left)
        .build(tx0);
    // The GPIO pin needs to be configured as an output.
    //sm.set_pindirs([(led_pin_id, hal::pio::PinDir::Output)]);
    tx0.set_pindirs([(tx_pin.id().num, hal::pio::PinDir::Output)]);

    //setup the rng
    let mut rng = WyRng::default();
    //fill the buffer

    for _i in 0..8 {
        tx.write(rng.next_u32());
    }
    //enable the output
    tx_en.set_high(); // use set_low() if your enable is logic low.
    tx0.start();
    loop {
        while tx.is_full() {}
        tx.write(rng.next_u32());
    }
}

// End of file
