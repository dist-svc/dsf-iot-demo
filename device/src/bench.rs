//#![feature(lang_items)]
#![no_std]
#![no_main]
#![feature(lazy_cell)]
#![feature(alloc_error_handler)]

use core::alloc::Layout;
use core::borrow::BorrowMut;
use core::cell::{Cell, RefCell};
use core::num::NonZeroU32;
use core::ops::DerefMut;
use core::panic::PanicInfo;

use core::sync::atomic::{AtomicUsize, Ordering};

extern crate alloc;
use alloc::{boxed::Box, string::ToString, vec::Vec};

use cortex_m_rt::{entry, exception};

use cortex_m::interrupt::{free, Mutex};
use cortex_m::peripheral::syst::SystClkSource;

#[cfg(not(feature = "defmt"))]
pub use log::{debug, error, info, trace, warn, Level};

#[cfg(feature = "defmt")]
pub use defmt::{debug, error, info, trace, warn};

extern crate alloc_cortex_m;
use alloc_cortex_m::CortexMHeap;

// Import global logger
#[cfg(feature = "defmt")]
use defmt_rtt as _;

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::MODE_0;

use hal::gpio::Speed;
use hal::prelude::*;
use hal::serial;
use hal::timer::{Event, Timer};
use hal::{i2c::I2c, serial::Serial, spi::Spi};
use stm32f4xx_hal as hal;

use rand_chacha::ChaChaRng;
use rand_core::{CryptoRng, RngCore, SeedableRng};

use bme280::i2c::BME280;

use radio_sx128x::prelude::*;
use radio_sx128x::Config as Sx128xConfig;

use lpwan::mac_802154::{AssocState, SyncState};
use lpwan::prelude::*;

use dsf_core::{
    prelude::*,
    service::{Publisher, ServiceBuilder},
    crypto::{Crypto, PubKey as _, SecKey as _},
};

use dsf_engine::store::Store;
use dsf_iot::prelude::*;

mod newlib;

mod serial_log;
use serial_log::SerialLogger;

mod timer;
use timer::SystickDelay;

mod common;
use common::{device_keys, HeaplessStore, RadioComms, StaticKeyStore};

//#[link(name = "c", kind = "static")]
//extern {}

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
const ALLOC_SIZE: usize = 20 * 1024;

const LOG_FILTERS: &'static [&'static str] = &[ "radio_sx128x" ];

// https://dev.to/minkovsky/rusted-brains-running-rust-firmware-on-a-cortex-m-microcontroller-3had
//static TIMER_TIM2: Mutex<RefCell<Option<Timer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));
//static TIM2_COUNT: AtomicUsize = AtomicUsize::new(0);

// Bind custom OS RNG
getrandom::register_custom_getrandom!(os_rand);

static mut RNG: Option<ChaChaRng> = None;

// OS RNG function
pub fn os_rand(buf: &mut [u8]) -> Result<(), getrandom::Error> {
    unsafe {
        let rng = match RNG.borrow_mut() {
            Some(rng) => rng,
            None => {
                let code = NonZeroU32::new(1).unwrap();
                return Err(getrandom::Error::from(code));
            }
        };

        rng.fill_bytes(buf);
    }

    Ok(())
}

#[entry]
fn main() -> ! {
    // Fetch device handles
    let core = cortex_m::peripheral::Peripherals::take().unwrap();
    let device = hal::pac::Peripherals::take().unwrap();

    // Setup system clock
    let rcc = device.RCC.constrain();
    let clocks = rcc
        .cfgr
        .hclk(180.MHz())
        .sysclk(180.MHz())
        .pclk1(45.MHz())
        .pclk2(90.MHz())
        .require_pll48clk()
        .freeze();

    // Initialize the allocator
    let start = cortex_m_rt::heap_start() as usize;
    unsafe { ALLOCATOR.init(start, ALLOC_SIZE) }

    let gpioa = device.GPIOA.split();
    let gpiob = device.GPIOB.split();
    let gpioc = device.GPIOC.split();
    let gpiod = device.GPIOD.split();
    let gpioe = device.GPIOE.split();
    let gpiof = device.GPIOF.split();

    // Setup pins
    let mut led0 = gpiob.pb7.into_push_pull_output();
    let mut led1 = gpiob.pb14.into_push_pull_output();

    // Setup serial (and logging)
    let uart_tx = gpiod.pd8;
    let _uart_rx = gpiod.pd9;
    #[cfg(not(feature = "defmt"))]
    let uart = device.USART3.tx(uart_tx, 115_200.bps(), &clocks).unwrap();

    #[cfg(not(feature = "defmt"))]
    SerialLogger::init(Level::Debug, Box::new(uart), LOG_FILTERS);

    let mut syst = core.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(18_000 - 1);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    // Create delay object
    let delay = SystickDelay{};

    // Fetch unique chip ID for address use
    let chip_id = hal::signature::Uid::get();
    let chip_id =
        (chip_id.waf_num() as u64) << 32 | (chip_id.y() as u64) << 16 | (chip_id.x() as u64);

    info!("hello world");

    info!("Starting device with ID: {}", chip_id);

    let device_keys = device_keys();

    info!("Configuring RNG");

    // Setup RNG
    let dev_rng = device.RNG.constrain(&clocks);
    let chacha_rng = ChaChaRng::from_rng(dev_rng).unwrap();
    unsafe { RNG = Some(chacha_rng) };

    let mut keystore = StaticKeyStore::new();

    info!("RNG bound");

    // Setup Pins
    let (mut rf_cs, rf_rst, rf_busy, rf_ready, mut rf_ant) = (
        gpioe.pe9.into_push_pull_output(),
        gpioc.pc0.into_push_pull_output(),
        gpiof.pf15.into_floating_input(),
        gpiof.pf12.into_floating_input(),
        gpiof.pf5.into_push_pull_output(),
    );



    let (pub_key, pri_key) = Crypto::new_pk().unwrap();
    let mut data = [0xabu8; 256];

    let timer = device.TIM1.counter_us(&clocks);

    info!("Start pk sign");

    let now = SystickDelay{}.ticks_us();

    for i in 0..100 {
        core::hint::black_box({
            let _sig = Crypto::pk_sign(&pri_key, &data).expect("Error generating signature");
        })
    }

    let elapsed = SystickDelay{}.ticks_us() - now;

    info!("PK sign complete in {} us ({} us / iteration)", elapsed, elapsed / 100);


    let sig = Crypto::pk_sign(&pri_key, &data).expect("Error generating signature");

    let now = SystickDelay{}.ticks_us();

    for i in 0..100 {
        core::hint::black_box({
            let _sig = Crypto::pk_verify(&pub_key, &sig, &data).expect("Error verifying signature");
        })
    }

    let elapsed = SystickDelay{}.ticks_us() - now;

    info!("PK verify complete in {} us ({} us / iteration)", elapsed, elapsed / 100);

    let sec_key = Crypto::new_sk().unwrap();

    let now = SystickDelay{}.ticks_us();

    for i in 0..100 {
        core::hint::black_box({
            let _tag = Crypto::sk_encrypt(&sec_key, None, &mut data).expect("Error encrypting data");
        })
    }

    let elapsed = SystickDelay{}.ticks_us() - now;

    info!("SK encrypt complete in {} us ({} us / iteration)", elapsed, elapsed / 100);

    let mut enc_data = [77u8; 256];

    let tag = Crypto::sk_encrypt(&sec_key, None, &mut enc_data).expect("Error encrypting data");
    let now = SystickDelay{}.ticks_us();

    for i in 0..100 {
        let mut dec_data = enc_data;

        core::hint::black_box({
            let _tag = Crypto::sk_decrypt(&sec_key, &tag, None, &mut dec_data).expect("Error encrypting data");
        })
    }

    let elapsed = SystickDelay{}.ticks_us() - now;

    
    info!("SK decrypt complete in {} us ({} us / iteration)", elapsed, elapsed / 100);

    loop {

        if (SystickDelay {}.ticks_ms() / 500) % 2 == 0 {
            led0.set_high();
        } else {
            led0.set_low();
        }

        SystickDelay {}.delay_ms(1u32);
    }
}


use num_traits::float::FloatCore;

fn roundf(v: f32) -> f32 {
    f32::round(v * 100.0) / 100.0
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    error!("{:?}", defmt::Debug2Format(info));

    loop {}
}
