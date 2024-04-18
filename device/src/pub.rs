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

#[cfg(not(feature = "defmt-log"))]
pub use log::{debug, error, info, trace, warn, Level};

#[cfg(feature = "defmt-log")]
pub use defmt::{debug, error, info, trace, warn};

extern crate alloc_cortex_m;
use alloc_cortex_m::CortexMHeap;

// Import global logger
#[cfg(feature = "defmt-log")]
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
    let uart = device.USART3.tx(uart_tx, 115_200.bps(), &clocks).unwrap();
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

    let _ = rf_cs.set_high();
    let _ = rf_ant.set_high();

    // Setup SPI
    let (spi_mosi, spi_miso, spi_sck) = {
        (
            gpioa.pa7.into_alternate().speed(Speed::VeryHigh),
            gpioa.pa6.into_alternate().speed(Speed::VeryHigh),
            gpioa.pa5.into_alternate().speed(Speed::VeryHigh),
        )
    };
    let spi = Spi::new(
        device.SPI1,
        (spi_sck, spi_miso, spi_mosi),
        MODE_0,
        1.MHz().into(),
        &clocks,
    );
    let rf_spi = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, rf_cs);


    // Setup I2C
    let (i2c_scl, i2c_sda) = {
        (
            gpiob
                .pb10
                .into_alternate()
                .speed(Speed::VeryHigh)
                .internal_pull_up(true)
                .set_open_drain(),
            gpiob
                .pb11
                .into_alternate()
                .speed(Speed::VeryHigh)
                .internal_pull_up(true)
                .set_open_drain(),
        )
    };

    let i2c = I2c::new(device.I2C2, (i2c_scl, i2c_sda), 10.kHz(), &clocks);
    //let i2c_bus = embedded_hal_bus::i2c::ExclusiveDevice::new_no_delay(i2c);

    // Setup a timer object
    //let mut timer = Timer::tim1(device.TIM1, 1.khz(), clocks).unwrap();

    info!("Initialising radio");

    // Initialise radio

    // Setup radio timer
    let rf_delay = device.TIM1.delay_us(&clocks);

    let rf_config = common::rf_config();
    let mut radio = match Sx128x::spi(rf_spi, rf_busy, rf_ready, rf_rst, rf_delay, &rf_config) {
        Ok(v) => {
            info!("Radio init okay!");
            v
        },
        Err(e) => {
            panic!("Error initialising radio: {:?}", e);
        }
    };

    // Re-set syncword for GFSK mode
    if let Modem::Gfsk(_gfsk) = &rf_config.modem {
        radio
            .set_syncword(1, &[0x11, 0x22, 0x33, 0x44, 0x55])
            .unwrap();
    }

    info!("Initialising bme280");

    // Initialise sensor

    // Setup i2c timer
    let mut bme_delay = device.TIM2.delay_us(&clocks);

    let mut bme280 = BME280::new(i2c, 0x77);
    if let Err(e) = bme280.init(&mut bme_delay) {
        panic!("Error initialising BME280: {:?}", e);
    }

    info!("Initialising networking");

    // Initialise network stack
    let address = ExtendedAddress(chip_id);
    let mac_config = mac_802154::Config {
        pan_coordinator: false,
        ..Default::default()
    };

    let timer = SystickDelay {};
    let mac = match mac_802154::Mac::new(address, mac_config, radio, timer.clone()) {
        Ok(m) => m,
        Err(e) => {
            panic!("Error initalising MAC: {:?}", e);
        }
    };

    debug!("Initialising 6lo");
    let sixlo_cfg = SixLoConfig {
        ..Default::default()
    };
    let mut sixlo = SixLo::<_, 127>::new(mac, MacAddress::Extended(PanId(1), address), sixlo_cfg);

    info!("Building service");

    // Create endpoints
    let info = IotInfo::<4>::new(&[
        EpDescriptor::new(EpKind::Temperature, EpFlags::R),
        EpDescriptor::new(EpKind::Pressure, EpFlags::R),
        EpDescriptor::new(EpKind::Humidity, EpFlags::R),
    ])
    .unwrap();

    #[cfg(wip)]
    {
        let comms = RadioComms::new();
        let store = HeaplessStore::new();

        let keys = device.iter().find(|(c, _k)| *c == chip_id);
        if let Some(k) = &keys {
            store.set_ident(&k.1);
        }

        let mut engine = match IotEngine::new(info, &[], comms, store) {
            Ok(e) => e,
            Err(e) => {
                error!("Failed to initialise engine: {:?}", e);
                loop {}
            }
        };
    }

    {
        let mut body = [0u8; 1024];

        // TODO: set kind to IotService
        let mut sb = ServiceBuilder::default();

        let keys = device_keys.iter().find(|(c, _k)| *c == chip_id);
        if let Some(k) = keys {
            sb = sb.private_key(k.1.pri_key.clone().unwrap());
        }

        let mut s = sb.body(info).build().unwrap();
        info!("Service ID: {}", s.id().to_string());

        let mut buff = [0u8; 1024];
        let (n, _p) = s
            .publish_primary(PrimaryOptions::default(), &mut buff)
            .unwrap();

        let primary = &buff[..n];

        info!("Generated primary page of {} bytes", n);

        debug!("page: {:02x?}", primary);

        info!("Starting app");

        let mut rf_buff = [0u8; 512];
        let mut gateway = None;
        let mut last_reading = 0;

        let mut net_state = NetState::None;

        loop {
            let now = SystickDelay {}.ticks_ms();

            // Update the mac
            if let Err(e) = sixlo.tick(now) {
                error!("MAC error: {:?}", e);
            }

            // TODO: check for received packets
            if let Ok(Some((n, addr, _hdr))) = sixlo.receive(now, &mut rf_buff) {
                debug!("Received {} byte packet", n);

                let base = match Container::parse(&mut rf_buff[..n], &keystore) {
                    Ok(v) => v,
                    Err(e) => {
                        error!("DSF parsing error: {:?}", e);
                        continue;
                    }
                };

                debug!("Parsed to base object");

                let id = base.id().clone();

                let m = match NetMessage::convert(base, &keystore) {
                    Ok(m) => m,
                    Err(_e) => {
                        error!("DSF rx was not a network message");
                        continue;
                    }
                };

                match &m {
                    // Handle discovery requests
                    NetMessage::Request(req) if req.data == NetRequestBody::Hello => {
                        // TODO: only respond to _unknown_ gateways (don't need to send this all the time)

                        info!(
                            "Received hello from {} ({:?}), sending service page",
                            id, addr
                        );

                        if let Err(e) = sixlo.transmit(now, addr, primary) {
                            error!("MAC transmit error: {:?}", e);
                        }

                        // TODO: this should not be _here_
                        gateway = Some(addr);
                    }
                    // Handle registration / deregistration messages
                    NetMessage::Response(resp) => {
                        info!("Received response {}", m.request_id());

                        info!("{:?}", resp.data);

                        if let NetState::Joining(gw) = net_state {
                            net_state = NetState::Joined(gw);
                            info!("Join complete");
                        }
                    }
                    // TODO: handle data requests
                    _ => {
                        warn!("Unhandled DSF request: {:?}", m);
                    }
                }
            }

            if let Ok(MacState::Synced(gw)) = sixlo.mac().state() {
                led1.set_high();

                match &net_state {
                    NetState::None => {
                        info!("Registering service with gw: {:?}", gw);

                        if let Err(e) = sixlo.transmit(now, gw, primary) {
                            error!("MAC transmit error: {:?}", e);
                        }

                        net_state = NetState::Joining(gw);
                    }
                    NetState::Joined(_) => {
                        if (now > (last_reading + 10_000)) || last_reading == 0 {
                            debug!("Measurement start");
        
                            let m = bme280.measure(&mut bme_delay).unwrap();
        
                            info!(
                                "Measurement temp: {} press: {} humid: {}",
                                m.temperature as i32, m.pressure as i32, m.humidity as i32
                            );
        
                            let endpoint_data = IotData::<4>::new(&[
                                EpData::new(roundf(m.temperature).into()),
                                EpData::new(roundf(m.pressure / 1000.0).into()),
                                EpData::new(roundf(m.humidity).into()),
                            ])
                            .unwrap();
        
                            let page_opts = DataOptions {
                                body: Some(endpoint_data),
                                ..Default::default()
                            };
        
                            let mut page_buff = [0u8; 512];
                            let (n, _p) = s.publish_data(page_opts, &mut page_buff[..]).unwrap();
                            let data = &page_buff[..n];
        
                            info!("Transmitting data object of {} bytes to: {:?}", n, gw);
        
                            if let Err(e) = sixlo.transmit(now, gw, data) {
                                error!("MAC transmit error: {:?}", e);
                            }
        
                            last_reading = SystickDelay {}.ticks_ms();
                        }
                    }
                    _ => (),
                }

            } else {
                net_state = NetState::None;
                led1.set_low();
            }

            if (SystickDelay {}.ticks_ms() / 500) % 2 == 0 {
                led0.set_high();
            } else {
                led0.set_low();
            }

            SystickDelay {}.delay_ms(1u32);
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub enum NetState {
    None,
    Joining(MacAddress),
    Joined(MacAddress),
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

    error!("{}", info);

    loop {}
}
