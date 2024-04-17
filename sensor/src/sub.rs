
#![no_std]
#![no_main]

#![feature(alloc_error_handler)]

use core::alloc::Layout;
use core::panic::PanicInfo;
use core::str::FromStr;
use core::convert::TryFrom;
use core::fmt::Write;

extern crate alloc;
use alloc::{boxed::Box, string::ToString};

use cortex_m_rt::{entry};

use cortex_m::peripheral::syst::SystClkSource;

use rand_core::RngCore;

#[cfg(not(feature = "defmt"))]
pub use log::{Level, trace, debug, info, warn, error};

#[cfg(feature = "defmt")]
pub use defmt::{trace, debug, info, warn, error};

extern crate alloc_cortex_m;
use alloc_cortex_m::CortexMHeap;

// Import global logger
use defmt_rtt as _;

use embedded_hal::spi::MODE_0;
use embedded_hal::delay::{DelayNs};

//use embedded_hal_compat::ReverseCompat;

use stm32f4xx_hal as hal;
use hal::{prelude::*};
use hal::gpio::{Speed};
use hal::serial;
use hal::{serial::Serial, spi::Spi, i2c::I2c};


use rand_core::{SeedableRng};
use rand_facade::GlobalRng;
use rand_chacha::ChaChaRng;

use radio_sx128x::prelude::*;
use radio_sx128x::{Config as Sx128xConfig};

use lpwan::prelude::*;
use lpwan::mac_802154::{SyncState, AssocState};

use embedded_graphics::{
    fonts::{Font12x16, Text},
    pixelcolor::BinaryColor,
    prelude::*,
};
use embedded_text::prelude::*;
use ssd1306::{mode::TerminalMode, prelude::*, I2CDisplayInterface, Ssd1306};

use dsf_core::prelude::*;

use dsf_iot::prelude::*;
use dsf_engine::{store::Store};

mod newlib;

mod serial_log;
use serial_log::SerialLogger;

mod timer;
use timer::SystickDelay;

mod common;
use common::{StaticKeyStore, DEVICE_KEYS, HeaplessStore, RadioComms};


lazy_static::lazy_static! {
    pub static ref PUB_ID: Id = Id::from_str("cVeVtbMqQCgl7CtWHk-oYFIarxgp4v63VsNu2ML_ApM=").unwrap();
}

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
const ALLOC_SIZE: usize = 20 * 1024;

const LOG_FILTERS: &'static [&'static str] = &[ "radio_sx128x" ];

const DISPLAY_UPDATE_MS: u64 = 1000;
const JOIN_TIMEOUT: u64 = 10000;

#[entry]
fn main() -> ! {
    // Initialize the allocator
    let start = cortex_m_rt::heap_start() as usize;
    unsafe { ALLOCATOR.init(start, ALLOC_SIZE) }

    let core = cortex_m::peripheral::Peripherals::take().unwrap();
    let device = hal::pac::Peripherals::take().unwrap();

    let gpioa = device.GPIOA.split();
    let gpiob = device.GPIOB.split();
    let _gpioc = device.GPIOC.split();
    let gpiod = device.GPIOD.split();
    let gpioe = device.GPIOE.split();
    let gpiof = device.GPIOF.split();

    // Setup pins
    let mut led0 = gpiob.pb7.into_push_pull_output();

    // Setup system clock
    let rcc = device.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(180.MHz()).require_pll48clk().freeze();

    // Setup serial (and logging)
    let uart_cfg = serial::config::Config::default().baudrate(115_200.bps());
    let uart_tx = gpiod.pd8.into_alternate_af7();
    let uart_rx = gpiod.pd9.into_alternate_af7();
    let uart = Serial::usart3(device.USART3, (uart_tx, uart_rx), uart_cfg, clocks).unwrap();

    SerialLogger::init(Level::Info, Box::new(uart), LOG_FILTERS);

    let mut syst = core.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(18_000-1);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    // Fetch unique chip ID for address use
    let chip_id = hal::signature::Uid::get();
    let chip_id = (chip_id.waf_num() as u64) << 32 | (chip_id.y() as u64) << 16 | (chip_id.x() as u64);

    info!("hello world");

    info!("Starting subscriber with ID: {}", chip_id);

    debug!("Configuring RNG");

    // Setup RNG
    let dev_rng = device.RNG.constrain(clocks);
    let mut chacha_rng = ChaChaRng::from_rng(dev_rng).unwrap();
    let _rng_guard = GlobalRng::set(core::pin::Pin::new(&mut chacha_rng));

    let mut keystore = StaticKeyStore::new();

    debug!("RNG bound");

    // Setup Pins
    let (mut rf_cs, rf_rst, rf_busy, rf_ready, mut rf_ant) = (
        gpiof.pf13.into_push_pull_output(),
        gpioa.pa3.into_push_pull_output(),
        gpioe.pe13.into_floating_input(),
        gpiod.pd13.into_floating_input(),
        gpiof.pf3.into_push_pull_output(),
    );

    let _ = rf_cs.try_set_high();
    let _ = rf_ant.try_set_high();

    // Setup SPI
    let (spi_mosi, spi_miso, spi_sck) = {(
        gpioa.pa7.into_alternate_af5().set_speed(Speed::VeryHigh),
        gpioa.pa6.into_alternate_af5().set_speed(Speed::VeryHigh),
        gpioa.pa5.into_alternate_af5().set_speed(Speed::VeryHigh),
    )};
    let spi = Spi::spi1(device.SPI1, (spi_sck, spi_miso, spi_mosi), MODE_0, 1.MHz().into(), clocks);


    // Setup I2C
    let (i2c_scl, i2c_sda) = {(
        gpiob.pb10.into_alternate_af4().set_speed(Speed::VeryHigh).internal_pull_up(true).set_open_drain(),
        gpiob.pb11.into_alternate_af4().set_speed(Speed::VeryHigh).internal_pull_up(true).set_open_drain(),
    )};

    let i2c = I2c::i2c2(device.I2C2, (i2c_scl, i2c_sda), 100.khz(), clocks);

    // Setup display
    let disp_if = I2CDisplayInterface::new(i2c);
    let mut disp = Ssd1306::new(disp_if, DisplaySize128x64, DisplayRotation::Rotate0);
    disp.init().unwrap();

    let text_style = TextStyleBuilder::new(Font12x16)
    .text_color(BinaryColor::On)
    .build();

    let textbox_style = TextBoxStyleBuilder::new(Font12x16)
        .text_color(BinaryColor::On)
        .height_mode(FitToText)
        .build();

    // Setup a timer object
    //let mut timer = Timer::tim1(device.TIM1, 1.khz(), clocks).unwrap();

    // Create delay object
    let delay = SystickDelay{};

    debug!("Initialising radio");

    // Initialise radio
    let rf_config = common::rf_config();
    let mut radio = match Sx128x::spi(spi, rf_cs, rf_busy, rf_ready, rf_rst, delay, &rf_config) {
        Ok(v) => v,
        Err(e) => {
            panic!("Error initialising radio: {:?}", e);
        }
    };

    // Re-set syncword for GFSK mode
    if let Modem::Gfsk(_gfsk) = &rf_config.modem {
        radio.set_syncword(1, &[0x11, 0x22, 0x33, 0x44, 0x55]).unwrap();
    }


    debug!("Initialising networking");

    // Initialise network stack
    let address = ExtendedAddress(chip_id);
    let mac_config = mac_802154::Config {
        pan_coordinator: false,
        ..Default::default()
    };

    let timer = SystickDelay{};
    let mac = match mac_802154::Mac::new(address, mac_config, radio, timer.clone()) {
        Ok(m) => m,
        Err(e) => {
            panic!("Error initalising MAC: {:?}", e);
        }
    };

    debug!("Initialising 6lo");
    let sixlo_cfg = SixLoConfig{
        ..Default::default()
    };
    let mut sixlo = SixLo::<_, _, 127>::new(mac, MacAddress::Extended(PanId(1), address), sixlo_cfg);


    let info = IotInfo::new(&[]).unwrap();

    let comms = RadioComms::new();
    let store = HeaplessStore::new();

    let keys = DEVICE_KEYS.iter().find(|(c, _k)| *c == chip_id);
    if let Some(k) = &&keys {
        store.set_ident(&k.1);
    }

    let mut engine = match IotEngine::new(info, &[], comms, store) {
        Ok(e) => e,
        Err(e) => {
            error!("Failed to initialise engine: {:?}", e);
            loop {}
        }
    };

    loop {}

    #[cfg(nope)]
    {

    // TODO: set kind to IotService
    let mut sb = ServiceBuilder::default();
    
    let keys = DEVICE_KEYS.iter().find(|(c, _k)| *c == chip_id);
    if let Some(k) = keys {
        sb = sb.private_key(k.1.pri_key.clone().unwrap());
    }

    
    let mut s = sb.body(Body::None).build().unwrap();
    info!("Service ID: {}", s.id().to_string());
 
    let mut buff = [0u8; 1024];
    let (n, _p) = s.publish_primary(&mut buff).unwrap();

    let primary = &buff[..n];

    info!("Generated primary page of {} bytes", n);

    info!("Starting app");

    let mut rf_buff = [0u8; 512];
    let mut net_state = NetState::None;
    let mut sub_state = SubState::None;
    let mut req_id = GlobalRng::get().next_u32() as u16;

    let mut eps = alloc::vec::Vec::new();
    let mut dat = alloc::vec::Vec::new();
    let mut last_display_update = 0;
    let mut last_state = 0;

    loop {        
        let now = SystickDelay{}.ticks_ms();

        // Update the mac
        if let Err(e) = sixlo.tick(now) {
            error!("MAC error: {:?}", e);
        }

        // TODO: check for received packets
        if let Ok(Some((n, addr, _hdr))) = sixlo.receive(now, &mut rf_buff) {
            let payload = &rf_buff[..n];

            trace!("Received {} byte packet", n);
            trace!("Page: {:02x?}", &rf_buff[..n]);

            let (base, _n) = match Base::parse(payload, &keystore) {
                Ok(v) => (v),
                Err(e) => {
                    error!("DSF parsing error: {:?}", e);
                    continue;
                }
            };

            let id = base.id().clone();

            match (NetMessage::convert(base.clone(), &keystore), Page::try_from(base)) {
                (Ok(NetMessage::Request(req)), _) => {
                    // Handle discovery requests
                    if let NetRequestKind::Hello = &req.data {
                        // NOTE: removed for client-based discovery
                    } else if let NetRequestKind::PushData(id, _d) = &req.data {
                        info!("Received push-data for service: {}", id);

                    } else {
                        warn!("Unhandled DSF request");
                    }
                },
                (Ok(NetMessage::Response(resp)), _) => {
                    match (&net_state, &sub_state, &resp.data) {
                        (NetState::Joining(gw), _, NetResponseKind::Status(s)) if *s == dsf_core::net::Status::Ok => {
                            info!("Received join response");
                            net_state = NetState::Joined(*gw);
                        },
                        (NetState::Joined(gw), SubState::Subscribing(req_id), _) => {
                            info!("Received subscribe response");
                            // TODO: handle subscribe response
                            sub_state = SubState::Subscribed;
                        },
                        _ => {
                            warn!("Unhandled DSF response");
                        }
                    }
                },
                (_, Ok(p)) => {
                    // Decode page
                    match p.info().clone() {
                        PageInfo::Primary(pri) => {
                            let ps = IotService::decode_page(p, None).unwrap();

                            info!("Subscribed to service ID: {}", ps.id);

                            keystore.insert(id.clone(), Keys::new(pri.pub_key.clone()));

                            eps = ps.endpoints.clone();

                            info!("Endpoints: ");
                            for i in 0..ps.endpoints.len() {
                                let e = &ps.endpoints[i];

                                info!("  - {:2}: {:?}", i, e.kind);
                            }

                            sub_state = SubState::Subscribed;
                        },
                        PageInfo::Data(_) => {
                            let sd = IotData::decode_page(p, None).unwrap();

                            let mut buff = alloc::string::String::new();

                            dat = sd.data.clone();

                            info!("Received data");
                            for i in 0..sd.data.len() {
                                let ep_data = &sd.data[i];
                    
                                info!("  - {:2}: {:.02}", i, ep_data.value);
                            }
                        },
                        _ => (),
                    }

                },
                _ => {
                    error!("DSF rx was not a network message");
                    continue;
                }
            };


        } else {

            if let (SyncState::Synced(gw), AssocState::Associated(pan)) = sixlo.mac().state() {

                match (&net_state, &sub_state) {
                    (NetState::None, _) => {
                        info!("Issuing register to gw: {:?}", gw);

                        // Transmit encoded primary page
                        if let Err(e) = sixlo.transmit(now, gw, primary) {
                            error!("MAC transmit error: {:?}", e);
                        }
                        // Update net state to joining
                        net_state = NetState::Joining(gw);
                        last_state = now;
                    }
                    (NetState::Joining(_), _) if now > (last_state + JOIN_TIMEOUT) => {
                        info!("Register timeout");
                        net_state = NetState::None;
                        last_state = now;
                    }
                    (NetState::Joined(_), SubState::None) => {
                        // Generate subscribe request
                        let sub_req = NetRequestKind::Subscribe(PUB_ID.clone());
                        let mut req = NetRequest::new(s.id(), req_id, sub_req, Flags::PUB_KEY_REQUEST);
                        req.set_public_key(s.public_key());
    
                        let n = s.encode_message(NetMessage::Request(req), &mut rf_buff).unwrap();
    
                        info!("Issuing subscribe to gw: {:?}", gw);
    
                        if let Err(e) = sixlo.transmit(now, gw, &rf_buff[..n]) {
                            error!("MAC transmit error: {:?}", e);
                        }
    
                        // Update state to subscribing
                        sub_state = SubState::Subscribing(req_id);
                        req_id = req_id.wrapping_add(1);
                    },
                    _ => (),
                }
            } else {
                net_state = NetState::None;
                sub_state = SubState::None;
            }
        }

        if (now / 500) % 2 == 0 {
            led0.try_set_high().unwrap();
        } else {
            led0.try_set_low().unwrap();
        }

        if now > (last_display_update + DISPLAY_UPDATE_MS) {
            
            let mut buff = alloc::string::String::new();

            if dat.len() > 0 && eps.len() > 0 {
                for i in 0..eps.len() {
                    let _ = writeln!(buff, "{:02} {}",  dat[i].value, eps[i].kind.unit());
                }
            } else {
                match (&net_state, &sub_state) {
                    (NetState::None, _) => writeln!(buff, "Searching").unwrap(),
                    (NetState::Joining(_), _) => writeln!(buff, "Connecting").unwrap(),
                    (_, SubState::None) => writeln!(buff, "Pending").unwrap(),
                    (_, SubState::Subscribing(_)) => writeln!(buff, "Subscribing").unwrap(),
                    (_, SubState::Subscribed) => writeln!(buff, "Subscribed").unwrap(),
                }
            }

            let bounds = Rectangle::new(Point::zero(), Point::new(128, 0));
            let text_box = TextBox::new(buff.as_str(), bounds).into_styled(textbox_style);

            disp.clear();
            text_box.draw(&mut disp).unwrap();
            disp.flush().unwrap();

            last_display_update = now;
        }

        SystickDelay{}.try_delay_ms(1u32).unwrap();
    }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub enum NetState {
    None,
    Joining(MacAddress),
    Joined(MacAddress),
}

#[derive(Clone, PartialEq, Debug)]
pub enum SubState {
    None,
    Subscribing(u16),
    Subscribed,
}

use num_traits::float::FloatCore;

fn _roundf(v: f32) -> f32 {
    f32::round(v * 100.0) / 100.0
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    error!("{}", alloc::format!("{}", info));

    loop {}
}




