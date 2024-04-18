use std::{
    collections::HashMap,
    net::UdpSocket,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Instant,
};

use log::{trace, debug, error, info, warn};

use humantime::Duration;
use clap::Parser;

use driver_pal::hal::{DeviceConfig, HalInst};
use embedded_hal::delay::DelayNs;
use linux_embedded_hal::Delay;

use radio_sx128x::prelude::*;
use radio_sx128x::Config as Sx128xConfig;

use lpwan::prelude::*;

use dsf_core::{
    base::DecodeOwned,
    net::{self, Request, Status},
    prelude::*,
    types::BaseKind,
};

use dsf_iot::prelude::*;

#[derive(Debug, Parser)]
struct Options {
    #[clap(flatten)]
    pub spi_config: DeviceConfig,

    /// Peer address for bridging
    #[clap(long, default_value = "127.0.0.1:10100")]
    pub peer_addr: String,

    /// Socket address to bind
    #[clap(long, default_value = "127.0.0.1:40715")]
    pub bind_addr: String,

    #[clap(long, default_value = "10s")]
    pub discovery_period: Duration,

    #[clap(long, hide = true)]
    pub discovery_enabled: bool,

    #[clap(long, default_value = "info")]
    /// Configure radio log level
    pub log_level: simplelog::LevelFilter,
}

#[derive(Clone, Debug)]
pub struct SystemTimer {
    start: Instant,
}

impl SystemTimer {
    fn new() -> Self {
        Self {
            start: Instant::now(),
        }
    }
}

impl MacTimer for SystemTimer {
    fn ticks_ms(&self) -> u64 {
        Instant::now().duration_since(self.start).as_millis() as u64
    }

    fn ticks_us(&self) -> u64 {
        Instant::now().duration_since(self.start).as_micros() as u64
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    // Load options
    let opts = Options::parse();

    // Initialise logging
    let log_cfg = simplelog::ConfigBuilder::new()
        //.add_filter_ignore_str("radio_sx128x")
        .add_filter_ignore_str("driver_cp2130")
        .build();
    let _ = simplelog::SimpleLogger::init(opts.log_level, log_cfg);

    info!("Starting dsf-gateway");

    debug!("Connecting to platform SPI");
    trace!("with config: {:?}", opts.spi_config);
    let HalInst { base: _, spi, pins } = match HalInst::load(&opts.spi_config) {
        Ok(v) => v,
        Err(e) => {
            return Err(anyhow::anyhow!("HAL error: {:?}", e));
        }
    };

    debug!("Initialising Radio");

    // Setup radio for GFSK mode
    #[cfg(feature = "gfsk")]
    let mut rf_config = Sx128xConfig::gfsk();
    #[cfg(not(feature = "gfsk"))]
    let mut rf_config = Sx128xConfig::lora();

    if let Modem::Gfsk(gfsk) = &mut rf_config.modem {
        gfsk.patch_preamble = false;
        gfsk.crc_mode = radio_sx128x::device::common::GfskFlrcCrcModes::RADIO_CRC_2_BYTES;
    }
    if let Channel::Gfsk(gfsk) = &mut rf_config.channel {
        gfsk.br_bw = radio_sx128x::device::common::GfskBleBitrateBandwidth::BR_0_800_BW_1_2;
    } else if let Channel::LoRa(lora) = &mut rf_config.channel {
        lora.sf = radio_sx128x::device::lora::LoRaSpreadingFactor::Sf7;
        lora.bw = radio_sx128x::device::lora::LoRaBandwidth::Bw1600kHz;
    }

    // Initialise radio
    let mut radio = match Sx128x::spi(
        spi,
        pins.busy,
        pins.ready,
        pins.reset,
        Delay {},
        &rf_config,
    ) {
        Ok(v) => v,
        Err(e) => {
            return Err(anyhow::anyhow!("Radio init error: {:?}", e));
        }
    };
    // Re-set syncword
    if let Modem::Gfsk(_gfsk) = &mut rf_config.modem {
        radio
            .set_syncword(1, &[0x11, 0x22, 0x33, 0x44, 0x55])
            .unwrap();
    }

    debug!("Binding socket ({:?})", opts.bind_addr);
    let udp_socket = UdpSocket::bind(opts.bind_addr)?;
    udp_socket.connect(opts.peer_addr)?;
    udp_socket.set_nonblocking(true)?;

    info!("Bound to: {:?}", udp_socket.local_addr());

    // Initialise network stack

    // Generate configuration
    let address = ExtendedAddress(rand::random::<u64>() % 1000);
    let mac_config = mac_802154::Config {
        pan_coordinator: true,
        ..Default::default()
    };

    debug!("Initialising MAC");

    // Initialise MAC
    let timer = SystemTimer::new();
    let mac = match mac_802154::Mac::new(address, mac_config, radio, timer.clone()) {
        Ok(m) => m,
        Err(e) => {
            return Err(anyhow::anyhow!("Error initalising MAC: {:?}", e));
        }
    };

    debug!("Initialising 6lo");
    let sixlo_cfg = SixLoConfig {
        ..Default::default()
    };
    let mut sixlo = SixLo::<_, 127>::new(mac, MacAddress::Extended(PanId(1), address), sixlo_cfg);

    debug!("Initialising DSF");

    // TODO: could use local daemon service for this..? or passthrough objects directly?
    let service = match ServiceBuilder::<Vec<u8>>::default().build() {
        Ok(s) => s,
        Err(e) => {
            return Err(anyhow::anyhow!("Error creating service: {:?}", e));
        }
    };

    let mut devices: HashMap<Id, (MacAddress, Service)> = HashMap::new();
    let mut keys: HashMap<Id, Keys> = HashMap::new();
    let mut reqs: HashMap<RequestId, Id> = HashMap::new();
    let mut subs: HashMap<Id, Id> = HashMap::new();

    let mut last_discovery = Instant::now();

    let mut rf_buff = [0u8; 512];

    let mut req_id = 0;

    // Handshake with DSF daemon to exchange keys
    let mut req = net::Request::new(
        service.id(),
        req_id,
        net::RequestBody::Hello,
        Flags::PUB_KEY_REQUEST,
    );
    req.set_public_key(service.public_key());

    let c: Container<[u8; 1024]> = service.encode_request_buff(&req, &Default::default())?;
    udp_socket.send(c.raw())?;

    debug!("Starting loop");

    while running.load(Ordering::SeqCst) {
        let now = timer.ticks_ms();

        // Tick the mac
        if let Err(e) = sixlo.tick(now) {
            error!("MAC error: {:?}", e);
        }

        // Check for received RF packets
        if let Ok(Some((n, addr, _header))) = sixlo.receive(now, &mut rf_buff) {
            debug!("Received data: {:02x?}", &rf_buff[..n]);

            // Parse out base object
            let base = match Container::parse(&mut rf_buff[..n], &keys) {
                Ok(v) => v,
                Err(e) => {
                    error!("Error parsing incoming packet: {:?}", e);
                    continue;
                }
            };
            let id = base.id().clone();
            let kind = base.header().kind().base();

            debug!("Received: {:?}", base);

            // TODO: handle other (subscribe / unsubscribe) requests here

            match (devices.contains_key(&id), kind) {
                // Handle pages for unknown services
                (false, BaseKind::Page) => {
                    let s = match Service::load(&base) {
                        Ok(s) => {
                            info!("Registered service: {:?}", id);
                            s
                        }
                        Err(e) => {
                            error!("Error loading service: {:?}", e);
                            continue;
                        }
                    };

                    if !base.encrypted() {
                        let endpoint_info = IotInfo::<8>::decode_owned(base.body_raw()).unwrap();

                        info!("Endpoints: {:?}", endpoint_info);
                    }

                    devices.insert(id.clone(), (addr, s.clone()));

                    let k = Keys::new(s.public_key());
                    keys.insert(id, k);

                    // Setup register request
                    let req_body = net::RequestBody::Register(s.id(), vec![base.to_owned()]);
                    let mut req =
                        net::Request::new(service.id(), req_id, req_body, Flags::PUB_KEY_REQUEST);
                    req.set_public_key(service.public_key());

                    // Register request to service ID for response handling
                    reqs.insert(req_id, s.id());

                    // Issue request
                    let c = service.encode_request_buff::<1024>(&req, &Default::default())?;
                    udp_socket.send(c.raw())?;

                    req_id = req_id.wrapping_add(1);
                }
                // Handle pages for known services (update local register)
                (true, BaseKind::Page) => {
                    let (_addr, s) = devices.get_mut(&id).unwrap();

                    match s.apply_primary(&base) {
                        Ok(true) => {
                            info!("Updated service: {:?}", id);
                        }
                        Ok(false) => (),
                        Err(e) => {
                            error!("Error updating service: {:?}", e);
                            continue;
                        }
                    }
                }
                // Handle data for known services
                (true, BaseKind::Data) => {
                    let (_addr, s) = devices.get_mut(&id).unwrap();

                    // Decode IoT data
                    if !base.encrypted() {
                        let endpoint_data = IotData::<8>::decode_owned(base.body_raw()).unwrap();

                        info!("Received device {} data: {:?}", s.id(), endpoint_data);

                        // TODO: something with this
                    }

                    // Setup data push request
                    let req_body = net::RequestBody::PushData(s.id(), vec![base.to_owned()]);
                    let mut req =
                        net::Request::new(service.id(), req_id, req_body, Flags::PUB_KEY_REQUEST);
                    req.set_public_key(service.public_key());

                    // Register request to service ID for response handling
                    reqs.insert(req_id, s.id());

                    // Issue request
                    let c = service.encode_request_buff::<1024>(&req, &Default::default())?;
                    udp_socket.send(c.raw())?;

                    req_id = req_id.wrapping_add(1);
                }
                // Forward messages to the daemon
                (_, BaseKind::Request | BaseKind::Response) => {
                    if let Ok(m) = NetMessage::convert(base, &keys) {
                        // TODO: register subscribe / unsubscribe paths
                        if let NetMessage::Request(req) = &m {
                            match &req.data {
                                net::RequestBody::Subscribe(sub_id) => {
                                    info!("Add subscription to {} for {}", sub_id, id);
                                    subs.insert(sub_id.clone(), id.clone());
                                }
                                _ => (),
                            }
                        }

                        // Forward to daemon
                        reqs.insert(m.request_id(), m.from());
                        udp_socket.send(&rf_buff[..n])?;
                    }
                }
                _ => unimplemented!(),
            }

            continue;
        }

        // Check for received network packets
        let mut udp_buff = [0u8; 4096];
        match udp_socket.recv(&mut udp_buff) {
            Ok(n) => {
                // Parse DSF data and convert to message type
                let (msg, _n) = NetMessage::parse(&mut udp_buff[..n], &keys).unwrap();

                // Lookup request to forward response
                info!("Received net message: {:?}", msg);

                if let Some(pk) = msg.pub_key() {
                    keys.insert(msg.from(), Keys::new(pk.clone()));
                }

                match &msg {
                    NetMessage::Request(req) => {
                        info!("Receive DSF request: {:?}", req);

                        // Handle pending requests
                        if let net::RequestBody::PushData(sub_id, data) = &req.data {
                            if let Some(device) =
                                subs.get(sub_id).map(|id| devices.get(&id)).flatten()
                            {
                                // Forward data object
                                if let Some(d) = data.iter().find(|p| p.header().kind().is_data()) {
                                    info!(
                                        "Forward page {} for service {} to {}",
                                        d.header().index(),
                                        sub_id,
                                        device.1.id()
                                    );

                                    let raw = d.raw().as_ref();

                                    if let Err(e) = sixlo.transmit(now, device.0, &raw) {
                                        error!("MAC transmit error: {:?}", e);
                                    }

                                    // Respond to daemon
                                    // TODO: what's up with this..?
                                    let _ack = net::Response::new(
                                        service.id(),
                                        req.id,
                                        net::ResponseBody::Status(Status::Ok),
                                        Flags::empty(),
                                    );

                                    let c = service
                                        .encode_request_buff::<1024>(&req, &Default::default())?;
                                    udp_socket.send(&c.raw())?;
                                }
                            } else {
                                warn!("No subscriber registered for service: {}", sub_id);
                            }
                        } else if let Some(device) =
                            reqs.remove(&req.id).map(|id| devices.get(&id)).flatten()
                        {
                            info!("Forwarding request {:?} to {:?}", req.data, device.0);

                            // Directly forward to client
                            if let Err(e) = sixlo.transmit(now, device.0, &udp_buff[..n]) {
                                error!("MAC transmit error: {:?}", e);
                            }
                        } else {
                            info!("Unhandled request: {:?}", req);
                        }
                    }
                    NetMessage::Response(resp) => {
                        // Locate device with pending request
                        let device =
                            match reqs.remove(&resp.id).map(|id| devices.get(&id)).flatten() {
                                Some(r) => r,
                                None => {
                                    warn!("No pending request for id: {}", resp.id);
                                    continue;
                                }
                            };

                        match &resp.data {
                            // Strip primary page from ValuesFound message
                            net::ResponseBody::ValuesFound(_id, pages) => {
                                if let Some(p) = pages.iter().find(|p| p.header().kind().is_page())
                                {
                                    let raw = p.raw().as_ref();

                                    if let Err(e) = sixlo.transmit(now, device.0, &raw) {
                                        error!("MAC transmit error: {:?}", e);
                                    }
                                }
                            }
                            // Forward other messages
                            _ => {
                                if let Err(e) = sixlo.transmit(now, device.0, &udp_buff[..n]) {
                                    error!("MAC transmit error: {:?}", e);
                                }
                            }
                        }
                    }
                }

                // Forward response to peer
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => (),
            Err(e) => return Err(e.into()),
        }

        // Send discovery messages
        if opts.discovery_enabled {
            if Instant::now().duration_since(last_discovery) > *opts.discovery_period {
                info!("Sending discovery message");

                // Setup discovery request
                let mut req = Request::new(
                    service.id(),
                    0,
                    net::RequestBody::Hello,
                    Flags::PUB_KEY_REQUEST,
                );
                req.set_public_key(service.public_key());

                // Encode request
                let keys = service.keys();
                let c = service.encode_request_buff::<1024>(&req, &Default::default())?;

                // Broadcast
                if let Err(e) =
                    sixlo.transmit(now, MacAddress::broadcast(&AddressMode::Short), &c.raw())
                {
                    error!("MAC transmit error: {:?}", e);
                }

                last_discovery = Instant::now();
            }

            // TODO: decode and handle these
            Delay {}.delay_ms(10);
        }
    }

    Ok(())
}
