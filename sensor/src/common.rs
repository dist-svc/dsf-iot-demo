
use core::marker::PhantomData;
use core::str::FromStr;
use core::fmt::Debug;

use dsf_core::prelude::*;
use dsf_core::keys::{Keys, KeySource};

use dsf_engine::{comms::Comms, store::{Store, ObjectInfo, Peer, StoreFlags}};

use radio_sx128x::prelude::*;
use radio_sx128x::{Config as Sx128xConfig};

lazy_static::lazy_static! {
    pub static ref DEVICE_KEYS: [(u64, Keys); 2] = [
        (30068834336, Keys {
            pub_key: PublicKey::from_str("oStasHnvHkXQ5SAMKRvFUMxxrwnEPmuGrgsJ-t8J520=").ok(),
            pri_key: PrivateKey::from_str("ui5TZvy5NCHkHK7HuNJkhDQDKcLKXVnmvcNKsl4I3lehK1qwee8eRdDlIAwpG8VQzHGvCcQ-a4auCwn63wnnbQ==").ok(),
            sec_key: SecretKey::from_str("QeXiB8ogpgecLEHSlXUP-LVxim_Rlf8qwDf40zke5So=").ok(),
            sym_keys: None,
        }),
        (17182883907, Keys{
            pub_key: PublicKey::from_str("W-tdno4Js2qvQTsUwJMb6Ga4xEMCiJIqWwHpwokiXGo=").ok(),
            pri_key: PrivateKey::from_str("Dtaqdqxj0KQmoySJyZ-5B9AI1z99-Jp49YqC8M635ndb612ejgmzaq9BOxTAkxvoZrjEQwKIkipbAenCiSJcag==").ok(),
            sec_key: SecretKey::from_str("lcXASg0RQX1_d3qW82S4aOlDx0WZMXgVjbZmfQWUA6Q=").ok(),
            sym_keys: None,
        }),
    ];
}

pub struct StaticKeyStore {
    k: [Option<(Id, Keys)>; 12],
}

impl StaticKeyStore {
    pub fn new() -> Self {
        Self {
            k: Default::default(),
        }
    }

    pub fn insert(&mut self, id: Id, keys: Keys) {
        for e in &mut self.k {
            if e.is_none() {
                *e = Some((id, keys));
                return;
            }
        }
    }
}

impl KeySource for StaticKeyStore {
    fn keys(&self, id: &Id) -> Option<Keys> {
        for k in &self.k {
            match &k {
                Some(v) if &v.0 == id => {
                    return Some(v.1.clone())
                },
                _ => (),
            }
        }
        return None;
    }
}

pub fn rf_config() -> Sx128xConfig {
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

    rf_config
}

pub struct RadioComms {

}

impl RadioComms {
    pub fn new() -> Self {
        Self{}
    }
}

impl Comms for RadioComms {
    type Address = u16;

    type Error = ();

    fn recv(&mut self, buff: &mut [u8]) -> Result<Option<(usize, Self::Address)>, Self::Error> {
        todo!()
    }

    fn send(&mut self, to: &Self::Address, data: &[u8]) -> Result<(), Self::Error> {
        todo!()
    }

    fn broadcast(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        todo!()
    }
}

pub struct HeaplessStore<Addr: Clone + Debug + 'static> {
    pub our_keys: Option<Keys>,
    pub last_sig: Option<ObjectInfo>,
    pub peers: heapless::LinearMap<Id, Peer<Addr>, 8>,
    pub pages: heapless::LinearMap<Signature, Container, 16>,
    _addr: PhantomData<Addr>,
}

impl <Addr: Clone + Debug + 'static> HeaplessStore<Addr> {
    pub fn new() -> Self {
        Self {
            our_keys: None,
            last_sig: None,
            peers: heapless::LinearMap::new(),
            pages: heapless::LinearMap::new(),
            _addr: PhantomData,
        }
    }
}

#[cfg(nyet)]
impl <Addr: Clone + Debug + 'static> Store for HeaplessStore<Addr> {
    const FEATURES: StoreFlags = StoreFlags::empty();

    type Address = Addr;

    type Error = ();

    type Iter<'a> = todo!();

    fn get_ident(&self) -> Result<Option<Keys>, Self::Error> {
        Ok(self.our_keys)
    }

    fn set_ident(&mut self, keys: &Keys) -> Result<(), Self::Error> {
        self.our_keys = Some(keys.clone());
        Ok(())
    }

    fn get_last(&self) -> Result<Option<ObjectInfo>, Self::Error> {
        Ok(self.last_sig)
    }

    #[cfg(nyet)]
    fn set_last(&mut self, info: &ObjectInfo) -> Result<(), Self::Error> {
        self.last_sig = Some(info.clone());
        Ok(())
    }

    fn get_peer(&self, id: &Id) -> Result<Option<Peer<Self::Address>>, Self::Error> {
        todo!()
    }

    fn update_peer<R: core::fmt::Debug, F: Fn(&mut Peer<Self::Address>)-> R>(&mut self, id: &Id, f: F) -> Result<R, Self::Error> {
        todo!()
    }

    fn peers<'a>(&'a self) -> Self::Iter<'a> {
        self.peers.iter()
    }

    fn store_page<T: dsf_core::types::ImmutableData>(&mut self, sig: &Signature, p: &dsf_core::wire::Container<T>) -> Result<(), Self::Error> {
        todo!()
    }

    fn fetch_page(&mut self, sig: &Signature) -> Result<Option<dsf_core::wire::Container>, Self::Error> {
        todo!()
    }
}

impl <Addr: Clone + Debug> KeySource for HeaplessStore<Addr> {
    fn keys(&self, id: &Id) -> Option<Keys> {
        todo!()
    }
}