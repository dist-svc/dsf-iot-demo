use core::cell::LazyCell;
use core::fmt::Debug;
use core::marker::PhantomData;
use core::str::FromStr;

use dsf_core::keys::{KeySource, Keys};
use dsf_core::prelude::*;

use dsf_engine::store::ObjectFilter;
use dsf_engine::{
    comms::Comms,
    store::{ObjectInfo, Peer, Store, StoreFlags},
};

use heapless::sorted_linked_list::Iter;
use radio_sx128x::prelude::*;
use radio_sx128x::Config as Sx128xConfig;

pub fn device_keys() -> [(u64, Keys); 2] {
    [
        (30068834336, Keys {
            pub_key: PublicKey::from_str("BRPdsZXDnr6jfWZNVCDWw8s2faTMhfhv2T9u3L1TT2ix").ok(),
            pri_key: PrivateKey::from_str("46YQK7Vscpwwh9EFuF72bAvNDTvjM9fXthmQjVgw8svnZ6VRPtLLB4cYyfhB6qaGzCqQaJVc4YYJ1HEh5jr79J5h").ok(),
            sec_key: SecretKey::from_str("Er4sf4TU7qrrbX4ziNZJCLwvahnku6MEzRtSsvARkF7L").ok(),
            sym_keys: None,
        }),
        (17182883907, Keys{
            pub_key: PublicKey::from_str("86Zaq2E4T1B2Sm5G2YFmp5eNrWLYLLeEnvetmWb86mNy").ok(),
            pri_key: PrivateKey::from_str("37G3aJhdhNTFRnqEVAb9Ku7UwxNxLa2gXetP9FYHDApefkc5j27aBjrqEJ5enpxyHd9HQDEK4zb7UQJxf1nGo6F9").ok(),
            sec_key: SecretKey::from_str("7516cAWs3wazvDZLEF6mQGJ5af58Ef4CkEAqE3oTnWvj").ok(),
            sym_keys: None,
        }),
    ]
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
                Some(v) if &v.0 == id => return Some(v.1.clone()),
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

pub struct RadioComms {}

impl RadioComms {
    pub fn new() -> Self {
        Self {}
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
    pub peers: heapless::Vec<(Id, Peer<Addr>), 8>,
    pub pages: heapless::LinearMap<Signature, Container, 16>,
    _addr: PhantomData<Addr>,
}

impl<Addr: Clone + Debug + 'static> HeaplessStore<Addr> {
    pub fn new() -> Self {
        Self {
            our_keys: None,
            last_sig: None,
            peers: heapless::Vec::new(),
            pages: heapless::LinearMap::new(),
            _addr: PhantomData,
        }
    }
}

impl<Addr: Clone + Debug + 'static> Store for HeaplessStore<Addr> {
    const FEATURES: StoreFlags = StoreFlags::empty();

    type Address = Addr;

    type Error = ();

    type Iter<'a> = PeerIter<'a, Addr>;

    fn get_ident(&self) -> Result<Option<Keys>, Self::Error> {
        Ok(self.our_keys.clone())
    }

    fn set_ident(&mut self, keys: &Keys) -> Result<(), Self::Error> {
        self.our_keys = Some(keys.clone());
        Ok(())
    }

    fn get_service(&self, id: &Id) -> Result<Option<Service>, Self::Error> {
        Ok(None)
    }

    fn update_service<R: Debug, F: Fn(&mut Service) -> R>(
        &mut self,
        id: &Id,
        f: F,
    ) -> Result<R, Self::Error> {
        Err(())
    }

    fn get_last(&self) -> Result<Option<ObjectInfo>, Self::Error> {
        Ok(self.last_sig.clone())
    }

    #[cfg(nyet)]
    fn set_last(&mut self, info: &ObjectInfo) -> Result<(), Self::Error> {
        self.last_sig = Some(info.clone());
        Ok(())
    }

    fn get_peer(&self, id: &Id) -> Result<Option<Peer<Self::Address>>, Self::Error> {
        todo!()
    }

    fn update_peer<R: core::fmt::Debug, F: Fn(&mut Peer<Self::Address>) -> R>(
        &mut self,
        id: &Id,
        f: F,
    ) -> Result<R, Self::Error> {
        todo!()
    }

    fn peers<'a>(&'a self) -> Self::Iter<'a> {
        PeerIter {
            index: 0,
            peers: &self.peers,
        }
    }

    fn store_page<T: dsf_core::types::ImmutableData>(
        &mut self,
        p: &dsf_core::wire::Container<T>,
    ) -> Result<(), Self::Error> {
        todo!()
    }

    fn fetch_page<T: MutableData>(
        &mut self,
        f: ObjectFilter,
        mut buff: T,
    ) -> Result<Option<dsf_core::wire::Container<T>>, Self::Error> {
        todo!()
    }
}

impl<Addr: Clone + Debug> KeySource for HeaplessStore<Addr> {
    fn keys(&self, id: &Id) -> Option<Keys> {
        todo!()
    }
}

pub struct PeerIter<'a, Addr: Clone + Debug> {
    peers: &'a heapless::Vec<(Id, Peer<Addr>), 8>,
    index: usize,
}

impl<'a, Addr: Clone + Debug> Iterator for PeerIter<'a, Addr> {
    type Item = (&'a Id, &'a Peer<Addr>);

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.peers.len() {
            return None;
        }

        let p = &self.peers[self.index];

        self.index += 1;

        Some((&p.0, &p.1))
    }
}
