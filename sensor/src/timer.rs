
// AN4013
// AN2592
// http://www.proiotware.com/index.php/9-blogs/9-stm32-chaining-two-16-bit-timers-to-create-32-bit-timer


use core::sync::atomic::{AtomicUsize, Ordering};

use cortex_m_rt::{exception};


use embedded_hal::delay::{DelayNs};
use lpwan::timer::Timer;

static SYSTICK_COUNT: AtomicUsize = AtomicUsize::new(0);

#[exception]
fn SysTick() {
    SYSTICK_COUNT.fetch_add(1, Ordering::Relaxed);
}

#[derive(Clone, Debug)]
pub struct SystickDelay {}

impl DelayNs for SystickDelay {
    fn delay_ns(&mut self, ms: u32) -> Result<(), Self::Error> {
        let start = SYSTICK_COUNT.load(Ordering::Relaxed) as u32 / 10;

        crate::trace!("Starting delay at {} for {} ms", start, ms);

        loop {
            let now = SYSTICK_COUNT.load(Ordering::Relaxed) as u32 / 10;

            if now > (start + ms) {
                crate::trace!("Delay expired at {}", now);
                break;
            }

            if now < start {
                crate::warn!("Delay wrap error");
                break;
            }
        }

        Ok(())
    }
}

impl Timer for SystickDelay {
    fn ticks_ms(&self) -> u64 {
        SYSTICK_COUNT.load(Ordering::Relaxed) as u64 / 10u64
    }

    fn ticks_us(&self) -> u64 {
        SYSTICK_COUNT.load(Ordering::Relaxed) as u64 * 1000 / 10u64
    }
}


#[cfg(nope)]
mod chain {
    use stm32f4xx_hal::time::Hertz;

    use stm32f4xx_hal::stm32::{TIM2, TIM3};

    struct TimerChain<T1, T2> {
        ta: T1,
        tb: T2,
    }


    impl TimerChain<TIM2, TIM3> {
        fn new(ta: TIM2, tb: TIM3) -> Self {
            Self {
                ta, tb,
            }
        }

        fn configure<T: Into<Hertz>>(&mut self, prescaler: T) {
            // pause
            self.ta.cr1.modify(|_, w| w.cen().clear_bit());
            self.tb.cr1.modify(|_, w| w.cen().clear_bit());

            // reset counters
            self.ta.cnt.reset();
            self.tb.cnt.reset();

            // Setup timer A as controller

            // Set timer in upcounting mode
            self.ta.cr1.modify(|_, w| w.dir().up() );

            // TODO: Setup timer prescaler and top count
            //let pclk = self.ta.pclk();

            // Select trigger output in CR2 MSM (MMS) register
            self.ta.cr2.modify(|_, w| w.mms().update() );

            // Use UPDATE event
            self.ta.egr.write(|w| w.tg().set_bit().ug().set_bit() );

            // Enable MSM bit in SMCR to allow sync via TRGO
            self.ta.smcr.modify(|_, w| w.msm().sync() );


            // Setup timer B as slave

            // Set timer in upcounting mode
            self.tb.cr1.modify(|_, w| w.dir().up() );

            // Configure SMCR for external clock mode
            self.tb.smcr.modify(|_, w| w.sms().ext_clock_mode() );

            // Configure internal trigger channel
            self.tb.smcr.modify(|_, w| w.ts().itr0() );
        }

        fn count(&self) -> (u32, u32) {
            unimplemented!()
        }
    }
}