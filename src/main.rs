#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use fugit::ExtU64;
use rtic::app;
use rtic_monotonics::systick::prelude::*;
use rtic_monotonics::systick_monotonic;
use rtic_sync::channel::{Receiver, Sender};
use rtic_sync::make_channel;
use rtic_sync::portable_atomic::{AtomicUsize, Ordering};
use stm32f7xx_hal::pac;
use stm32f7xx_hal::prelude::*;

#[app(device = pac, peripherals = true, dispatchers = [SPI1, SPI2, SPI3, SPI4])]
mod app {
    use fugit::Duration;
    use stm32f7xx_hal::{
        pac::TIM4,
        rcc::{HSEClock, HSEClockMode},
        timer,
    };

    use crate::*;

    systick_monotonic!(Mono, 1_000);

    pub type MsgOut = Sender<'static, [u8; 20], 32>;
    pub type MsgIn = Receiver<'static, [u8; 20], 32>;
    pub type SemaphoreSend = Sender<'static, u8, 3>;
    pub type SemaphoreRecv = Receiver<'static, u8, 3>;

    #[shared]
    pub struct Shared {}

    #[local]
    pub struct Local {
        // pub timer: timer::Counter<TIM4, 1000000>,
        pub semaphore_recv: SemaphoreRecv,
    }

    #[init()]
    fn init(cx: init::Context) -> (Shared, Local) {
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(96u32.MHz())
            .hclk(96u32.MHz())
            .hse(HSEClock::new(8.MHz(), HSEClockMode::Bypass))
            .freeze();

        // the JTAG bits are write-only (stupid)
        let (spam_tx_send, spam_tx_recv) = make_channel!([u8; 20], 32);
        let (semaphore_send, semaphore_recv) = make_channel!(u8, 3);

        Mono::start(cx.core.SYST, clocks.sysclk().to_Hz());
        spam_queuer::spawn(spam_tx_recv, semaphore_send).ok();
        spam_sender::spawn(spam_tx_send.clone()).ok();
        timer_isr::spawn().ok();

        (
            Shared {},
            Local {
                // timer,
                semaphore_recv,
            },
        )
    }

    #[task(priority = 4, local=[semaphore_recv])]
    async fn timer_isr(cx: timer_isr::Context) {
        loop {
            let randomness = (Mono::now().duration_since_epoch().ticks() & 0b1) as u64;

            // set the timer "randomly"
            Mono::delay((1 + randomness).millis()).await;

            // clear a semaphore slot
            cx.local.semaphore_recv.try_recv().ok();
        }
    }

    // Tasks
    #[task(priority = 2)]
    async fn spam_sender(_cx: spam_sender::Context, mut outbound_queue: MsgOut) {
        loop {
            for i in 0u8..255 {
                outbound_queue.send([i; 20]).await.ok();
            }
            Mono::delay(1u64.millis()).await;
        }
    }

    #[task(priority = 1)]
    async fn spam_queuer(
        _cx: spam_queuer::Context,
        mut outbound_queue: MsgIn,
        mut semaphore_send: SemaphoreSend,
    ) {
        while let Ok(msg) = outbound_queue.recv().await {
            // we enforce a timeout here so the outbound message handler won't hang if the timer irq never clears enough
            defmt::println!("Received a message: {}", msg[0]);
            Mono::timeout_after(4u64.millis(), semaphore_send.send(0u8))
                .await
                .ok();
        }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

pub fn reboot() -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}
