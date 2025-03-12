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

#[app(device = pac, peripherals = true, dispatchers = [RTC_WKUP, USART1, USART2, I2C1_ER, I2C1_EV])]
mod app {
    use stm32f7xx_hal::{
        pac::TIM4,
        rcc::{HSEClock, HSEClockMode},
        timer,
    };

    use crate::*;

    systick_monotonic!(Mono, 1000);

    pub type MsgOut = Sender<'static, [u8; 20], 32>;
    pub type MsgIn = Receiver<'static, [u8; 20], 32>;
    pub type SemaphoreSend = Sender<'static, u8, 3>;
    pub type SemaphoreRecv = Receiver<'static, u8, 3>;

    #[shared]
    pub struct Shared {}

    #[local]
    pub struct Local {
        pub timer: timer::Counter<TIM4, 1000000>,
        pub semaphore_recv: SemaphoreRecv,
    }

    #[init()]
    fn init(cx: init::Context) -> (Shared, Local) {
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(100u32.MHz())
            .hclk(100u32.MHz())
            .hse(HSEClock::new(8.MHz(), HSEClockMode::Bypass))
            .freeze();

        // the JTAG bits are write-only (stupid)
        let (spam_tx_send, spam_tx_recv) = make_channel!([u8; 20], 32);
        let (semaphore_send, semaphore_recv) = make_channel!(u8, 3);

        let mut timer: timer::Counter<TIM4, 1000000> = cx.device.TIM4.counter(&clocks);
        //.start_count_down(10_u32.millis());
        timer.listen(timer::Event::Update);
        timer.start(10_u32.millis()).unwrap();

        Mono::start(cx.core.SYST, clocks.sysclk().to_Hz());
        spam_queuer::spawn(spam_tx_recv, semaphore_send).ok();
        spam_sender::spawn(spam_tx_send.clone()).ok();

        (
            Shared {},
            Local {
                timer,
                semaphore_recv,
            },
        )
    }

    #[task(priority = 13, binds = TIM4, local=[timer, semaphore_recv])]
    fn timer_isr(cx: timer_isr::Context) {
        let randomness = (Mono::now().duration_since_epoch().ticks() & 0b111111) as u32;
        cx.local.timer.cancel().ok();

        // set the timer "randomly"
        cx.local
            .timer
            .start(fugit::ExtU32::micros(100 + randomness))
            .unwrap();
        // clear a semaphore slot
        cx.local.semaphore_recv.try_recv().ok();
    }

    // Tasks
    #[task(priority = 6)]
    async fn spam_sender(_cx: spam_sender::Context, mut outbound_queue: MsgOut) {
        loop {
            for i in 0u8..255 {
                outbound_queue.send([i; 20]).await.ok();
            }
            Mono::delay(1u64.millis()).await;
        }
    }

    #[task(priority = 4)]
    async fn spam_queuer(
        _cx: spam_queuer::Context,
        mut outbound_queue: MsgIn,
        mut semaphore_send: SemaphoreSend,
    ) {
        while let Ok(msg) = outbound_queue.recv().await {
            // we enforce a timeout here so the outbound message handler won't hang if the timer irq never clears enough
            defmt::info!("Received a message: {}", msg[0]);
            defmt::println!("Before mono");
            Mono::timeout_after(4_u64.millis(), semaphore_send.send(0u8))
                .await
                .ok();
            defmt::println!("After mono");
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
