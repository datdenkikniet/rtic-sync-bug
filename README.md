# rtic-sync bug repro

this reproduces an rtic-sync bug on an stm32f411 nucleo

# steps to reproduce

1. plug in an stm32f411 nucleo (other stm32f4 models may work as well)
2. run `cargo run`
3. wait for this:

```bash
<lvl> Received a message: 123
└─ rtic_sync_bug_stm32f4::app::spam_queuer::{async_fn#0} @ src/main.rs:119 
<lvl> Received a message: 124
└─ rtic_sync_bug_stm32f4::app::spam_queuer::{async_fn#0} @ src/main.rs:119 
<lvl> Received a message: 125
└─ rtic_sync_bug_stm32f4::app::spam_queuer::{async_fn#0} @ src/main.rs:119 
<lvl> Received a message: 126
└─ rtic_sync_bug_stm32f4::app::spam_queuer::{async_fn#0} @ src/main.rs:119 
<lvl> Received a message: 127
└─ rtic_sync_bug_stm32f4::app::spam_queuer::{async_fn#0} @ src/main.rs:119 
ERROR panicked at $HOME/.cargo/registry/src/index.crates.io-6f17d22bba15001f/rtic-sync-1.3.0/src/channel.rs:315:17:
assertion failed: !self.0.access(cs).freeq.is_empty()
└─ panic_probe::print_defmt::print @ $HOME/.cargo/registry/src/index.crates.io-6f17d22bba15001f/panic-probe-0.3.2/src/lib.rs:104 
```

it may take up to a minute to occur, but it will typically happen within 30 seconds.

RustC version: rustc 1.85.0 (4d91de4e4 2025-02-17)