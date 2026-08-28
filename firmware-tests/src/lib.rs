#![no_std]

// The production firmware includes this module directly.  Keeping the test
// package dependency-free makes it runnable on the native Rust test harness
// while exercising the exact production packet and ownership code.
#[allow(
    dead_code,
    reason = "production firmware uses these functions; this package only executes their tests"
)]
#[path = "../../src/controller.rs"]
mod controller;

#[cfg(test)]
extern crate std;
