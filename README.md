# pico-mixer-rs
A physical audio mixer for Linux using a Raspberry Pi Pico and rotary potentiometers. The microcontroller firmware is written in Rust (Embassy/RP2040-HAL) and communicates via serial USB to a host Python daemon. The host script utilizes pulsectl to map individual physical knobs to specific application volume streams for granular control.
