# IGNITER

Synchronized 4-channel igniter primitives for Raspberry Pi runtimes.

Main components:

- `VN5E160S`: low-level fault filtering and latch behavior
- `Igniter`: non-blocking arm/fire/off state machine
- `IgniterBank`: coordinated 4-channel control with batch GPIO writes

The hardware backend uses libgpiod when available at build time.
