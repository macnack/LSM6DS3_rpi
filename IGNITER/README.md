# IGNITER

Synchronized 4-channel igniter primitives for Raspberry Pi runtimes.

Main components:

- `VN5E160S`: low-level fault filtering and latch behavior
- `Igniter`: non-blocking arm/fire/off state machine
- `IgniterBank`: coordinated 4-channel control with batch GPIO writes

The hardware backend uses libgpiod when available at build time.

## C++ examples

Built executables:

- `igniter_vn5e160s_demo`
- `igniter_state_machine_demo`
- `igniter_bank_demo`

Run from repo root after build:

```bash
./build/IGNITER/igniter_vn5e160s_demo
./build/IGNITER/igniter_state_machine_demo
./build/IGNITER/igniter_bank_demo
```

## Python CLI

`igniter_rpi` exposes one CLI with subcommands for all three classes:

- `vn5e160s`
- `igniter`
- `bank`

Examples:

```bash
igniter-demo vn5e160s --status-seq 1,1,1,0,1 --step-ms 5
igniter-demo igniter --duration-ms 200 --status-seq 1,1,1,1,1,1 --step-ms 40
igniter-demo bank --mask 0x0F --duration-ms 250 --steps 12 --step-ms 30
igniter-demo bank --fault-channel 2 --fault-step 5
```
