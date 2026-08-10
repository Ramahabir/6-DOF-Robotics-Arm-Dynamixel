# STM32F411 6-DOF Robot Arm Controller

Firmware for a 6-degree-of-freedom robot arm based on an STM32F411CEU6 (Black Pill). The controller is designed to operate four Dynamixel joints over a half-duplex serial bus and two conventional PWM servos for the end effector.

The project uses STM32Cube HAL with PlatformIO and is uploaded/debugged through ST-Link.

## System overview

- **4 × Dynamixel joints** — controlled through USART2 using Dynamixel Protocol 1.0
- **1 × wrist/gripper rotation servo** — PWM output on PA8
- **1 × gripper servo** — PWM output on PA6
- **USB-to-TTL data logger** — connected through USART1
- **ESP32-C3 Super Mini** — planned wireless interface through USART6
- **74LS241 buffer** — converts the Dynamixel UART connection into a controlled half-duplex bus

## Hardware

- STM32F411CEU6 Black Pill development board
- Dynamixel AX-series actuators (the driver currently targets the AX-12A control table)
- 74LS241 tri-state buffer for the Dynamixel half-duplex bus
- Two PWM-compatible hobby servos
- USB-to-TTL serial adapter
- ST-Link programmer/debugger
- ESP32-C3 Super Mini (planned)
- Suitable external power supplies for the Dynamixel and hobby servos

## Pin assignment

![STM32F411CEU6 robot arm controller pinout](docs/stm32pinout.png)

| Function | STM32 peripheral | Pin | Direction | Notes |
|---|---|---:|---|---|
| Dynamixel TX | USART2_TX | PA2 | Output | 1 Mbps, 8-N-1 |
| Dynamixel RX | USART2_RX | PA3 | Input | Receives Dynamixel status packets |
| 74LS241 direction | GPIO | PB0 | Output | High = transmit, low = receive/bus released |
| Data logger TX | USART1_TX | PA9 | Output | Connect to USB-to-TTL RX |
| Data logger RX | USART1_RX | PA10 | Input | Connect to USB-to-TTL TX |
| ESP32-C3 TX | USART6_TX | PA11 | Output | Planned wireless link; connect to ESP32 RX |
| ESP32-C3 RX | USART6_RX | PA12 | Input | Planned wireless link; connect to ESP32 TX |
| Gripper rotation servo | TIM1_CH1 | PA8 | PWM output | Rotates the gripper |
| Gripper servo | TIM3_CH1 | PA6 | PWM output | Opens and closes the gripper |
| Built-in status LED | GPIO | PC13 | Output | Active-low on the Black Pill |

> The USART1 pin assignment above follows the target hardware pinout. The generated firmware currently maps USART1 to PB6/PB7; update the alternate-function configuration to PA9/PA10 before wiring the logger to those target pins.

## Dynamixel bus

Dynamixel Protocol 1.0 uses a single bidirectional data wire. USART2 provides separate TX and RX signals, so the 74LS241 is used to control when the STM32 drives the bus.

```text
STM32 PA2 (USART2_TX) ──> 74LS241 input
STM32 PB0 (DIR)       ──> 74LS241 output-enable/direction control
74LS241 bus output    ──> Dynamixel DATA
STM32 PA3 (USART2_RX) <── Dynamixel DATA / receive path

STM32 GND ─────────────── Dynamixel interface GND
```

The firmware sets PB0 high while transmitting and returns it low before waiting for a servo response. USART2 is configured for **1,000,000 baud, 8 data bits, no parity, and 1 stop bit**.

Do not power the actuators from the STM32 board. Use correctly rated external supplies and connect the grounds of the STM32, servo supplies, Dynamixel bus, USB-to-TTL adapter, and ESP32 interface.

## Joint IDs and calibration

The earlier arm configuration stored in this repository uses the following IDs and zero offsets:

| Joint | Dynamixel ID | Home position | Approx. offset | Positive direction |
|---|---:|---:|---:|---|
| Base | 1 | 493 | 144.57° | Counterclockwise |
| Shoulder | 9 | 308 | 90.32° | Counterclockwise |
| Elbow | 13 | 481 | 141.35° | Counterclockwise |

The current test program in `src/main.c` controls **Dynamixel ID 4**, moving it between raw positions 350 and 650. Confirm every actuator ID and mechanical zero before enabling torque on the assembled arm.

For AX-series position control, raw values `0..1023` represent approximately `0..300°`:

```text
angle = raw_position × 300 / 1023
```

## Firmware status

Currently implemented:

- USART2 Dynamixel Protocol 1.0 communication at 1 Mbps
- Half-duplex direction control through PB0
- Ping, 8/16-bit read and write, torque, LED, goal-position, speed, and present-position commands
- Packet validation for ID, length, checksum, timeout, and servo-reported errors
- TIM1 channel 1 configuration for PA8
- TIM3 channel 1 configuration for PA6
- USART1 configuration at 115200 baud

Still to be integrated into the active application:

- Initialize USART1 and move its GPIO mapping from PB6/PB7 to PA9/PA10
- Add and initialize USART6 on PA11/PA12
- Initialize TIM1 and TIM3 and start both PWM channels
- Add coordinated control for all six axes
- Define the command protocol between the STM32 and ESP32-C3
- Add data-logger output to the active control loop

At present, `main()` initializes only GPIO and USART2. The other peripheral configuration files exist but are not yet called by the active program.

## Building and uploading

### Requirements

- [Visual Studio Code](https://code.visualstudio.com/) with the PlatformIO extension, or PlatformIO Core
- ST-Link programmer connected to SWDIO, SWCLK, GND, and 3.3 V reference

### PlatformIO commands

Build the firmware:

```sh
pio run
```

Upload through ST-Link:

```sh
pio run --target upload
```

Open the serial monitor for logger output once USART1 logging is enabled:

```sh
pio device monitor --baud 115200
```

The PlatformIO environment is `genericSTM32F411CE`, using the STM32Cube framework and ST-Link for upload and debugging.

## Project structure

```text
.
├── include/
│   ├── dynamixel.h       # Dynamixel driver API and control-table addresses
│   ├── gpio.h
│   ├── main.h
│   ├── tim.h
│   └── usart.h
├── src/
│   ├── dynamixel.c       # Protocol 1.0 packet transport and commands
│   ├── gpio.c            # LED and 74LS241 direction GPIO setup
│   ├── main.c            # Active test application
│   ├── tim.c             # PA8 and PA6 PWM timer configuration
│   └── usart.c           # USART1 and USART2 configuration
├── main_lama/            # Earlier application code kept for reference
└── platformio.ini        # PlatformIO target and build configuration
```

## Safety notes

- Test one actuator at a time before operating the complete arm.
- Verify Dynamixel IDs and baud rates before enabling torque.
- Keep the arm clear of people and objects during first motion tests.
- Use a current-limited actuator supply and provide an accessible emergency power cutoff.
- Set conservative position and speed limits in software to prevent mechanical collisions.
- Never connect 5 V logic directly to a non-5-V-tolerant STM32 input; verify the complete 74LS241 interface and supply arrangement before powering it.

## License

No project-level license has been added yet. Add a `LICENSE` file before redistributing the firmware.
