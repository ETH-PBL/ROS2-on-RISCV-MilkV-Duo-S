# BT-MV Duo 01 Deck
*A Crazyflie 2.1 expansion deck that brings a Milk-V Duo Module 01 (Sophgo SG2000) onto the nano-UAV, with microSD, USB, UART and a camera connector.*

> **Status (Rev-A):** fully operational as a ROS 2 companion computer. The board boots Ubuntu 22.04 from microSD and runs native ROS 2 Humble; power, microSD, USB, Wi-Fi and the UART link to the Crazyflie are validated.
> **Open item:** the camera interface is not validated yet.

Rev-A is the platform used for all ROS 2 experiments and measurements reported in this repository.

---

## What this project is
This folder contains the hardware design of a Crazyflie 2.1 deck integrating a Milk-V Duo Module 01. The module carries the Sophgo SG2000 SoC (two RISC-V C906 cores, 512 MB of in-package DRAM, 0.5 TOPS TPU) and Wi-Fi 6, and weighs 6.7 g. The deck gives the Crazyflie an on-board Linux computer in a lightweight, compact board that can be manufactured, assembled, and iterated on like any other Crazyflie expansion deck.

---

## Highlights
- Crazyflie deck form factor (mechanically compatible)
- Milk-V Duo Module 01 integration
- Power path for both Crazyflie supply (VCOM) and external USB 5 V (VBUS) with automatic switchover
- Direct UART connection to the Crazyflie flight controller (UART2, 115200 baud)
- Interfaces on the board: microSD (SD/MMC), USB 2.0, OV5647-style camera connector, UART debug

---

## What works (verified on Rev-A)
### Power & regulation
- 3.3 V rail is stable across expected input conditions
- VBUS/VCOM switchover behaves as intended (no brown-out observed during switching)

### Boot & storage
- Boots Ubuntu 22.04 with the custom kernel from microSD
- U-Boot prompt and Linux console available over the UART debug port

### USB
- USB 2.0 interface operational

### Communication
- UART2 link to the Crazyflie flight controller: attitude stream received at 100 Hz
- Wi-Fi link to a remote computer: ROS 2 messages exchanged over DDS

### Power consumption
Measured at board level with a Keysight N6705C supplying 5 V. The values cover the complete deck (SoC, USB subsystem, Wi-Fi module, microSD card, power management, status LEDs).

| Operating phase              | Mean   | Peak   |
|------------------------------|--------|--------|
| Idle, Linux only             | 443 mW | –      |
| ROS 2 start-up               | 534 mW | –      |
| Active publishing (35–45 Hz) | 510 mW | –      |
| Reboot                       | 540 mW | 746 mW |

---

## Not validated yet
### Camera
- Camera rails exist and basic clock presence was observed on Rev-A; full camera bring-up has not been validated yet.

---

## Folder contents
```text
Hardware Setup/
  BT-MV Duo 01 Deck_3D_full.pdf        # Full 3D render/export of the assembled PCB
  BT-MV Duo 01 Deck_BD.pdf             # PCB deck block diagram (general / power supply)
  BT-MV Duo 01 Deck_BOM.csv            # Bill of Materials (component list)
  BT-MV Duo 01 Deck_Lay.pdf            # PCB layer plots/export (copper, silkscreen, mask, etc.)
  BT-MV Duo 01 Deck_LayStackup.pdf     # PCB layer stack-up definition (materials / thicknesses)
  BT-MV Duo 01 Deck_Sch.pdf            # Schematic export (PDF)
```
