# BT-MV Duo 01 Deck
*A Crazyflie 2.1 expansion deck that brings a Milk-V Duo Module 01 into the nano-UAV world — with hooks for SD/MMC, USB, and a camera.*

> **Status (Rev-A):** boots over UART and power is solid.  
> **Blockers:** SD/MMC and USB are not working yet, which prevents full end-to-end camera validation.

---

## What this project is
This repository contains the hardware design for a Crazyflie 2.1 deck integrating a Milk-V Duo Module 01 (dual-core SoM) to enable on-board compute beyond the stock platform. The aim is a lightweight, compact board that can be manufactured, assembled, and iterated on like any other Crazyflie expansion deck.

---

## Highlights
- Crazyflie deck form factor (mechanically compatible)
- Milk-V Duo Module 01 integration
- Power path designed for both Crazyflie supply (VCOM) and external USB 5 V (VBUS) with automatic switchover
- Interfaces on the board (intended): microSD (SD/MMC), USB 2.0, OV5647-style camera connector, UART debug

---

## What works (verified on Rev-A)
### ✅ Power & regulation
- 3.3 V rail is stable across expected input conditions
- VBUS/VCOM switchover behaves as intended (no brown-out observed during switching)

### ✅ Boot & console
- RISC-V boot path reachable via UART
- U-Boot prompt available and interactive

---

## What does *not* work yet (known issues)
### ❌ SD/MMC (microSD)
Symptoms:
- SDHCI initialisation repeatedly times out
- Environment load from FAT fails; boot falls back to defaults

Most likely causes (current hypotheses):
- Signal integrity / timing margin issues on CMD/CLK/DAT lines
- Protection / filtering choice on the SD bus introducing too much series impedance

### ❌ USB (device not recognised)
Symptoms:
- VBUS present (board powers), but host does not enumerate the device
- Removing protection components on D+/D− did not resolve enumeration

Most likely causes (current hypotheses):
- Attach signalling / CC configuration (if USB-C path is involved)
- D+/D− routing/impedance/return path issues

### ⚠️ Camera
- Camera rails exist and basic clock presence was observed on Rev-A,
  but full camera bring-up is blocked until a reliable flashing / data path exists
  (SD and/or USB need to work first).

---

## Repository structure
```text
BT-MV Duo 01 Deck/
  BT-MV Duo 01 Deck.kicad_pcb            # PCB layout
  BT-MV Duo 01 Deck.kicad_prl            # Local project settings
  BT-MV Duo 01 Deck.kicad_pro            # KiCad project file

  MV Duo 01 Deck Schematics/             # Hierarchical schematics
    BT-MV Duo 01 Deck.kicad_sch          # Top-level schematic
    BT-Power_CFexp.kicad_sch             # Power + Crazyflie expansion interface
    BT-SDcard.kicad_sch                  # microSD (SD/MMC)
    BT-USB_UART.kicad_sch                # USB + UART debug
    BT-CameraOV.kicad_sch                # Camera connector + signals
    BT-MV_DM01.kicad_sch                 # Duo Module integration (pins/straps)
    BT-Others.kicad_sch                  # LEDs, buttons, misc

  BT-MVD01 libraries/                    # Custom KiCad libraries
    Symbols and Footprints/
    3D Models/
    BT-Milk V Duo S.pretty/              # Footprint library

  BT-MV Duo 01 Deck.csv                  # BOM
  BT-MV Duo 01 Deck.kicad_dru            # Design rules

  Related Work/
    CF Proto Exp/                        # Bitcraze deck template/docs
    Milk Module schematics/              # Module + EVB references
    Camera OV5674/                       # Camera reference material (may include mixed sensor notes)
