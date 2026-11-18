# Native ROS 2 on Nano-UAVs with a RISC-V Co-Processor

This repository contains everything needed to run native ROS 2 on a Crazyflie 2.1 nano-UAV through a lightweight RISC-V co-processor: a native ROS 2 Humble installation on Ubuntu 22.04 for the Sophgo SG2000 (riscv64), the custom Crazyflie 2.1 expansion deck that carries the SoC, the Crazyflie firmware driver that streams the attitude estimate over UART, the ROS 2 bridge node, and the remote visualization.

---
## System overview

| Platform | Role |
|---|---|
| **Crazyflie 2.1** (STM32F405, FreeRTOS) | A custom deck driver reads roll, pitch and yaw from the state estimator through the logging framework and sends them over UART2 at 100 Hz. Flight control stays entirely on the STM32. |
| **Custom deck** (Milk-V Duo Module 01, Sophgo SG2000, Ubuntu 22.04) | A native ROS 2 Humble bridge node decodes the UART stream and publishes `JointState` messages on `/joint_states` over DDS. No micro-ROS agent is involved. |
| **Remote Linux PC** (ROS 2 Humble in Docker) | `robot_state_publisher` turns the joint states into TF transforms using a simple URDF model, and RViz2 shows the drone orientation in real time. |

**Key parameters:**
- **UART link:** UART2, 115200 baud, 10-byte packets (header `0xAA55`, three signed 16-bit angles in hundredths of a degree, footer `0x55AA`)
- **Bridge node:** Python (`rclpy`), nominal publishing rate 50 Hz
- **ROS 2 network:** domain ID 42, standard DDS discovery over Wi-Fi
- **SG2000:** Linux and ROS 2 run on the 1.0 GHz C906 core; 314 MB of the 512 MB DRAM are accessible to Linux

---
## Results

Measured on the custom deck (Rev-A) while running the demo.

| Metric | Value |
|---|---|
| Linux-accessible DRAM | 314 MB |
| System memory usage | 118 MB |
| Bridge node RSS | 56 MB |
| Swap usage | 0 MB |
| System CPU utilization | 100 % |
| Bridge node CPU utilization | 96 % |
| UART reception rate | ~100 Hz |
| ROS 2 publishing rate | 35–45 Hz |

Board-level power consumption of the custom deck (Keysight N6705C, 5 V supply, ~1 kHz sampling):

| Operating phase | Mean | Peak |
|---|---|---|
| Idle, Linux only | 443 mW | – |
| ROS 2 start-up | 534 mW | – |
| Active publishing (35–45 Hz) | 510 mW | – |
| Reboot | 540 mW | 746 mW |

The single Linux-accessible core is saturated by the Python bridge node, so the measured publishing rate stays below the nominal 50 Hz. Memory is not the constraint: the complete system uses 38 % of the available DRAM without touching swap.

---
## Repository structure
```text
README.md                 # This file: overview and quick start
SETUP_GUIDE.md            # Complete build instructions from scratch
firmware/
  imuUartDeck.c           # Crazyflie deck driver: streams the attitude over UART2
ros2/
  crazyflie_imu/          # ROS 2 bridge node running on the SG2000
visualization/
  start_ros2_container.sh # Starts the ROS 2 Humble Docker container on the remote PC
  drone_visualization/    # URDF model, launch file, RViz config
Hardware Setup/           # Custom deck (BT-MV Duo 01 Deck): schematic, layout, BOM
```

A prebuilt SD-card image and the files needed for the QEMU build are on [Zenodo](https://zenodo.org/records/17723913).

---
## Hardware
- **Custom deck** ([BT-MV Duo 01 Deck](Hardware%20Setup/HW_README.md)) with a Milk-V Duo Module 01, the reference platform. The software also runs on a standalone **Milk-V Duo S** development board, which uses the same SG2000 SoC.
- **SD card:** 32GB minimum
- **Crazyflie 2.1** with Crazyradio USB Dongle
- **Linux computer**
	- With Docker installed
	- Preferably Linux Mint or Ubuntu
	- Minimum 4GB RAM, 8GB recommended
- **Jumper cables:** for the UART connection between the Crazyflie and the board

---
## Initial Setup (One-Time)
### Option 1: Use Prepared Image (Recommended)

**1. Flash the Image**
- Download `milkV-duoS_ROS_demo` from the [zenodo page](https://zenodo.org/records/17723913). The same image runs on the custom deck and on the Duo S.
- Use balenaEtcher or similar to write it to the SD card
- Insert the SD card into the board

**2. Connect to the board**
- **Via network:** SSH to the DHCP-assigned IP (check router)
- **Or: Via USB to TTL adapter:**
	- Connect USB to TTL adapter to UART0 of the board.
	- Use serial terminal like teraterm with baudrate 115200
- **Credentials:**
    - Username: `root`
    - Password: `milkv`

**3. Set Up Demo Components**
Follow these chapters from `SETUP_GUIDE.md`:
- **Chapter 3.1:** Prepare Crazyflie firmware
- **Chapter 3.2:** Set up visualization on Linux computer
### Option 2: Build From Scratch
For custom modifications or learning purposes, follow the complete instructions in `SETUP_GUIDE.md`.

---
## Running the System (After Every Reboot)

**IMPORTANT:** These steps must be repeated after every reboot of the board to initialize UART and start the ROS 2 node.
### Part 1: Hardware Connections
The pin names, the `duo-pinmux` commands and the UART device are the same on the custom deck and on the Duo S.

1. **Wire the UART connection:**
    ```
    Crazyflie              SG2000 board
    ─────────              ────────────
    GND         <─────>    GND
    TX2         ───────>   Pin B12 (UART2_RX)
    RX2         <───────   Pin B11 (UART2_TX)
    ```
2. **Power on Crazyflie drone**
### Part 2: Start the bridge node on the board
1. **SSH into the board:**
2. **Wait for login to complete** (minutes while ROS2 services initialize). Don't interrupt with Ctrl+C.

**Configure UART and start the bridge node:**
```bash
# Configure UART pins (required after every reboot)
duo-pinmux -w B11/UART2_TX
duo-pinmux -w B12/UART2_RX

# Verify UART device exists
ls -la /dev/ttyS2
# Expected: crw-rw---- 1 root dialout ... /dev/ttyS2

# Source ROS2 environment
source ~/ros2_humble/install/setup.bash
source ~/crazyflie_ws/install/setup.bash

# Start the bridge node
ros2 run crazyflie_imu imu_uart_publisher
```

**Expected output after ~30 seconds:**
```
[00:00:xx] Process: CPU  xx% | RAM  xxMB
System:    CPU  xx% | RAM xx% (xx/314MB)
Rates:     UART  xxHz | ROS  xxHz | Attitude: R:  x° P: x° Y: x°
```
*Attitude values should update in real-time as you move the drone*
### Part 3: Visualize on Linux Computer
```bash
# Allow Docker to access display
xhost +local:docker

# Navigate to workspace
cd ~/drone_viz_ws

# Start ROS2 container
./start_ros2_container.sh
```

**Inside the Docker container:**
```bash
# Source environment (if not automatic)
source ~/.bashrc

# Verify ROS2 can see the publisher on the board
ros2 topic list
# Should show: /joint_states, /robot_description, /tf, /tf_static

# Optional: Check if data is flowing
ros2 topic echo /joint_states --once
# Should show roll_joint, pitch_joint, yaw_joint with changing values

# Launch visualization
ros2 launch drone_visualization visualize_drone.launch.py
```

**Expected result:**
- RViz2 window opens showing 3D environment
- Blue square appears at center
- Drone rotates in real-time matching Crazyflie's physical orientation

---
## License
Released under the MIT License, see [`LICENSE`](LICENSE).
