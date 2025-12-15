# ROS2 on a MilkV-DuoS board
This project implements a complete ROS2 Humble system on the MilkV Duo S (RISC-V). The included demo receives real-time IMU data from a Crazyflie drone via UART, and publishes it over a ROS2 network. It then visualizes the drone's orientation on another computer in RViz2 to verify the system is functioning correctly.

---
## Prerequisites
- **MilkV Duo S** with prepared image
- **SD card:** 32GB minimum
### For the demo
- **Crazyflie 2.1** with Crazyradio USB Dongle
- **Linux computer** 
	- With Docker installed
	- Preferably Linux Mint or Ubuntu
	- Minimum 4GB RAM, 8GB recommended
- **Jumper cables:** For Crazyflie-to-MilkV connection

---
## Initial Setup (One-Time)
### Option 1: Use Prepared Image (Recommended)

**1. Flash the Image**
- Download `milkV-duoS_ROS_demo.img`
- Use balenaEtcher or similar to write it to the SD card
- Insert SD card into MilkV Duo S

**2. Connect to MilkV**
- **Via network ethernet:** Use DHCP-assigned IP (check router)
- **Or: Via USB to TTL adapter:** 
	- Connect USB to TTL adapter to UART0 on MilkV.
	- Use serial terminal like teraterm with baudrate 115200: 
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

**IMPORTANT:** These steps must be repeated after every MilkV reboot to initialize UART and start ROS2 services.
### Part 1: Hardware Connections
1. **Wire the UART connection:**
    ```
    Crazyflie              MilkV Duo S
    ─────────              ────────────
    GND         <─────>    GND
    TX2         ───────>   Pin B12 (UART2_RX)
    RX2         <───────   Pin B11 (UART2_TX)
    ```
2. **Power on Crazyflie drone**
### Part 2: Start Publisher on MilkV
1. **SSH into MilkV:**
2. **Wait for login to complete** (minutes while ROS2 services initialize). Don't interrupt with Ctrl+C.

**Configure UART and start publisher:**
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

# Start IMU publisher
ros2 run crazyflie_imu imu_uart_publisher
```

**Expected output after ~30 seconds:**
```
[00:00:xx] Process: CPU  xx% | RAM  xxMB
System:    CPU  xx% | RAM xx% (xx/512MB)
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

# Verify ROS2 can see the MilkV publisher
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

**Key components:**
- **Crazyflie:** Custom firmware streams attitude at 100Hz via UART2
- **MilkV Duo S:** Python node reads UART, publishes to ROS2 at 50Hz
- **ROS2 Network:** Uses domain ID 42, standard DDS discovery
- **Visualization:** Robot state publisher + RViz2 drone model

---
## File Reference
- **This file:** Quick start guide
- **`SETUP_GUIDE.md`:** Complete build instructions from scratch
- **`milkV-duoS_ROS_demo.img`:** Ready-to-use SD card image with ROS2 pre-installed