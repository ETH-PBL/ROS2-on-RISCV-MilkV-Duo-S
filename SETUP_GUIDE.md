# System Requirements Overview
## Development Workstation (Ubuntu Computer)
**For building custom Ubuntu image and compiling ROS 2 in QEMU:**

**Hardware:**
- **CPU:** 4+ cores recommended
- **RAM:** 16GB minimum (8GB for QEMU + 8GB for host system)
- **Storage:** 60GB free disk space

**Software:**
- **OS:** Ubuntu 20.04 LTS or newer (tested on 22.04)
- **Privileges:** Sudo access required for debootstrap and image operations
# Part 1: Build custom ubuntu Image
**On ubuntu computer**
## 1.1: Install required dependencies
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install build tools
sudo apt install -y \
  build-essential \
  git \
  wget \
  curl \
  python3 \
  python3-pip \
  device-tree-compiler \
  libncurses5-dev \
  bc \
  bison \
  flex \
  libssl-dev \
  ninja-build \
  cmake \
  rsync \
  cpio \
  unzip \
  file \
  libelf-dev \
  kmod \
  python3-jinja2

# Install Ubuntu rootfs tools
sudo apt install -y \
  debootstrap \
  qemu-user-static \
  binfmt-support \
  dpkg-cross \
  --no-install-recommends

# Create python symlink (required by build scripts)
sudo ln -s /usr/bin/python3 /usr/bin/python

# Verify
python --version
ninja --version
```
## 1.2: Clone SDK and configure build
```bash
# Create project directory
mkdir -p MilkV_DuoS_Ubuntu_Build
cd MilkV_DuoS_Ubuntu_Build

# Clone SDK V2 (for Duo S with SG2000)
git clone https://github.com/milkv-duo/duo-buildroot-sdk-v2.git --depth=1

cd duo-buildroot-sdk-v2
```
## 1.3: Configure Kernel for Swap and ROS2
### 1.3.1: Edit Kernel Configuration
```bash
nano build/boards/cv181x/sg2000_milkv_duos_musl_riscv64_sd/linux/cvitek_sg2000_milkv_duos_musl_riscv64_sd_defconfig
```

**Add these lines at the end:**
```bash
# Systemd requirements for Ubuntu
CONFIG_CGROUPS=y
CONFIG_CGROUP_FREEZER=y
CONFIG_CGROUP_PIDS=y
CONFIG_CGROUP_DEVICE=y
CONFIG_CPUSETS=y
CONFIG_PROC_PID_CPUSET=y
CONFIG_CGROUP_CPUACCT=y
CONFIG_PAGE_COUNTER=y
CONFIG_MEMCG=y
CONFIG_CGROUP_SCHED=y
CONFIG_NAMESPACES=y
CONFIG_OVERLAY_FS=y
CONFIG_AUTOFS4_FS=y
CONFIG_SIGNALFD=y
CONFIG_TIMERFD=y
CONFIG_EPOLL=y
CONFIG_IPV6=y
CONFIG_FANOTIFY=y

# Swap support
CONFIG_SWAP=y
CONFIG_ZSMALLOC=y
CONFIG_ZRAM=y
CONFIG_ZRAM_DEF_COMP_LZORLE=y

# Additional
CONFIG_BLK_DEV_LOOP=y
CONFIG_EXT4_FS=y
CONFIG_EXT4_FS_POSIX_ACL=y
CONFIG_EXT4_FS_SECURITY=y
```
Save and exit
### 1.3.2: Set Rootfs Size to 16GB
```bash
nano device/milkv-duos-musl-riscv64-sd/genimage.cfg
```

Find the line:
```
size = 768M
```

Change to:
```
size = 16G
```
Save and exit
## 1.4: Disable Optional Build Component
```bash
nano build/envsetup_milkv.sh
```

Find this line and comment it out:
```bash
build_pqtool_server || return $?
```
Save and exit
## 1.5: Build the Image
```bash
cd MilkV_DuoS_Ubuntu_Build/duo-buildroot-sdk-v2

# Start build
./build.sh milkv-duos-musl-riscv64-sd
```
## 1.6: Verify and Backup
```bash
cd MilkV_DuoS_Ubuntu_Build/duo-buildroot-sdk-v2/out/

# Check image size
ls -lh milkv-duos-*.img
# Should show ~16-17 GB

# Backup the base kernel image
cp milkv-duos-*.img milkv-duos-kernel-base.img
```
## 1.7: Create Ubuntu 22.04 Rootfs
**We use Ubuntu 22.04 (Jammy) for ROS 2 Humble Compatibility** ROS 2 Humble requires Python 3.10, which ships with Ubuntu 22.04. Using Ubuntu 24.04 (Noble) will cause build failures due to Python 3.12 incompatibilities. All commands below use `jammy` for Ubuntu 22.04.
```bash
# Move to directory with permissions
sudo rm -rf MilkV_DuoS_Ubuntu_Build/ubuntu-rootfs

sudo mkdir -p /opt/milkv-ubuntu-build 

sudo chown $USER:$USER /opt/milkv-ubuntu-build 

cd /opt/milkv-ubuntu-build

# Create Ubuntu rootfs
sudo debootstrap --arch=riscv64 --foreign jammy ./ubuntu-rootfs http://ports.ubuntu.com/ubuntu-ports

# Mount special filesystems 
sudo mount --bind /proc ubuntu-rootfs/proc 
sudo mount --bind /sys ubuntu-rootfs/sys 
sudo mount --bind /dev ubuntu-rootfs/dev 
sudo mount --bind /dev/pts ubuntu-rootfs/dev/pts

# Enter chroot
sudo chroot ubuntu-rootfs /bin/bash

# Complete bootstrap
/debootstrap/debootstrap --second-stage
```

### 1.7.1: Configure Ubuntu (Inside Chroot)
```bash
# Create sources.list file and write repositories into
cat >/etc/apt/sources.list <<EOF
deb http://ports.ubuntu.com/ubuntu-ports jammy main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports jammy-updates main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports jammy-backports main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports jammy-security main restricted universe multiverse
EOF

# Update and install packages
apt-get update
apt-get install --no-install-recommends -y \
  util-linux \
  haveged \
  openssh-server \
  systemd \
  systemd-sysv \
  kmod \
  initramfs-tools \
  conntrack \
  ebtables \
  ethtool \
  iproute2 \
  iptables \
  mount \
  socat \
  ifupdown \
  iputils-ping \
  vim \
  nano \
  dhcpcd5 \
  sudo \
  chrony \
  locales \
  wget \
  curl \
  ca-certificates \
  build-essential \
  python3 \
  python3-pip \
  git \
  zram-config

# start zram-config: Start automatically at boot
systemctl enable zram-config

# Configure network
mkdir -p /etc/network

cat >/etc/network/interfaces <<EOF
auto lo
iface lo inet loopback
auto eth0
iface eth0 inet dhcp
auto usb0
iface usb0 inet dhcp
EOF

cat >/etc/resolv.conf <<EOF
nameserver 8.8.8.8
nameserver 1.1.1.1
EOF

# Configure fstab
cat >/etc/fstab <<EOF
/dev/root / ext4 rw,noatime 0 1
proc /proc proc defaults 0 0
devpts /dev/pts devpts defaults,gid=5,mode=620,ptmxmode=0666 0 0
tmpfs /dev/shm tmpfs mode=0777 0 0
tmpfs /tmp tmpfs mode=1777,size=128M 0 0
tmpfs /run tmpfs mode=0755,nosuid,nodev,size=64M 0 0
sysfs /sys sysfs defaults 0 0
/swapfile none swap sw 0 0
EOF

# Setup locale
locale-gen en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Set hostname (overwrite /etc/hostname)
echo "milkv-duos-ubuntu" > /etc/hostname

# Set root password (chpasswd is a command)
echo "root:milkv" | chpasswd

# Uncomment "PermitRootLogin yes" in sshd_config
sed -i "s/#PermitRootLogin.*/PermitRootLogin yes/g" /etc/ssh/sshd_config

# Exit chroot
exit

# Unmount special filesystems 
sudo umount ubuntu-rootfs/dev/pts 
sudo umount ubuntu-rootfs/dev 
sudo umount ubuntu-rootfs/sys 
sudo umount ubuntu-rootfs/proc
```
### 1.7.2: Create Rootfs Tarball, move it
```bash
cd /opt/milkv-ubuntu-build

# Create tarball
sudo tar -cSf Ubuntu-jammy-rootfs.tar -C ubuntu-rootfs .

gzip Ubuntu-jammy-rootfs.tar

# Move it
mv Ubuntu-jammy-rootfs.tar.gz MilkV_DuoS_Ubuntu_Build/
```
## 1.8: Combine Kernel + Ubuntu Rootfs
```bash
cd MilkV_DuoS_Ubuntu_Build

# Copy base image
cp duo-buildroot-sdk-v2/out/milkv-duos-*.img milkv-duos-ubuntu22.img

# Create mount points
rm -rf mnt

# Create mount points
mkdir -p mnt/boot mnt/rootfs

# Mount image
sudo losetup -fP milkv-duos-ubuntu22.img
LOOP_DEV=$(losetup -l | grep milkv-duos-ubuntu22.img | awk '{print $1}')

sudo mount ${LOOP_DEV}p1 mnt/boot
sudo mount ${LOOP_DEV}p3 mnt/rootfs

# Replace rootfs
sudo rm -rf mnt/rootfs/*
sudo tar -xzf Ubuntu-jammy-rootfs.tar.gz -C mnt/rootfs/

# Verify
ls -la mnt/rootfs/
# Should see: bin, boot, dev, etc, home, lib, usr, var

# Unmount
sudo umount mnt/boot
sudo umount mnt/rootfs
sudo losetup -d $LOOP_DEV
```
## 1.9: Write to SD Card to test
```bash
# Insert SD card and find device
lsblk

# Write image (REPLACE sdX with your SD card!)
sudo dd if=MilkV_DuoS_Ubuntu_Build/milkv-duos-ubuntu22.img \
   of=/dev/sdX \
   bs=4M \
   status=progress \
   conv=fsync

sync
```
## 1.10: Boot and Test
1. Insert SD card into MilkV Duo S
2. Connect USB-C cable
### 1.10.1: Connect
- Username: root
- Password: milkv

**Option A: Via Network Ethernet (recommended)**
- Connect ethernet cable to MilkV
- Find IP address from the router
- SSH into the MilkV

**Option B: Via USB to TTL Serial Adapter**
- Connect USB to TTL adapter to UART0 on MilkV.
- Use serial terminal like teraterm with baudrate 115200: 
### 1.10.2: Verify
```bash
# Check OS
cat /etc/os-release

# Check architecture
uname -m

# Check swap
free -h
swapon --show

# Check disk space
df -h
```
# Part 2: Build ROS on QEMU VM
**On ubuntu computer**
## 2.1: Install QEMU
```bash
sudo apt update
sudo apt install -y qemu-system-riscv64
```
## 2.2: Setup VM
Inside project folder: Get these files from the [zenodo page](https://zenodo.org/records/17723913):
- `ubuntu-22.04-server-cloudimg-riscv64-initrd-generic`
- `ubuntu-22.04-server-cloudimg-riscv64-vmlinuz-generic`
- `user-data.img`
Move the previously generated ubuntu image into the project folder

## 2.3: Start VM
```bash
qemu-system-riscv64 \
-machine virt \
-m 8192 \
-smp 8 \
-nographic \
-bios default \
-kernel ubuntu-22.04-server-cloudimg-riscv64-vmlinuz-generic \
-initrd ubuntu-22.04-server-cloudimg-riscv64-initrd-generic \
-append "console=ttyS0 root=/dev/vda3 rw" \
-drive file=milkv-duos-ubuntu22.img,if=virtio,format=raw \
-drive file=user-data.img,format=raw \
-device virtio-net-device,netdev=net0 \
-netdev user,id=net0,hostfwd=tcp::2222-:22
```
- Username: `root`, Password: `milkv`
- Poweroff: `sudo shutdown -h now`
- Login from 2nd terminal: `ssh root@localhost -p 2222`
## 2.4: Fix networking
**From here: Inside QEMU**
```bash
sudo mkdir -p /etc/systemd/resolved.conf.d/ 
sudo tee /etc/systemd/resolved.conf.d/qemu-dns.conf <<EOF
[Resolve] 
DNS=10.0.2.3 
FallbackDNS=8.8.8.8 1.1.1.1 
EOF

# Restart systemd-resolved 
sudo systemctl restart systemd-resolved 

# Verify the configuration took effect 
resolvectl status 

# Test DNS resolution 
ping -c 3 8.8.8.8 
ping -c 3 ports.ubuntu.com 
```
## 2.5: Install Dependencies
### 2.5.1: Build Dependencies
```bash
sudo apt update && sudo apt upgrade -y

# Install essential build tools
sudo apt install -y \
  build-essential \
  cmake \
  git \
  wget \
  python3-pip \
  python3-setuptools \
  software-properties-common

# Enable Universe repository
sudo add-apt-repository universe -y
sudo apt update
```
### 2.5.2: ROS2-Specific Dependencies
```bash
# Install system libraries
sudo apt install -y \
  libacl1-dev \
  libattr1-dev \
  libtinyxml2-dev \
  libeigen3-dev \
  libssl-dev \
  libcurl4-openssl-dev \
  libsqlite3-dev \
  libasio-dev \
  libconsole-bridge-dev \
  liblz4-dev \
  libbz2-dev \
  zlib1g-dev \
  libbullet-dev \
  libbullet-extras-dev \
  libboost-all-dev \
  libpoco-dev \
  liblog4cxx-dev
```
### 2.5.3: Python Build Tools
ROS2 Humble requires empy 3.3.4 (not 4.x)
```bash
# Remount /tmp with 2GB to have plenty for packages (numpy)
sudo mount -o remount,size=2G /tmp

# Install Python tools with specific versions
python3 -m pip install -U \
  empy==3.3.4 \
  catkin_pkg \
  rosdep \
  vcstool \
  colcon-common-extensions \
  lark

# Add pip binaries to PATH
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```
### 2.5.4: ROS2 source code
```bash
# Create workspace
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble

# Download ROS2 Humble source code (takes some time)
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos
```

### 2.5.5: Patch mimick_vendor for RISC-V Support
The mimick library doesn't support RISC-V by default. Use a community fork:
```bash
# Edit mimick_vendor CMakeLists.txt
nano ~/ros2_humble/src/ros2/mimick_vendor/CMakeLists.txt
```
Find these lines (around line 61-63) and change:
**FROM:**
```cmake
set(mimick_version "de11f8377eb95f932a03707b583bf3d4ce5bd3e7")
# ...
GIT_REPOSITORY https://github.com/ros2/Mimick.git
```
**TO:**
```cmake
set(mimick_version "90d02296025f38da2e33c67b02b7fa0c7c7d460c")
# ...
GIT_REPOSITORY https://github.com/ziyao233/Mimick.git
```
### 2.5.6: Initialize rosdep
```bash
# Install rosdep system-wide (required for sudo access)
sudo python3 -m pip install rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```
### 2.5.7: Install ROS2 Dependencies
```bash
cd ~/ros2_humble

# Install dependencies (some may fail on RISC-V - this is expected)
rosdep install --from-paths src --ignore-src -y \
  --rosdistro humble \
  --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers ignition-cmake2 ignition-math6" || true
```
The error "command [apt-get install -y python3-catkin-pkg-modules] failed" can be ignored. Already installed catkin_pkg previously. 
## 2.6: Build ROS
**Not built:**
- rviz (visualization)
- Gazebo/Ignition (simulation)
- Some camera/image processing packages
### 2.6.1: Skip Optional Visualization Packages
These packages fail on RISC-V and aren't needed for core ROS2 functionality:
```bash
# Skip visualization and simulation packages
touch ~/ros2_humble/src/ros2/rviz/COLCON_IGNORE
touch ~/ros2_humble/src/ros-visualization/COLCON_IGNORE
touch ~/ros2_humble/src/ros2/demos/intra_process_demo/COLCON_IGNORE
touch ~/ros2_humble/src/ros2/demos/image_tools/COLCON_IGNORE

# Skip Ignition/Gazebo simulation packages
find ~/ros2_humble/src -name "*ignition*" -type d -exec touch {}/COLCON_IGNORE \;
find ~/ros2_humble/src -name "*gazebo*" -type d -exec touch {}/COLCON_IGNORE \;
```
### 2.6.2: Build ROS2
**This takes about 12 hours!** Run over night.
```bash
cd ~/ros2_humble

# Build with limited parallelism (adjust -j value based on CPU cores)
colcon build --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF \
  --parallel-workers 2 \
  --executor sequential
```

**Build options explained:**
- `--symlink-install`: Faster development iteration
- `-DBUILD_TESTING=OFF`: Skips test building (saves time)
- `--parallel-workers 2`: Limits parallel builds (adjust based on RAM: 1 for 4GB, 2 for 8GB)
- `--executor sequential`: Prevents system freezing on limited resources
**If build runs out of memory:** Reduce `--parallel-workers` to 1 or add swap space.
**If build fails partway through:** Resume with:
```bash
cd ~/ros2_humble

colcon build --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF \
  --parallel-workers 2 \
  --executor sequential \
  --packages-skip-build-finished
```

## 2.7: Test and write ROS2
### 2.7.1: Set Up Environment
```bash
# Source ROS2 setup
source ~/ros2_humble/install/setup.bash

# Add to .bashrc for automatic sourcing
echo "source ~/ros2_humble/install/setup.bash" >> ~/.bashrc
```
### 2.7.2: Verify Installation

**Terminal 1:**
```bash
source ~/ros2_humble/install/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2:**
```bash
source ~/ros2_humble/install/setup.bash
ros2 run demo_nodes_py listener
```

The talker should be publishing messages and the listener receiving them.

**Verify message interfaces:**
```bash
ros2 interface list | grep builtin_interfaces
```

Should show:
```
builtin_interfaces/msg/Duration
builtin_interfaces/msg/Time
```

### 2.7.3: Fix incorrect local_setup.bash path
```bash
# Backup .bashrc
cp ~/.bashrc ~/.bashrc.backup

# Remove all ROS2 lines
sed -i '/source ~\/ros2_humble\/install\/setup.bash/d' ~/.bashrc
sed -i '/source ~\/crazyflie_ws\/install\/setup.bash/d' ~/.bashrc
sed -i '/export ROS_DOMAIN_ID/d' ~/.bashrc

# Add them back once
cat >> ~/.bashrc << 'EOF'
# ROS2 Environment
source ~/ros2_humble/install/setup.bash
source ~/crazyflie_ws/install/setup.bash
export ROS_DOMAIN_ID=42
EOF

# Fix setup.bash
sed -i 's|COLCON_CURRENT_PREFIX="/root/install"|COLCON_CURRENT_PREFIX="/root/ros2_humble/install"|g' ~/ros2_humble/install/setup.bash

# Fix setup.sh
sed -i 's|COLCON_CURRENT_PREFIX="/root/install|COLCON_CURRENT_PREFIX="/root/ros2_humble/install|g' ~/ros2_humble/install/setup.sh
```

### 2.7.4: Write to SD Card
```bash
# Exit the VM
sudo shutdown -h now
```

**now we are outside QEMU again**
```bash
# Insert SD card and find device
lsblk

# Write image (REPLACE sda with your SD card!)
sudo dd if=milkv-duos-ubuntu22.img \
   of=/dev/sda \
   bs=4M \
   status=progress \
   conv=fsync

sync
```
# Part 3 Prepare demo
## 3.1: Prepare demo on Crazyflie
**Do this on an Ubuntu Machine**
### 3.1.1: Install everything required
```bash
sudo apt update
sudo apt install build-essential libncurses5-dev gcc-arm-none-eabi git python3-pip

mkdir Crazyflie && cd Crazyflie

# Clone firmware
git clone https://github.com/bitcraze/crazyflie-firmware.git
cd crazyflie-firmware
git submodule init
git submodule update

# Clone clients (for cfloader tool)
cd Crazyflie
git clone https://github.com/bitcraze/crazyflie-clients-python.git

# Install cfloader tool
cd Crazyflie/crazyflie-clients-python
pip3 install -e . --break-system-packages
```
### 3.1.2: Set Up USB Permissions
```bash
# Add user to plugdev group
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER

# Create udev rules
sudo nano /etc/udev/rules.d/99-crazyradio.rules
```

Paste this content:
```
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
```
Save and exit

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Plug the Crazyradio into an USB Port**
### 3.1.3: Configure Firmware
#### 3.1.3.1: Create UART Publisher code
```bash
cd Crazyflie/crazyflie-firmware
# Load default configuration
make cf2_defconfig

nano src/deck/drivers/src/imuUartDeck.c
```

*Paste this publisher code:* It takes the angles from the crazyflie state estimates and sends it over uart at 100Hz.
```c
#define DEBUG_MODULE "ANGLEUART"

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deck.h"
#include "uart2.h"
#include "log.h"
#include "debug.h"

#define UART_BAUDRATE 115200
#define SEND_FREQUENCY_MS 10    // 100Hz

#define PACKET_HEADER 0xAA55
#define PACKET_FOOTER 0x55AA

typedef struct {
    uint16_t header;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint16_t footer;
} __attribute__((packed)) AttitudePacket;

static int logIdRoll, logIdPitch, logIdYaw;
static bool logIdsInitialized = false;

static void initLogIds() {
    if (!logIdsInitialized) {
        logIdRoll = logGetVarId("stateEstimate", "roll");
        logIdPitch = logGetVarId("stateEstimate", "pitch");
        logIdYaw = logGetVarId("stateEstimate", "yaw");
        
        if (logIdRoll != -1 && logIdPitch != -1 && logIdYaw != -1) {
            logIdsInitialized = true;
            DEBUG_PRINT("ANGLE UART: Initialized\n");
        }
    }
}

static void angleUartTask(void *param) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    AttitudePacket packet;
    float roll, pitch, yaw;
    
    vTaskDelay(M2T(1000));
    
    while(1) {
        if (!logIdsInitialized) {
            initLogIds();
            vTaskDelayUntil(&lastWakeTime, M2T(SEND_FREQUENCY_MS));
            continue;
        }
        
        roll = logGetFloat(logIdRoll);
        pitch = logGetFloat(logIdPitch);
        yaw = logGetFloat(logIdYaw);
        
        packet.header = PACKET_HEADER;
        packet.roll = (int16_t)(roll * 100.0f);
        packet.pitch = (int16_t)(pitch * 100.0f);
        packet.yaw = (int16_t)(yaw * 100.0f);
        packet.footer = PACKET_FOOTER;
        
        uart2SendData(sizeof(packet), (uint8_t*)&packet);
        
        vTaskDelayUntil(&lastWakeTime, M2T(SEND_FREQUENCY_MS));
    }
}

static void angleUartDeckInit(DeckInfo *info) {
    uart2Init(UART_BAUDRATE);
    DEBUG_PRINT("ANGLE UART: Started\n");
    
    xTaskCreate(angleUartTask, "ANGLEUART", 
                configMINIMAL_STACK_SIZE * 2, NULL, 
                3, NULL);
}

static bool angleUartDeckTest() {
    return true;
}

static const DeckDriver angleUartDeck = {
    .name = "bcIMUUART",
    .usedGpio = 0,
    .usedPeriph = DECK_USING_UART2,
    .init = angleUartDeckInit,
    .test = angleUartDeckTest,
};

DECK_DRIVER(angleUartDeck);
```
*Save and exit*
#### 3.1.3.2: Add it as a deck
```bash
nano src/deck/drivers/src/Kconfig
```

Add before the final `endmenu`:
``` 
config DECK_IMU_UART
	bool "Support custom IMU UART streaming deck" 
	default n 
	select DECK_UART2 
	help 
		Streams IMU data over UART2
```

```bash
nano src/deck/drivers/src/Kbuild
```

Add this line:
```makefile
obj-y += imuUartDeck.o
```

```bash
# Open configuration menu
make menuconfig
```

**In menuconfig:**
1. Navigate to **"Expansion deck configuration"**
2. Select **"Force load specified custom deck driver"**
    - Type: `bcIMUUART` (exactly as written)
3. Press `S` to save → confirm filename (`build/.config`)
4. Press `Q` to exit
#### 3.1.3.3: Build Firmware
```bash
make -j

# verify our file compiled
ls -la build/src/deck/drivers/src/imuUartDeck.o

```

The compiled firmware will be at `build/cf2.bin`
#### 3.1.3.4: Flash Firmware
1. **Put Crazyflie in bootloader mode:**
    - Hold power button for ~3 seconds until blue LEDs blink
2. **Flash:**
```bash
cfloader flash build/cf2.bin stm32-fw
```

Wait for "Restart the Crazyflie..." and flashing to complete. The crazyflie should now continuously transmit the IMU angles over UART. 
## 3.2: Prepare demo on Linux Mint computer
**Can also be done on ubuntu, but I did it in Mint**
### 3.2.1: Install and configure Docker
```bash
# Update package index
sudo apt update

# Install prerequisites
sudo apt install -y ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Set up Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  noble stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add user to docker group
sudo usermod -aG docker $USER

# Apply group changes
newgrp docker

# Verify installation
docker --version

docker run hello-world # Should return "Hello from Docker!"
```
### 3.2.2: Set up ROS2
```bash
mkdir -p ~/drone_viz_ws/src
docker pull osrf/ros:humble-desktop-full
nano ~/start_ros2_container.sh
```

Paste this content:
```bash
#!/bin/bash

CONTAINER_NAME="ros2_drone_viz"

# Check if container already exists
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Container exists. Starting it..."
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
else
    echo "Creating new container..."
    xhost +local:docker
    
    docker run -it \
        --name $CONTAINER_NAME \
        --network host \
        --env DISPLAY=$DISPLAY \
        --env QT_X11_NO_MITSHM=1 \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --volume ~/drone_viz_ws:/root/drone_viz_ws:rw \
        --device /dev/dri:/dev/dri \
        osrf/ros:humble-desktop-full \
        bash
fi
```
Save and exit

```bash
# Make it executable
chmod +x start_ros2_container.sh

# Allow Docker GUI Access
xhost +local:docker

# Start container
./start_ros2_container.sh
```
### 3.2.3: Create Visualization Package (Inside Container)
```bash
# Install pip
apt update
apt install -y nano python3-pip

pip3 install pyserial psutil --break-system-packages

# Create Ros2 Package
cd /root/drone_viz_ws/src
source /opt/ros/humble/setup.bash

ros2 pkg create --build-type ament_python drone_visualization \
  --dependencies rclpy sensor_msgs urdf robot_state_publisher

# Create URDF Model
cd /root/drone_viz_ws/src/drone_visualization
mkdir urdf
nano urdf/simple_drone.urdf
```

Paste this content, which describes the drone in rviz:
```xml
<?xml version="1.0"?>
<robot name="simple_drone">
  
  <!-- Base Link (fixed world reference) -->
  <link name="base_link"/>

  <!-- Invisible intermediate links (zero size) -->
  <link name="roll_link"/>
  <link name="pitch_link"/>
  <link name="yaw_link"/>

  <!-- Drone Body (the only visible element) -->
  <link name="drone_body">
    <visual>
      <geometry>
        <box size="0.35 0.35 0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.5 1.0 1.0"/>
      </material>
    </visual>
    <!-- arrow -->
    <visual>
      <origin xyz="0.2 0 0.03" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.05 0.01"/>
      </geometry>
      <material name="orange">
        <color rgba="1.0 0.5 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Roll Joint - zero offset -->
  <joint name="roll_joint" type="continuous">
    <parent link="base_link"/>
    <child link="roll_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="-1 0 0"/>
  </joint>

  <!-- Pitch Joint - zero offset -->
  <joint name="pitch_joint" type="continuous">
    <parent link="roll_link"/>
    <child link="pitch_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Yaw Joint - zero offset -->
  <joint name="yaw_joint" type="continuous">
    <parent link="pitch_link"/>
    <child link="yaw_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Attach drone body directly to yaw link (zero offset) -->
  <joint name="body_joint" type="fixed">
    <parent link="yaw_link"/>
    <child link="drone_body"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

</robot>
```
Save and exit
### 3.2.4: Create Launch file
```bash
mkdir launch
nano launch/visualize_drone.launch.py
```

Paste this content:
```python
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('drone_visualization'),
        'urdf',
        'simple_drone.urdf'
    )
    
    # Read URDF content
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # RViz2 node
    rviz_config_file = os.path.join(
        get_package_share_directory('drone_visualization'),
        'rviz',
        'drone_view.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )
    
    return LaunchDescription([
        robot_state_publisher,
        rviz_node
    ])
```
Save and exit.
### 3.2.5: Create RViz Config
```bash
mkdir rviz
nano rviz/drone_view.rviz
```

Paste this content, which describes the rviz environment
```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Topic:
        Value: /robot_description
      Visual Enabled: true
      
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: false
      Show Axes: true
      Show Names: true
      
  Global Options:
    Fixed Frame: base_link
    
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1.5
      Focal Point:
        X: 0.0
        Y: 0.0
        Z: 0.0
      Yaw: 0.785
      Pitch: 0.524
```
Save and exit.
### 3.2.6: Configure Package Setup
```bash
cd /root/drone_viz_ws/src/drone_visualization
nano setup.py
```

Replace the content with:
```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Simple drone visualization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```
Save and exit.
### 3.2.7: Build Package
```bash
cd /root/drone_viz_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_visualization
```
### 3.2.8: Configure environment
```bash
# Add to bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /root/drone_viz_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Source for current session
source ~/.bashrc
```

## 3.3: Prepare demo on MilkV
**SSH into the MilkV board**
### 3.3.1: Set up UART
#### 3.3.1.1: Install duo-pinmux
Download the "duo-pinmux" file from https://github.com/tommasopolonelli/rtkbase_riscv/tree/master/duo-pinmux/duos

```bash
# If not in system path, install it:
chmod +x duo-pinmux
cp duo-pinmux /usr/local/bin/

# Verify installation
duo-pinmux
```
#### 3.3.1.2: Configure UART pins
**This has to be done after every reboot!**
```bash
duo-pinmux -w B11/UART2_TX
duo-pinmux -w B12/UART2_RX

# Verify
duo-pinmux -r B11
duo-pinmux -r B12
ls -la /dev/ttyS2 # should show .../dev/ttyS2
```
### 3.3.2: Create custom ROS2 Package
```bash
# Source ROS2
source ~/ros2_humble/install/setup.bash

# Create ROS2 Workspace
mkdir -p ~/crazyflie_ws/src
cd ~/crazyflie_ws/src

# Create ROS2 Package
ros2 pkg create --build-type ament_python crazyflie_imu \
  --dependencies rclpy sensor_msgs std_msgs
```
### 3.3.3: Create IMU Publisher node
```bash
cd ~/crazyflie_ws/src/crazyflie_imu/crazyflie_imu
nano imu_uart_publisher.py
```

Paste this code, which takes the angles from uart and publishes them over the ROS network. It also outputs relevant system data.
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import serial
import struct
import time
import threading
import psutil
import os

class ImuUartPublisher(Node):
    def __init__(self):
        super().__init__('imu_uart_publisher')
        
        # Publisher for joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # UART configuration
        self.uart_port = '/dev/ttyS2'
        self.baudrate = 115200
        self.packet_header = 0xAA55
        self.packet_footer = 0x55AA
        self.packet_size = 10
        
        # Thread-safe data storage
        self.latest_roll = 0.0
        self.latest_pitch = 0.0
        self.latest_yaw = 0.0
        self.data_lock = threading.Lock()
        self.packet_count = 0
        self.published_count = 0
        
        # Performance monitoring
        self.start_time = time.time()
        self.process = psutil.Process(os.getpid())
        
        # Initialize CPU measurements (establish baseline)
        psutil.cpu_percent(interval=None)
        self.process.cpu_percent(interval=None)
        time.sleep(0.1)
        
        # Rate calculation tracking
        self.last_packet_count = 0
        self.last_published_count = 0
        self.last_rate_calc_time = time.time()
        
        # Stats update interval
        self.stats_interval = 0.1
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.ser.reset_input_buffer()
            time.sleep(0.5)
            self.get_logger().info(f'UART opened on {self.uart_port}')
            
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open UART: {e}')
            return
        
        # Start continuous reading thread
        self.running = True
        self.read_thread = threading.Thread(target=self.uart_read_loop, daemon=True)
        self.read_thread.start()
        
        # Create timer for publishing at 50Hz
        self.timer = self.create_timer(0.02, self.publish_callback)
        
        # Create timer for stats display
        self.stats_timer = self.create_timer(self.stats_interval, self.print_statistics)
    
    def get_system_stats(self):
        try:
            process_cpu = min(self.process.cpu_percent(interval=None), 100.0)
            process_mem_mb = self.process.memory_info().rss / 1024 / 1024
            system_cpu = psutil.cpu_percent(interval=None)
            system_mem = psutil.virtual_memory()
            system_swap = psutil.swap_memory()
            runtime = time.time() - self.start_time
            return {
                'process_cpu': process_cpu,
                'process_mem_mb': process_mem_mb,
                'system_cpu': system_cpu,
                'system_mem_percent': system_mem.percent,
                'system_mem_mb': system_mem.used / 1024 / 1024,
                'system_mem_total_mb': system_mem.total / 1024 / 1024,
                'system_swap_percent': system_swap.percent,
                'system_swap_mb': system_swap.used / 1024 / 1024,
                'runtime_seconds': runtime
            }
        except Exception as e:
            self.get_logger().error(f'Error in get_system_stats: {e}')
            return None
    
    def print_statistics(self):
        current_time = time.time()
        elapsed = current_time - self.last_rate_calc_time
        
        stats = self.get_system_stats()
        if stats is None:
            return
        
        runtime = stats['runtime_seconds']
        hours = int(runtime // 3600)
        minutes = int((runtime % 3600) // 60)
        seconds = int(runtime % 60)
        
        # Get current counts (thread-safe)
        with self.data_lock:
            current_packets = self.packet_count
            current_published = self.published_count
            roll = self.latest_roll
            pitch = self.latest_pitch
            yaw = self.latest_yaw
        
        # Calculate instantaneous rates
        uart_rate = (current_packets - self.last_packet_count) / elapsed if elapsed > 0 else 0
        ros_rate = (current_published - self.last_published_count) / elapsed if elapsed > 0 else 0
        
        # Update tracking variables
        self.last_packet_count = current_packets
        self.last_published_count = current_published
        self.last_rate_calc_time = current_time
        
        # Build compact output (3 lines)
        output = f"\r\033[K"
        
        # Line 1: Time and Process metrics
        output += f"[{hours:02d}:{minutes:02d}:{seconds:02d}] "
        output += f"Process: CPU {stats['process_cpu']:5.1f}% | RAM {stats['process_mem_mb']:5.1f}MB"
        
        # Line 2: System metrics
        output += f"\n\033[K"
        output += f"System:  CPU {stats['system_cpu']:5.1f}% | "
        output += f"RAM {stats['system_mem_percent']:4.1f}% ({stats['system_mem_mb']:.0f}/{stats['system_mem_total_mb']:.0f}MB)"
        
        # Add swap if being used
        if stats['system_swap_percent'] > 0.1:
            output += f" | Swap {stats['system_swap_percent']:4.1f}% ({stats['system_swap_mb']:.0f}MB)"
        
        # Line 3: Communication rates and attitude
        output += f"\n\033[K"
        output += f"Rates:   UART {uart_rate:6.1f}Hz | ROS {ros_rate:5.1f}Hz | "
        output += f"Attitude: R:{roll:6.2f}° P:{pitch:6.2f}° Y:{yaw:6.2f}°"
        
        # Move cursor up 2 lines
        output += "\033[2A"
        
        print(output, end='', flush=True)
    
    def find_header(self):
        """Find packet header for synchronization"""
        buffer = bytearray()
        timeout = 1000
        
        while timeout > 0 and self.running:
            byte = self.ser.read(1)
            if len(byte) == 0:
                timeout -= 1
                continue
                
            buffer.append(byte[0])
            if len(buffer) > 2:
                buffer.pop(0)
            
            if len(buffer) == 2:
                value = struct.unpack('<H', bytes(buffer))[0]
                if value == self.packet_header:
                    return True
        return False
    
    def uart_read_loop(self):
        """Continuous UART reading thread"""
        self.get_logger().info('UART reading thread started')
        
        if not self.find_header():
            self.get_logger().error('Failed to synchronize!')
            return
        
        self.get_logger().info('Synchronized with IMU stream')
        print("\n\n\n")
        
        error_count = 0
        
        while self.running:
            try:
                data = self.ser.read(self.packet_size - 2)
                
                if len(data) != self.packet_size - 2:
                    if self.find_header():
                        continue
                    break
                
                values = struct.unpack('<3hH', data)
                roll_raw, pitch_raw, yaw_raw, footer = values
                
                if footer != self.packet_footer:
                    error_count += 1
                    if self.find_header():
                        continue
                    break
                
                roll = roll_raw / 100.0
                pitch = pitch_raw / 100.0
                yaw = yaw_raw / 100.0
                
                with self.data_lock:
                    self.latest_roll = roll
                    self.latest_pitch = pitch
                    self.latest_yaw = yaw
                    self.packet_count += 1
                
                if not self.find_header():
                    break
                    
            except struct.error:
                error_count += 1
                if self.find_header():
                    continue
                break
            except Exception as e:
                print(f"\n\n\n")
                self.get_logger().error(f'Read error: {e}')
                break
        
        print(f"\n\n\n")
        self.get_logger().warn('UART reading thread stopped')
    
    def publish_callback(self):
        """Publish latest IMU data as joint states"""
        with self.data_lock:
            roll = self.latest_roll
            pitch = self.latest_pitch
            yaw = self.latest_yaw
        
        roll_rad = roll * 3.14159 / 180.0
        pitch_rad = pitch * 3.14159 / 180.0
        yaw_rad = yaw * 3.14159 / 180.0
        
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.name = ['roll_joint', 'pitch_joint', 'yaw_joint']
        msg.position = [roll_rad, pitch_rad, yaw_rad]
        
        self.publisher.publish(msg)
        
        with self.data_lock:
            self.published_count += 1
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        print(f"\n\n\n")
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=2.0)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuUartPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```
Save and exit
### 3.3.4: Prepare package build
```bash
pip3 install pyserial
cd ~/crazyflie_ws/src/crazyflie_imu
nano setup.py
```

Find the `entry_points` section and modify it:
```python
    entry_points={
        'console_scripts': [
            'imu_uart_publisher = crazyflie_imu.imu_uart_publisher:main',
        ],
    },
```
Save and exit
### 3.3.5: Build the package
```bash
cd ~/crazyflie_ws
source ~/ros2_humble/install/setup.bash
colcon build --packages-select crazyflie_imu
```
Expected output: `Summary: 1 package finished [X.XXs]`
### 3.3.6: Source it
```bash
source ~/crazyflie_ws/install/setup.bash

# Add to `~/.bashrc` for automatic sourcing:
echo "source ~/ros2_humble/install/setup.bash" >> ~/.bashrc
echo "source ~/crazyflie_ws/install/setup.bash" >> ~/.bashrc
```
### 3.3.7: Set ROS Domain ID
Add ROS Domain ID:
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

**The setup is now finished. Follow "Run the demo" of the main readme file to test it.**