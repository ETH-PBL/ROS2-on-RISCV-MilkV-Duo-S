# Setup Guide
Complete build instructions for the system in this repository: the Ubuntu 22.04 image for the Sophgo SG2000, the ROS 2 Humble build for riscv64, the Crazyflie firmware driver, the ROS 2 bridge node and the remote visualization. To run a system that is already set up, see [`README.md`](README.md).


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
The custom deck uses the same SDK build target as the Milk-V Duo S (`milkv-duos-musl-riscv64-sd`), so the resulting image runs on both boards.

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

# Clone SDK V2 (for SG2000 boards)
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
1. Insert SD card into the board
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
#### 3.1.3.1: Add the UART deck driver
The deck driver is [`firmware/imuUartDeck.c`](firmware/imuUartDeck.c). It reads the roll, pitch and yaw estimates through the firmware logging framework and sends them over UART2 at 100 Hz. Each packet is 10 bytes: a `0xAA55` header, three signed 16-bit angles in hundredths of a degree, and a `0x55AA` footer.
```bash
# Get the source files of this repository (skipped if already cloned)
[ -d ~/ROS2-on-RISCV-MilkV-Duo-S ] || git clone https://github.com/ETH-PBL/ROS2-on-RISCV-MilkV-Duo-S.git ~/ROS2-on-RISCV-MilkV-Duo-S

cd Crazyflie/crazyflie-firmware
# Load default configuration
make cf2_defconfig

cp ~/ROS2-on-RISCV-MilkV-Duo-S/firmware/imuUartDeck.c src/deck/drivers/src/
```
#### 3.1.3.2: Add it as a deck
```bash
nano src/deck/drivers/src/Kconfig
```

Add before the final `endmenu`:
``` 
config DECK_IMU_UART
	bool "Support custom attitude UART streaming deck" 
	default n 
	select DECK_UART2 
	help 
		Streams the attitude estimate over UART2
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

Wait for "Restart the Crazyflie..." and flashing to complete. The crazyflie should now continuously transmit the attitude angles (roll, pitch, yaw) over UART2. 
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

# Get the container start script from this repository (clone skipped if already done)
[ -d ~/ROS2-on-RISCV-MilkV-Duo-S ] || git clone https://github.com/ETH-PBL/ROS2-on-RISCV-MilkV-Duo-S.git ~/ROS2-on-RISCV-MilkV-Duo-S
cp ~/ROS2-on-RISCV-MilkV-Duo-S/visualization/start_ros2_container.sh ~/drone_viz_ws/
cd ~/drone_viz_ws
chmod +x start_ros2_container.sh

# Allow Docker GUI Access
xhost +local:docker

# Start container
./start_ros2_container.sh
```
### 3.2.3: Create Visualization Package (Inside Container)
The package files are in [`visualization/drone_visualization/`](visualization/drone_visualization/).
```bash
# Install tools
apt update
apt install -y nano git python3-pip

# Get the source files of this repository
cd /root
git clone https://github.com/ETH-PBL/ROS2-on-RISCV-MilkV-Duo-S.git
REPO=/root/ROS2-on-RISCV-MilkV-Duo-S/visualization/drone_visualization

# Create Ros2 Package
cd /root/drone_viz_ws/src
source /opt/ros/humble/setup.bash

ros2 pkg create --build-type ament_python drone_visualization \
  --dependencies rclpy sensor_msgs urdf robot_state_publisher

# Add the URDF model, which describes the drone in rviz
cd /root/drone_viz_ws/src/drone_visualization
mkdir urdf
cp $REPO/urdf/simple_drone.urdf urdf/
```
### 3.2.4: Add the Launch file
```bash
mkdir launch
cp $REPO/launch/visualize_drone.launch.py launch/
```
### 3.2.5: Add the RViz Config
```bash
mkdir rviz
cp $REPO/rviz/drone_view.rviz rviz/
```
### 3.2.6: Configure Package Setup
Replace the generated `setup.py` with the one from this repository, which also installs the launch, URDF and RViz files:
```bash
cp $REPO/setup.py /root/drone_viz_ws/src/drone_visualization/setup.py
```
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

## 3.3: Prepare demo on the SG2000 board
**SSH into the board**
```bash
# Get the source files of this repository
cd ~
git clone https://github.com/ETH-PBL/ROS2-on-RISCV-MilkV-Duo-S.git
```
### 3.3.1: Set up UART
The pin names, the `duo-pinmux` tool and the UART device are the same on the custom deck and on the Duo S.

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
### 3.3.3: Add the bridge node
The bridge node is [`ros2/crazyflie_imu/imu_uart_publisher.py`](ros2/crazyflie_imu/imu_uart_publisher.py). It reads the attitude packets from UART2, publishes roll, pitch and yaw as `JointState` messages on `/joint_states`, and prints the runtime statistics (CPU, memory, UART and ROS 2 rates).
```bash
cp ~/ROS2-on-RISCV-MilkV-Duo-S/ros2/crazyflie_imu/imu_uart_publisher.py \
   ~/crazyflie_ws/src/crazyflie_imu/crazyflie_imu/
```
### 3.3.4: Prepare package build
```bash
pip3 install pyserial psutil
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