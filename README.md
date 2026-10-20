# AR4 ROS Driver
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)

## **NOTE: TELEOP IS NOT YET STABLE AS OF THIS VERSION USE AT YOUR OWN RISK !!!**

Expanded Ros 2 driver of the AR4 Robotic arm from [Annin Robotics](https://anninrobotics.com). Tested with ROS 2 Jazzy on Ubuntu 24.04.
The original Github Repository can be found [here](https://github.com/Annin-Robotics/ar4_ros_driver)



**Supports:**
- Tested on the MK5 Robotic arm

**Features:**
- MoveIt Control
- Gazebo Manipulation
- SteamVR Tracking Teleoperation
- [Keyboard Manipulation](https://github.com/ShellyFisherr/jazzy_ros2_art4_ros_driver_edit/tree/main/annin_ar4_keyboard_controll)
- UDP bridge
- OpenVR Tracking bridge (coming soon)

## Layout
- [Overview](#overview)
- [Hardware Components](#hardware-components)
- [Installation](#installation)
  - [Windows 11 with WSL](#windows-11-with-wsl)
  - [Linux (Recommended)](#Linux)
- [Usage](#usage)
- [Parameters](#parameters)

## Overview
- `annin_ar4_description`
  - Hardware description of arm & servo gripper urdf
- `annin_ar4_driver`
  - ROS interfaces for the arm and servo gripper drivers, built on the ros2_control framework.
  - Manages joint offsets, limits and conversion between joint and actuator messages.
  - Handles communication with the microcontrollers
- `annin_ar4_firmware`
  - Firmware for the Teensy and Arduino Nano microcontrollers.
- `annin_ar4_gazebo`
  - Simulation files for Gazebo
- `annin_ar4_keyboard_controll`
  - C++ demo files for keyboard input reading.
  - C++ files for basic robotic movement both with and without MoveIt.
- `annin_ar4_moveit_config`
  - MoveIt module for motion planning.
  - Controlling the arm and servo gripper through Rviz.
  - Responsible for reading and uploading joint states.
  - Has the related controllers for controlling the robot.
- `annin_ar4_moveit_demos`
  - C++ demo file for sending coordinates through MoveIt to the robot to move towards.
- `annin_ar4_teleoperator`
  - Teleoperation system using ROS2 and steam OpenVR tracking.
  - Responsible for coordinate system translation and conversion.
  - Responsible for velocity scaling.
  - Sends outputs from tracker directly to microcontrollers, bypassing MoveIt.
  - MoveIt is still used for reading the joint states.
- `tracker_udp_bridge`
  - Module to link the Windows version of the OpenVR module to WSL
  - (Linux version will be provided later, same for windows version)

## Hardware components
 - AR4 Robotic arm
 - PC or laptop running either windows 11 or Ubuntu 24.04.
 - Tracker and headset compatible with SteamVR

## Installation
This repository is developed and tested against `ROS 2 Jazzy` on `Ubuntu 24.04`. The installation process consists of several layers, and two different verions. Note that some parts may overlap with the download of the original github documentation from [Annin Robotics](https://anninrobotics.com)

---
### **Windows 11 with WSL**
Note that this version is only tested with the Gazebo digital Twin of the robot. This means that the guide for installing the Arduino and Teensy Firmware may be incorrect, as it is copied from the Annin Robotics Github page. The Linux version is recommended.

The process is as follows:

1. [Install WSL Ubuntu 24.04](#1-install-wsl-ubuntu-2404)
2. [Install ROS 2 Jazzy (Base system)](#2-install-ros-2-jazzy-base-system)
3. [Create and build a ROS 2 workspace](#3-create-and-build-a-ros-2-workspace)
4. [Install Steam and SteamVR](#4-install-steam-and-steamvr)
5. [Install the OpenVR SDK](#5-install-the-openvr-sdk)
6. [Install the UDP Windows module (to be added later)](#6-install-the-udp-windows-module)
7. [Install ROS control and MoveIt](#7-install-ros-controll-and-moveit)
8. [Clone and build the AR4 ROS driver](#8-clone-and-build-the-ar4-ros-driver)
9. [Configure serial permissions](#9-configure-serial-permissions)
10. [Install Arduino + Teensy toolchain](#10-install-arduino--teensy-toolchain)
11. [Flash firmware to Teensy and Arduino Nano](#11-flash-firmware-to-teensy-and-arduino-nano)
12. [Configure UDP bridge](#12-configure-udp-bridge)

The sections below walk through these steps in a clean end-to-end order.

---

### **1. Install WSL Ubuntu 24.04**
Open Powershell as Administrator. Press the Windows key, type `PowerShell`, right-click it and choose **Run as Administrator**

Enable WSL by running the following command in the terminal to enable wsl and install the required windows features:
```
wsl --install
```
Restart your computer now if prompted to complete the base WSL activation.

Install Ubuntu 24.04 (Noble) by opening the PowerShell as Administrator and run the following command to install the specific 24.04 version:
```
wsl --install -d Ubuntu-24.04
```
Set up your account once the download finished by following the instructions that appear on the terminal.

Go back to the powershell window and confirm that the correct version is installed by using the command:
```
wsl -l -v
```
It should say something along the lines of:
```
NAME            STATE           VERSION
Ubuntu-24.04    Running         2
```
---
### **2. Install ROS 2 Jazzy (Base System)**
The same tutorial as below can be found on the [official site](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) for ROS 2 Jazzy. The tutorial is also mentioned below. You should follow it until the **Environment Setup** step is completed. The official site will also have demos to try, they are not included in the tutorial below.

#### **System setup**
Make sure you have a locale which supports `UTF-8`. If you are in a minimal enviroment (such as a docker container), the locale may be something minimal like `POSIX`. We test the following settings. However, it should be fine if you're using a different `UTF-8` supported locale.
```
locale # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale # verify settings
```
#### **Enable required repositories**
You will need to add the ROS 2 apt repository to your system.

First ensure that the Ubuntu Universe repository is enabled.
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
The [ros-apt-source](https://github.com/ros-infrastructure/ros-apt-source/) packages provide keys and apt source configuration for the various ROS repositories.

Installing the ros2-apt-source packages will configure ROS 2 repositories for your system. Updates to repository configuration will occur automatically when new versions of this package are released to the ROS repositories.
```
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```
#### **Install ROS 2**
Update Your apt repository caches after setting up the repositories
```
sudo apt update
```
ROS 2 packages are built on frequently updates Ubuntu systems. It is alwas recommended that you ensure your system is up to date before installing new packages.
```
sudo apt upgrade
```
Desktop Install: ROS, Rviz, demos, tutorials
```
sudo apt install ros-jazzy-desktop
```
#### **Setup Environment**
Set up your environment
```
source /opt/ros/jazzy/setup.bash
```
---
### **3 Create and build a ROS 2 workspace**
Create and move into a new workspace using
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
This creates a standard ROS 2 workspace layout:
```
ros2_ws/
  ├── src/
  ├── build/
  ├── install/
  └── log/
```
build the basic workspace and source it using:
```
  source /opt/ros/jazzy/setup.bash
  cd ~/ros2_ws
  colcon build
  source ~/ros2_ws/install/setup.bash
```
Since you need to do this every time after building, it is adviced to setup the following:

First, put the source files into the `/.bashrc`. This way, any time a new terminal is opened, the workspace is automatically sourced. To do this, use the command:
```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

Next thing to do is to setup a command alias to build the workspaces to prevent accidentaly building workspaces in the work folder. This can be done and saved by putting the alias into the `.bashrc` file using:
```
echo "alias build='cd ~/ros2_ws && colcon build && source ~/.bashrc'"
```
now by using the command `build`, you move to the main `ros2_ws` workspace folder, build all your modules and source the needed files.

---
### **4. Install Steam and SteamVR**
Install Steam by downloading the launcher from their [official site](https://store.steampowered.com/about/).

When you open the launcher, you can find and download `SteamVR` by going to their storepage and searching for it in the searchbar in the top-right.

Alternatively, you can download it of their [digital page](https://store.steampowered.com/app/250820/SteamVR/). The official steam launcher however is still required.

To set up your system, simply follow the steps given by the program until the full system is configured.

---
### **5. Install the OpenVR SDK**

The official OpenVR SDK can be found on the [github page](https://github.com/valvesoftware/openvr). Dowload the latest version by clicking on the latest version underneath `Releases` on the right side of the page. This package uses [OpenVR SDK 2.15.6](https://github.com/ValveSoftware/openvr/releases/tag/v2.15.6), but later versions should also be compatible.

Download the `Source code (zip)` file and move it to a prefered location and unzip it there. It is not adviced to keep it in the `Downloads` folder on your computer, but to move it somewhere else. For example, a separate `openvr` folder somewhere on your preferred drive.

---
### **6. Install the UDP windows module**
Will be added later on a separate github page. sorry for the inconvenience as of now.

---
### **7. Install ROS controll and MoveIt**
Now back to the `Ubuntu 24.04` terminal. The AR4 driver is built on `ros2_control`, which provides the hardware interface layer between `MoveIt` and the robot. It can be installed using the following command:
```
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager
```

Likewise, MoveIt can be installed using:
```
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y ros-jazzy-moveit ros-jazzy-moveit-py
sudo ldconfig
```
`MoveIt` is used as a control system for robotic motion planning. It can both read the current robot state and joint states, as well as use inverse kinematics to process path planning for the robot.

### **8. Clone and build the AR4 ros driver**
The repository can be copied to your system using:
```
cd ~/ros2_ws/src
git clone https://github.com/ShellyFisherr/jazzy_ros2_art4_ros_driver_edit ar4_ros_driver
```
Your workspace should now look like:
```
~/ros2_ws/src/ar4_ros_driver
```
To use the package, multiple separate ROS dependencies have to be installed. This needs to be done once per system, or when a new version of this repository is installed to make sure all dependencies are installed. First of all, download `rosdep`.
```
sudo apt update
sudo apt install python3-rosdep
```
Then initialize it:
```
sudo rosdep init
rosdep update
```
from here, to install all dependencies, simply use:
```
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```
Now, with all dependencies installed and the alias set up previously, the system can be build using the command
```
build
```
Which builds the workspace in the correct location and sources it.

If everything went correctly, the system can be verified by launching the `MoveIt` demo. This should be done before connecting real hardware to verify that ROS + MoveIt are working correctly:
```
ros2 launch annin_ar4_moveit_config demo.launch.py ar_model:=mk5
```
This should open RViz with the AR4 model and interactive planning tools.

---
### **9. Configure serial permissions**
Now to setup the link between software and hardware. First, enable the serial port access required for communicating with the Teensy and Arduino Nano:
```
sudo adduser $USER dialout
```
then log out and back in or reboot your system for the changes to take effect.

---
### **10. Install Arduino + Teensy toolchain**

Arduino IDE can be downloaded from their [official page](https://docs.arduino.cc/software/ide-v2/tutorials/getting-started/ide-v2-downloading-and-installing/) by following the instructions.

Teensyduino can be installed by following PJRC's [official instructions](https://www.pjrc.com/teensy/td_download.html).

In Arduino IDE:

1. Open **Preferences**
2. Add the following URL to **Additional Boards Managers URLs**:
```
https://www.pjrc.com/teensy/package_teensy_index.json
```
3. Open **Tools** $\rightarrow$ **Board** $\rightarrow$ **Boards Manager**
4. Search for and install **Teensy**
5. Select **Teensy 4.1** as the active board

To install the required Arduino Libraries, from **Tools** $\rightarrow$ **Manage Libraries**, install:
- Bounce2

---
### **11. Flash firmware to Teensy and Arduino Nano**
The firmware is located in:
```
annin_ar4_firmware/
```
**Teensy 4.1**
- Open the Teensy firmware sketch in Arduino IDE
- Select **Board: Teensy 4.1**
- Select the correct USB port
- Click **Upload**

**Arduino Nano (Gripper)**
- Open the gripper firmware sketch
- Select **Board: Arduino Nano**
- Select **Processor: ATmega328P (Old Bootloader)** if required
- Click **Upload**

After flashing the firmware or power cycling the robot:
- A **Calibration Run is required**
- his is normal and expected behavior

Calibration is triggered automatically when launching the driver with
```
calibrate :=True
```

**NOTE: TELEOP IS NOT YET STABLE AS OF THIS VERSION!!!**

---
### **12. configure udp bridge**
To be added at a later date

At this point, the system can be run on the real hardware.
---
### **Linux**
Note that this is the preferred version as the original system is also build completely in ubuntu.

The required software before continuing is that your linux distro is `Ubuntu 24.04`

The process is as follows:
1. [Install ROS 2 Jazzy (Base system)](#1-install-ros-2-jazzy-base-system)
2. [Create and build a ROS 2 workspace](#2-create-and-build-a-ros-2-workspace)
3. [Install Steam and SteamVR](#3-install-steam-and-steamvr)
4. [Install the OpenVR SDK](#4-install-the-openvr-sdk)
5. [Install the UDP LINUX module (to be added later)](#5-install-the-linux-udp-module)
6. [Install ROS control and MoveIt](#6-install-ros-controll-and-moveit)
7. [Clone and build the AR4 ROS driver](#7-clone-and-build-the-ar4-ros-driver)
8. [Configure serial permissions](#8-configure-serial-permissions)
9. [Install Arduino + Teensy toolchain](#9-install-arduino-and-teensy-toolchain)
10. [Flash firmware to Teensy and Arduino Nano](#10-flash-firmware-to-teensy-and-arduino)
11. [Configure UDP bridge](#11-configure-udp-bridge)

The section below goes through the steps in a clean end-to-end order

### **1. Install ROS 2 Jazzy (Base System)**
The same tutorial as below can be found on the [official site](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) for ROS 2 Jazzy. The tutorial is also mentioned below. You should follow it until the **Environment Setup** step is completed. The official site will also have demos to try, they are not included in the tutorial below.

#### **System setup**
Make sure you have a locale which supports `UTF-8`. If you are in a minimal enviroment (such as a docker container), the locale may be something minimal like `POSIX`. We test the following settings. However, it should be fine if you're using a different `UTF-8` supported locale.
```
locale # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale # verify settings
```
#### **Enable required repositories**
You will need to add the ROS 2 apt repository to your system.

First ensure that the Ubuntu Universe repository is enabled.
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
The [ros-apt-source](https://github.com/ros-infrastructure/ros-apt-source/) packages provide keys and apt source configuration for the various ROS repositories.

Installing the ros2-apt-source packages will configure ROS 2 repositories for your system. Updates to repository configuration will occur automatically when new versions of this package are released to the ROS repositories.
```
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```
#### **Install ROS 2**
Update Your apt repository caches after setting up the repositories
```
sudo apt update
```
ROS 2 packages are built on frequently updates Ubuntu systems. It is alwas recommended that you ensure your system is up to date before installing new packages.
```
sudo apt upgrade
```
Desktop Install: ROS, Rviz, demos, tutorials
```
sudo apt install ros-jazzy-desktop
```
#### **Setup Environment**
Set up your environment
```
source /opt/ros/jazzy/setup.bash
```
---
### **2. Create and build a ROS 2 workspace**
Create and move into a new workspace using
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
This creates a standard ROS 2 workspace layout:
```
ros2_ws/
  ├── src/
  ├── build/
  ├── install/
  └── log/
```
build the basic workspace and source it using:
```
  source /opt/ros/jazzy/setup.bash
  cd ~/ros2_ws
  colcon build
  source ~/ros2_ws/install/setup.bash
```
Since you need to do this every time after building, it is adviced to setup the following:

First, put the source files into the `/.bashrc`. This way, any time a new terminal is opened, the workspace is automatically sourced. To do this, use the command:
```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

Next thing to do is to setup a command alias to build the workspaces to prevent accidentaly building workspaces in the work folder. This can be done and saved by putting the alias into the `.bashrc` file using:
```
echo "alias build='cd ~/ros2_ws && colcon build && source ~/.bashrc'"
```
now by using the command `build`, you move to the main `ros2_ws` workspace folder, build all your modules and source the needed files.

---
### **3. Install Steam and steamVR**
First, make sure the entire system is up to date by running the following commands:
```
sudo apt update
sudo apt upgrade
```

Now, install the Steam by downloading their LINUX package from their [official site](https://store.steampowered.com/about/).

The downloaded package should be named something similar to `steam_latest.deb`.

change your directory to the folder that has the downloaded `.deb` file. It is most likely in Downloads, which can be accessed with:
```
cd ~/Downloads
```
from there, to install the Steam `.deb` package file, use
```
sudo dpkg -i steam_latest.deb
```
If needed, replace `steam_latest.deb` with the appropriate `.deb` file that was donwloaded.

Steam can now be launched from the applications menu, or by using the command:
```
steam
```
On first launch, there are some setup steps, so just follow the on-screen instructions

Fom then on, steam can be launched and you can either make a new account or log in.

When you open the launcher, you can find and download `SteamVR` by going to their storepage and searching for it in the searchbar in the top-right.

To set up your system when launching steamVR, simply follow the steps given by the program until the full system is configured.

---
### **4. Install the openVR SDK**
The official OpenVR SDK can be found on the [github page](https://github.com/valvesoftware/openvr). Dowload the latest version by clicking on the latest version underneath `Releases` on the right side of the page. This package uses [OpenVR SDK 2.15.6](https://github.com/ValveSoftware/openvr/releases/tag/v2.15.6), but later versions should also be compatible.

Download the `Source code (zip)` file and move it to a prefered location and unzip it there. It is not adviced to keep it in the `Downloads` folder on your computer, but to move it somewhere else. For example, a separate `openvr` folder somewhere on your preferred drive.

---
### **5. Install the LINUX UDP module**
Will be added later as bridge type will most likely be switched.

---
### **6. Install ROS controll and MoveIt**
Now back to the terminal. The AR4 driver is built on `ros2_control`, which provides the hardware interface layer between `MoveIt` and the robot. It can be installed using the following command:
```
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager
```

Likewise, MoveIt can be installed using:
```
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y ros-jazzy-moveit ros-jazzy-moveit-py
sudo ldconfig
```
`MoveIt` is used as a control system for robotic motion planning. It can both read the current robot state and joint states, as well as use inverse kinematics to process path planning for the robot.

---
### **7. Clone and build the AR4 ROS driver**
The repository can be copied to your system using:
```
cd ~/ros2_ws/src
git clone https://github.com/ShellyFisherr/jazzy_ros2_art4_ros_driver_edit ar4_ros_driver
```
Your workspace should now look like:
```
~/ros2_ws/src/ar4_ros_driver
```
To use the package, multiple separate ROS dependencies have to be installed. This needs to be done once per system, or when a new version of this repository is installed to make sure all dependencies are installed. First of all, download `rosdep`.
```
sudo apt update
sudo apt install python3-rosdep
```
Then initialize it:
```
sudo rosdep init
rosdep update
```
from here, to install all dependencies, simply use:
```
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```
Now, with all dependencies installed and the alias set up previously, the system can be build using the command
```
build
```
Which builds the workspace in the correct location and sources it.

If everything went correctly, the system can be verified by launching the `MoveIt` demo. This should be done before connecting real hardware to verify that ROS + MoveIt are working correctly:
```
ros2 launch annin_ar4_moveit_config demo.launch.py ar_model:=mk5
```
This should open RViz with the AR4 model and interactive planning tools.


---
### **8. Configure serial permissions**
Now to setup the link between software and hardware. First, enable the serial port access required for communicating with the Teensy and Arduino Nano:
```
sudo adduser $USER dialout
```
then log out and back in or reboot your system for the changes to take effect.

---
### **9. Install Arduino and teensy toolchain**

First, it is required to istall FUSE, which is needed for AppImage to use on the applications menu.

It can be installed using:
```
sudo apt update
sudo apt install -y libfuse2t64
```
From there, the Arduino IDE can be downloaded and ran using:
```
mkdir -p ~/apps/arduino
cd ~/apps/arduino
wget https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.6_Linux_64bit.AppImage -O arduino-ide.AppImage
chmod +x arduino-ide.AppImage
./arduino-ide.AppImage --no-sandbox
```

When running the Arduino IDE as an AppImage, it will not automatically appear in the Ubuntu Applications menu or taskbar. The steps below add a proper icon, launcher entry, and optional terminal shortcut.

The arduino icon can be added using
```
sudo mkdir -p /usr/share/icons/hicolor/256x256/apps
sudo wget -O /usr/share/icons/hicolor/256x256/apps/arduino.png \
  https://upload.wikimedia.org/wikipedia/commons/8/87/Arduino_Logo.svg
sudo update-icon-caches /usr/share/icons/hicolor
```

Now create and edit the launcher file:
```
nano ~/.local/share/applications/arduino-ide.desktop
```
Paste the following contents in the file:
```
[Desktop Entry]
Name=Arduino IDE
Comment=Arduino IDE 2.x
Exec=/home/chris/apps/arduino/arduino-ide.AppImage --no-sandbox
Icon=/usr/share/icons/hicolor/256x256/apps/arduino.png
Type=Application
Terminal=false
Categories=Development;IDE;Electronics;
StartupNotify=true
```
Save and exit the editor by pressing `Ctrl + O`, then `Enter`, then `Press Ctrl + X`.

From there, make the launcher executable and update the desktop database using:
```
chmod +x ~/.local/share/applications/arduino-ide.desktop
sudo update-desktop-database
```
Now launch and Pin the Ardiuno IDE by doing:
- Press the **Super (Windows)** key amd type `Arduino`.
- The Arduino IDE should appear in the Applications list.
- Right-click $\rightarrow$ **Add to Favorites** to pin it to the taskbar.

If you want to be able to launch Arduino IDE from any terminal, use:
```
sudo ln -s /home/chris/apps/arduino/arduino-ide.AppImage /usr/local/bin/arduino
```
Now you can launch the program by typing
```
arduino
```

Teensyduino can be installed by going to the [official site](https://www.pjrc.com/teensy/td_download.html) and follwong PJRC's instructions.

In Arduino IDE:
1. Open **Preferences**
2. Add the following URL to **Additional Boards Manager URLs**:
```
https://www.pjrc.com/teensy/package_teensy_index.json
```
3. open **Tools** $\rightarrow$ **Board** $\rightarrow$ **Boards Manager**
4. Search for and install **Teensy**
5. Select **Teensy 4.1** as the active board

Now install the required Arduino Libraries:

From **Tools** $\rightarrow$ **Manage Libraries**, install:
- **Bounce2**

Lastly, Install PJRC udev rules so Linux can access the teensy without sudo:
```
cd ~/Downloads
wget https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---
### **10. Flash firmware to Teensy and Arduino**

The firmware is located in
```
annin_ar4_firmware/
```
**Teensy 4.1**
- Open the Teensy firmware sketch in Arduino IDE
- Select **Board**: **Teensy 4.1**
- Select the correct USB port
- Click **Upload**

**Arduino Nano (Gripper, not used in this teleoperation)**
- Open the gripper firmware sketch
- Select **Board**: **Arduino Nano**
- Select **Processor**: **ATmega328P (Old Bootloader)** if required
- Click **Upload**

After flasing firmware or power cycling the robot:
  - A **Calibration run is required**
  - This is normal and expected behavior

Calibration is triggered automatically when launching the driver with:
```
calibrate:=True
```

---
### **11. Configure UDP bridge**
Will be added later since bridge type will most likely be changed


At this point, the system can be run on the real hardware.

---
## **Usage**

To be updated at later date (want everything in 1 launch file if possible)


## **Parameters**
Parameters for the `annin_ar4_teleoperator` package will be divided between the different yaml files (found in the config folder of the package). They have a short desctiption of what they represent, along with their unit if applicable and their data type and requirements. Lastly, the standard value will be given as well.

- [DLS_controller.yaml](#dls_controlleryaml)
- [translation_scaling.yaml](#translation_scalingyaml)
- [workspace.yaml](#workspaceyaml)

### **DLS_controller.yaml**
---
- controller_frequency: 
  - `DEFAULT:` `60.0` [Hz]
  - `TYPE:` `double`
  - `REQUREMENTS:` Must be positive and finite.
  - `USAGE:` Update frequency of the controller. Determines how many timer per second the controller loops.
  
- show_logs: 
  - `DEFAULT:` `true`
  - `TYPE:` `bool`
  - `REQUREMENTS:` `true` or `false`.
  - `USAGE:` wether or not to show the logs, usefull for debugging, must be `true` for data collection.

- deadzone_velocity: 
  - `DEFAULT:` `0.002` [m/s]
  - `TYPE:` `double`
  - `REQUREMENTS:` Must be finite and greater or equal to zero.
  - `USAGE:` A deadzone filter to filter out remaining noise in translation, should be calibrated depending on trackers used. Can be disabled, but more noise will make it through. Anything below the threshold value is set to 0.

- deadzone_twist: 
  - `DEFAULT:` `0.008` [rad/s]
  - `TYPE:` `double`
  - `REQUREMENTS:` Must be finite and greater or equal to zero.
  - `USAGE:` A deadzone filter to filter out remaining noise in rotation, should be calibrated depending on trackers used. Can be disabled, but more noise will make it through. Anything below the threshold value is set to 0. 

- timeout_age: 
  - `DEFAULT:` `0.1` [s]
  - `TYPE:` `double`
  - `REQUREMENTS:` Must be positive and finite.
  - `USAGE:` A check to make sure no stale data remains in the system. Any incoming data older than tha threshold will be disregarded.
  
- deadzone_translation_error: 
  - `DEFAULT:` `0.002` [m]
  - `TYPE:` `double`
  - `REQUREMENTS:` must be finite and greater or equal to zero.
  - `USAGE:` A filter for translation error. Works the same way as the deazone_velocity, but is applied to error calculation instead. Can be disabled, but more noise can make it through.

- deadzone_orientation_error: 
  - `DEFAULT:` `0.008` [rad]
  - `TYPE:` `double`
  - `REQUREMENTS:` Must be finite and greater or equal to zero.
  - `USAGE:` A filter for orientation error. Works the same way as the deazone_twist, but is applied to error calculation instead. Can be disabled, but more noise can make it through.

- position_gain: 
  - `DEFAULT:` `1.0`
  - `TYPE:` `double`
  - `REQUREMENTS:` Must be positive and finite.
  - `USAGE:` A multiplier for errors correction in translation. Higher values result in a more aggressive error correction, but may result in overshooting requested positions.

- orientation_gain: 
  - `DEFAULT:` `1.0`
  - `TYPE:` `double`
  - `REQUREMENTS:` Must be positive and finite.
  - `USAGE:` A multiplier for errors correction in orientation. Higher values result in a more aggressive error correction, but may result in overshooting requested positions.

- DLS_damping: 
  - `DEFAULT:` `0.005`
  - `TYPE:` `double`
  - `REQUREMENTS:` Must be positive and finite.
  - `USAGE:` Damping coefficient for the DLS (damped-least-squared) method. the higher the number the more agressive it becomes.

- use_velocity_limit: 
  - `DEFAULT:` `true`
  - `TYPE:` `bool`
  - `REQUREMENTS:` `true` or `false`.
  - `USAGE:` Wether or not to use the velocity limits to protect motors. When enabled, all final velocities are scaled so that the largest velocity stays within the set limit.

- use_acceleration_limit: 
  - `DEFAULT:` `true`
  - `TYPE:` `bool`
  - `REQUREMENTS:` `true` or `false`.
  - `USAGE:` Wether or not to use the aceleration limits to protect the motors. When enabled, all final velocities are scaled so that the largest velocity stays within the set limit.
  
- max_velocity: 
  - `DEFAULT:` `[60.0, 60.0, 60.0, 60.0, 60.0, 60.0]` [deg/s]
  - `TYPE:` `array double`
  - `REQUREMENTS:` Must be positive and finite and contain 6 values.
  - `USAGE:` The max velocity each joint can generate.

- max_acceleration: 
  - `DEFAULT:` `[30.0, 30.0, 30.0, 30.0, 30.0, 30.0]` [deg/s]
  - `TYPE:` `array double`
  - `REQUREMENTS:` Must be positive and finite and contain 6 values.
  - `USAGE:` The max acceleration each joint can generate.

### **translation_scaling.yaml**
---

- target_frame: 
  - `DEFAULT:` `"teleop_world"`
  - `TYPE:` `string`
  - `REQUIREMENTS:` Must be the same as the general world coordinate frame.
  - `USAGE:` Is used to check if the incoming data is w.r.t. the correct coordinate frames.

- robot_near:
  - `DEFAULT:` `[0.0, -0.25, 0.3]` [m]
  - `TYPE:` `vector double`
  - `REQUIREMENTS:` Must be finite and contain 3 values.
  - `USAGE:` Is the starting point of the robot for distance calibration, but is also the starting point of the robot when running the system.

- robot_far:
  - `DEFAULT:` `[0.0, -0.6, 0.3]` [m]
  - `TYPE:` `vector double`
  - `REQUIREMENTS:` Must be finite and contain 3 values.
  - `USAGE:` Is the end point of the robot for distance calibration. Not used for anything else.

- capture_sample_count:
  - `DEFAULT:` `30`
  - `TYPE:` `int`
  - `REQUIREMENTS:` Must be positive and finite.
  - `USAGE:` The amount of samples taken on a calibration spot to increase accuracy and counter tracker noise.

- minimum_tracker_movement:
  - `DEFAULT:` `0.05` [m]
  - `TYPE:` `double`
  - `REQUIREMENTS:` Must be positive and finite.
  - `USAGE:` The minimum amount a tracker must move between start and end points of the calibration.

- minimum_robot_movement:
  - `DEFAULT:` `0.01` [m]
  - `TYPE:` `double`
  - `REQUIREMENTS:` Must be positive and finite.
  - `USAGE:` The minimum distance required between `robot_near` and `robot_far`.

- maximum_capture_spread:
  - `DEFAULT:` `0.02` [m]
  - `TYPE:` `double`
  - `REQUIREMENTS:` Must be positive and finite.
  - `USAGE:` The maximum average distance that can be between all gathered samples. If samples are over the distance, the measurement is counted as invalid and the calibration must be done again.

### **workspace.yaml**
- samples: 
  - `DEFAULT:` `100000`
  - `TYPE:` `int`
  - `REQUIREMENTS:` Must be positive and finite.
  - `USAGE:` The amount of samples taken to determine a rough sketch of the workspace.

- planning_group:
  - `DEFAULT:` `"ar_manipulator"`
  - `TYPE:` `string`
  - `REQUIREMENTS:` Must be the name of the planning group used by `MoveIt`.
  - `USAGE:` Used to get the relevant planning group from `MoveIt`.

- end_effector_link:
  - `DEFAULT:` `"ee_link"`
  - `TYPE:` `string`
  - `REQUIREMENTS:` Must be the name of the last used link in the system.
  - `USAGE:` Used to grab the last relevant link in the system to perform the kinematics with and determine positions from.

- output_csv:
  - `DEFAULT:` `"~/.config/annin_ar4_teleoperator/workspace.csv"`
  - `TYPE:` `string`
  - `REQUIREMENTS:` Must be in a reachable space within the `ROS 2` workspace.
  - `USAGE:` Place where the output file is stored.


