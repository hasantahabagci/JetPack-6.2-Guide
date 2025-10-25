# ArduPilot SITL + MAVProxy + MAVROS2 Setup Guide

This document describes how to install ArduPilot SITL, MAVProxy, and connect to MAVROS2 on Ubuntu.

## 1. Clone ArduPilot Repository

```bash
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

## 2. Install Required Dependencies

```bash
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Reload profile:

```bash
. ~/.profile
```

Reboot system:

```bash
sudo reboot now
```

## 3. Checkout Copter Firmware Version

```bash
cd ~/ardupilot
git checkout Copter-4.4.4
git submodule update --init --recursive
```

## 4. Run SITL

```bash
sim_vehicle.py -v ArduCopter --map --console
```

## 5. Install MAVProxy and Supporting Libraries

```bash
sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
python3 -m pip install PyYAML mavproxy --user
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
```

Add user to dialout group:

```bash
sudo usermod -a -G dialout ituarc
```

Upgrade MAVProxy and pymavlink:

```bash
python3 -m pip install mavproxy pymavlink --user --upgrade
```

Install MAVProxy from master branch:

```bash
python3 -m pip install mavproxy --user git+https://github.com/ArduPilot/mavproxy.git@master
```

## 6. Connect to MAVROS2

```bash
ros2 launch mavros apm.launch.py fcu_url:="udp://:14550@127.0.0.1:14550"
```

## 7. Set MAVLink Message Rate in MAVProxy

```bash
set streamrate 20
```

