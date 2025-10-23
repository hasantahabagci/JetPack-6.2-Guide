# ROS 2 Humble + MAVROS + Custom Camera Driver Installation Guide

This guide walks through the complete setup of **ROS 2 Humble**, **MAVROS**, and a **custom camera driver** for development on Ubuntu (e.g., 22.04 LTS). It is designed for developers working with PX4 drones and external camera modules.

---

## 🤝 1. ROS 2 Humble Installation

```bash
# 1.1 Check for UTF-8 locale
locale

# 1.2 Install and configure locales
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 1.3 Verify the locale is correctly set
locale
```

```bash
# 1.4 Install required dependencies and add universe repo
sudo apt install software-properties-common
sudo add-apt-repository universe

# 1.5 Install curl
sudo apt update && sudo apt install curl -y
```

```bash
# 1.6 Add ROS 2 apt source from the official GitHub repository
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

# 1.7 Download and install ROS 2 apt source .deb package
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

```bash
# 1.8 Update and upgrade package lists
sudo apt update
sudo apt upgrade
```

```bash
# 1.9 Install the full desktop version of ROS 2 Humble
sudo apt install ros-humble-desktop

# 1.10 Optional development tools
sudo apt install ros-dev-tools

# 1.11 rosbag2
sudo apt-get install ros-$ROS2_DISTRO-ros2bag ros-$ROS2_DISTRO-rosbag2* # rosbag utilities (seems to be separate)

# 1.12 libeigen3 libceres
sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev
```



```bash
# 1.11 Add ROS setup to .bashrc for automatic sourcing on terminal launch
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 1.12 Verify environment variables
printenv | grep -i ROS

# 1.13 Run a basic ROS 2 demo node
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

```bash
sudo apt update
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

---

## 🚁 2. MAVROS Installation for ROS 2

```bash
# 2.1 Install required dependencies for building from source
sudo apt install -y python3-vcstool python3-rosinstall-generator python3-osrf-pycommon
```

```bash
# 2.2 Create workspace and move into it
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

```bash
# 2.3 Fetch MAVLink and MAVROS source code
rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos
rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos

# 2.4 Import repositories
vcs import src < /tmp/mavlink.repos
vcs import src < /tmp/mavros.repos
```

```bash
# 2.5 Install dependencies
rosdep install --from-paths src --ignore-src -y
```

```bash
# 2.6 Install GeographicLib datasets (required by MAVROS)
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

```bash
# 2.7 Build the workspace
colcon build --symlink-install
```

```bash
# 2.8 Source the built workspace
source install/setup.bash
```

```bash
# 2.9 Launch MAVROS with PX4 serial connection
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
```

```bash
# 2.10 Set MAVLink stream and message rates
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
    "{stream_id: 0, message_rate: 1, on_off: true}"

ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval \
    "{message_id: 33,  message_rate: 20.0}"  # IMU scaled

ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval \
    "{message_id: 27,  message_rate: 200.0}" # Raw IMU

ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval \
    "{message_id: 105, message_rate: 200.0}" # Attitude quaternion
```

```bash
# 2.11 Verify IMU topic frequency
ros2 topic hz /mavros/imu/data_raw -w 1000
```

---

## 👁️ 3. Add ROS 2 Environment Setup to ~/.bashrc

```bash
# Add these lines to your ~/.bashrc to enable auto environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 📸 4. Custom Camera Driver Integration

```bash
# 4.1 Place your camera driver inside your ROS 2 workspace
# camera ros link : https://github.com/valentinomario/camera_driver
# Example path:
# ~/ros2_ws/src/camera_driver/
```

```bash
# 4.2 Build only the camera driver package
colcon build --symlink-install --packages-select camera_driver
```

```bash
# 4.3 Run your camera test script
python ros_cam_test.py
```

---

## 🧭 5. OpenVINS Installation Guide

```bash
# 5.1 Navigate to your ROS 2 workspace source directory
cd ~/ros2_ws/src

# 5.2 Clone the OpenVINS repository
git clone https://github.com/rpng/open_vins/

# 5.3 Go back to the workspace root
cd ..

# 5.4 Build OpenVINS packages with verbose output
colcon build --event-handlers console_cohesion+ --packages-select ov_core ov_init ov_msckf ov_eval
```


## ✅ Final Tips

- Always source the workspace after building new packages:
  ```bash
  source install/setup.bash
  ```
- Use `ros2 topic list` or `ros2 node list` to verify nodes are running
- Run `ros2 doctor` if you encounter any issues

---

Happy Robotics Development!
