# v2x_stack

## Overview

ROS 2 support for integrating a V2X stack into the ROS environment. 

### Core Components

The stack consists of three main components:

- **UDP Handler (`udp_dispatcher`)**: Communicates via UDP with the V2X device and converts received packets into ROS topics.
- **BTP Service (`btp_data_service`)**: Extracts the BTP header and payload, then publishes the information to the `/btp_data` ROS topic.
- **V2X Services**: Each service listens to the `/btp_data` topic, evaluates the port (e.g., CA Service uses port 2001), and converts serialized data into ROS messages and vice versa.

## Environment Information

This setup was implemented and tested under the following conditions:

- **Operating System**: Ubuntu 24.04.2
- **ROS 2 Distribution**: Jazzy

## Package Dependencies

_(To be added, if there are specific ROS 2 or system dependencies)_

## Install ROS 2

### Enable the Ubuntu Universe Repository
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### Add the ROS 2 GPG Key
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Add the ROS 2 Repository
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install Development Tools
```bash
sudo apt update && sudo apt install ros-dev-tools
```

### Update System Packages
```bash
sudo apt update
sudo apt upgrade
```

### Install ROS 2

#### Desktop Install (Recommended for GUI and tools support)
```bash
sudo apt install ros-jazzy-desktop
```

#### ROS-Base Install (Minimal installation)
```bash
sudo apt install ros-jazzy-ros-base
```

## Clone and Build This Repository

### Create a ROS 2 Workspace and Clone the Repository
```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone <REPO_URL>
```

### Build the Workspace
```bash
cd ~/colcon_ws
colcon build
```

### Source the Workspace
```bash
source ~/colcon_ws/install/setup.bash
```

## Usage

### Running the BTP Data Service
To start the BTP data service, run the following command:
```bash
ros2 launch v2x_stack services_launch.py
```

### Running the V2X Services
To launch the V2X services, use:
```bash
ros2 launch v2x_stack_btp services_launch.py
```

### Adding New Services
To add new services to the launch file, modify the configuration file:
```bash
/config/services_params.yml
```

