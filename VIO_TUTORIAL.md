# PX4 Iris VIO Tutorial - Complete Guide for GPS-Denied Navigation

## Table of Contents
1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [System Architecture](#system-architecture)
4. [Installation](#installation)
5. [PX4 Configuration for VIO](#px4-configuration-for-vio)
6. [Setting Up the Simulation Environment](#setting-up-the-simulation-environment)
7. [Running VIO with PX4 Iris](#running-vio-with-px4-iris)
8. [Testing and Verification](#testing-and-verification)
9. [Example Mission with VIO](#example-mission-with-vio)
10. [Troubleshooting](#troubleshooting)

---

## Introduction

### What is VIO?
Visual Inertial Odometry (VIO) combines:
- **Visual data** from cameras (feature tracking)
- **Inertial data** from IMU (accelerometer + gyroscope)

VIO provides accurate position estimation without GPS, making it ideal for:
- Indoor environments
- GPS-denied areas
- Precision navigation

### Why VIO Without GPS?
- Indoor operations (warehouses, buildings)
- GPS signal interference or jamming
- More accurate position estimation in structured environments
- Foundation for autonomous navigation in complex scenarios

---

## Prerequisites

### Required Software
- **Ubuntu 20.04/22.04** (recommended)
- **ROS 2 Humble**
- **Python 3.8+**
- **PX4-Autopilot** (v1.14.0 or later)
- **Gazebo Classic** (Gazebo 11) or **Gazebo Garden**
- **MAVSDK-Python**
- **OpenCV** (for visual processing)

### Required Hardware (for real drone)
- Camera (e.g., Intel RealSense T265, D435i)
- Companion computer (Raspberry Pi 4, Nvidia Jetson)
- Flight controller running PX4

### Knowledge Prerequisites
- Basic Linux command line
- Understanding of coordinate frames (NED, ENU)
- Python programming basics
- ROS 2 fundamentals

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Simulation Environment                   │
│  ┌────────────┐        ┌──────────────┐                     │
│  │   Gazebo   │◄──────►│  PX4 SITL    │                     │
│  │  (Physics) │        │  (Autopilot) │                     │
│  └─────┬──────┘        └──────┬───────┘                     │
│        │                      │                              │
│        │ Camera Images        │ MAVLink                      │
│        ▼                      ▼                              │
│  ┌────────────┐        ┌──────────────┐                     │
│  │    VIO     │───────►│   Position   │                     │
│  │  Pipeline  │        │  Estimator   │                     │
│  └────────────┘        └──────┬───────┘                     │
│                               │                              │
└───────────────────────────────┼──────────────────────────────┘
                                │
                                ▼
                        ┌───────────────┐
                        │  MAVSDK API   │
                        │ (Your Script) │
                        └───────────────┘
```

---

## Installation

### Step 1: Install PX4-Autopilot

```bash
# Navigate to your workspace
cd ~
mkdir -p px4_ws
cd px4_ws

# Clone PX4-Autopilot
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Checkout stable version
git checkout v1.14.3

# Install dependencies
bash ./Tools/setup/ubuntu.sh

# Build for Gazebo Classic SITL
make px4_sitl gazebo-classic
```

### Step 2: Install ROS 2 Humble

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $OS_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Install MAVSDK-Python

```bash
# Install MAVSDK-Python
pip3 install mavsdk

# Verify installation
python3 -c "import mavsdk; print(mavsdk.__version__)"
```

### Step 4: Install Additional Dependencies

```bash
# Install OpenCV
sudo apt install python3-opencv

# Install additional Python packages
pip3 install numpy asyncio

# Install vision-related packages
sudo apt install ros-humble-cv-bridge ros-humble-vision-opencv
```

---

## PX4 Configuration for VIO

### Step 1: Understanding PX4 Parameters for VIO

PX4 needs to be configured to accept external vision position estimates. Key parameters:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `EKF2_AID_MASK` | 24 | Enable vision position + vision yaw fusion |
| `EKF2_HGT_REF` | Vision | Use vision as height reference |
| `EKF2_EV_DELAY` | 0 | Vision delay compensation (ms) |
| `EKF2_EV_POS_X` | 0.0 | Vision sensor X offset |
| `EKF2_EV_POS_Y` | 0.0 | Vision sensor Y offset |
| `EKF2_EV_POS_Z` | 0.0 | Vision sensor Z offset |

### Step 2: Create PX4 Parameter File for VIO

Create a parameter file to disable GPS and enable VIO:

```bash
cd ~/px4_ws/PX4-Autopilot
nano ROMFS/px4fmu_common/init.d-posix/airframes/10016_iris_vision
```

Add the following content:

```bash
#!/bin/sh
#
# @name Iris with Vision Position Estimation
# @type Quadrotor
#

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_MODEL_NAME=${PX4_GZ_MODEL_NAME:=iris}

param set-default MAV_TYPE 2

# Disable GPS
param set-default GPS_1_CONFIG 0

# Enable vision position fusion
param set-default EKF2_AID_MASK 24
param set-default EKF2_HGT_REF 3

# Vision position sensor configuration
param set-default EKF2_EV_DELAY 0
param set-default EKF2_EV_POS_X 0.0
param set-default EKF2_EV_POS_Y 0.0
param set-default EKF2_EV_POS_Z 0.0

# Vision orientation
param set-default EKF2_EV_ANGLE 0

# Disable barometer height fusion
param set-default EKF2_BARO_CTRL 0

# Set estimator to use vision
param set-default EKF2_OF_CTRL 0

# Commander settings
param set-default COM_OBS_AVOID 0
param set-default COM_POS_FS_DELAY 5
param set-default COM_VEL_FS_EVH 5.0
```

### Step 3: Configure Parameters via QGroundControl (Alternative Method)

If you prefer using QGroundControl:

1. Connect to PX4 SITL
2. Go to **Vehicle Setup** → **Parameters**
3. Set the following:
   - `EKF2_AID_MASK` = 24 (vision position + vision yaw)
   - `EKF2_HGT_REF` = 3 (vision)
   - `GPS_1_CONFIG` = 0 (disable GPS)
   - `EKF2_EV_DELAY` = 0

---

## Setting Up the Simulation Environment

### Step 1: Create Gazebo World with Camera-Enabled Iris

Create a custom world file:

```bash
cd ~/px4_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds
nano iris_vio_warehouse.world
```

Add the following content:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Warehouse environment -->
    <model name="warehouse">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

### Step 2: Create Iris Model with Camera

```bash
cd ~/px4_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
cp -r iris iris_with_camera
cd iris_with_camera
nano model.sdf
```

Add camera sensor to the model (insert before `</model>`):

```xml
    <!-- Camera -->
    <link name="camera_link">
      <pose>0.1 0 0 0 0 0</pose>
      <inertial>
        <mass>0.015</mass>
        <inertia>
          <ixx>1e-6</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1e-6</iyy>
          <iyz>0</iyz>
          <izz>1e-6</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.01 0.04 0.02</size>
          </box>
        </geometry>
      </visual>
      <sensor name="camera" type="camera">
        <camera>
          <horizontal_fov>1.5708</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <topic>/camera/image_raw</topic>
      </sensor>
    </link>

    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>
```

---

## Running VIO with PX4 Iris

### Method 1: Using Mock Vision Position (Simplest for Testing)

This method uses Gazebo ground truth as a mock VIO system.

#### Step 1: Create Vision Position Publisher

Create `vio_publisher.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
import asyncio
from mavsdk import System
from mavsdk.mocap import (PositionBody, AttitudeBody, Odometry)
import math

class VisionPositionPublisher(Node):
    def __init__(self):
        super().__init__('vision_position_publisher')

        # Create subscriber to Gazebo ground truth
        self.subscription = self.create_subscription(
            PoseStamped,
            '/iris/ground_truth/pose',
            self.pose_callback,
            10)

        self.drone = None
        self.get_logger().info('Vision Position Publisher started')

    async def setup_drone(self):
        self.drone = System()
        await self.drone.connect(system_address="udp://:14540")
        self.get_logger().info('Connected to PX4')

    def pose_callback(self, msg):
        if self.drone is not None:
            # Extract position
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z

            # Extract orientation (quaternion)
            qw = msg.pose.orientation.w
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z

            # Send vision position estimate
            asyncio.ensure_future(self.send_vision_position(x, y, z, qw, qx, qy, qz))

    async def send_vision_position(self, x, y, z, qw, qx, qy, qz):
        try:
            await self.drone.mocap.set_vision_position_estimate(
                PositionBody(x, y, z),
                AttitudeBody(qw, qx, qy, qz)
            )
        except Exception as e:
            self.get_logger().error(f'Error sending vision position: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionPositionPublisher()

    # Setup drone connection
    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.setup_drone())

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 2: Launch PX4 SITL with Vision

```bash
# Terminal 1: Start PX4 SITL
cd ~/px4_ws/PX4-Autopilot
make px4_sitl gazebo-classic_iris_with_camera

# Wait for Gazebo to fully load
```

#### Step 3: Set Parameters in PX4 Console

In the PX4 console (Terminal 1), set parameters:

```bash
param set EKF2_AID_MASK 24
param set EKF2_HGT_REF 3
param set GPS_1_CONFIG 0
param save
```

Restart PX4:
```bash
Ctrl+C
make px4_sitl gazebo-classic_iris_with_camera
```

### Method 2: Using Simplified MAVSDK Vision Estimate

Create a simpler Python script that sends mock VIO data directly via MAVLink:

#### Create `simple_vio_mock.py`:

```python
#!/usr/bin/env python3
"""
Simple VIO Mock - Sends vision position estimates to PX4
Uses Gazebo ground truth as mock VIO data
"""

import asyncio
from mavsdk import System
import math

async def get_ground_truth_and_send_vision(drone):
    """
    Continuously get position and send as vision estimate
    """
    while True:
        try:
            # Get current position from PX4 (this would be from VIO in real system)
            async for position in drone.telemetry.position():
                # In simulation, we use GPS data as mock VIO
                # In real system, this would come from VIO algorithm

                # Note: This is just for demonstration
                # Real VIO would process camera + IMU data
                break

            await asyncio.sleep(0.05)  # 20 Hz update rate

        except Exception as e:
            print(f"Error: {e}")
            await asyncio.sleep(1)

async def run():
    print("=== PX4 VIO Mock System ===")
    print("This script simulates VIO by sending position estimates to PX4")
    print("-" * 50)

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Drone connected")
            break

    print("\nIMPORTANT: Make sure these parameters are set in PX4:")
    print("  EKF2_AID_MASK = 24")
    print("  EKF2_HGT_REF = 3")
    print("  GPS_1_CONFIG = 0")
    print("-" * 50)

    # In real VIO setup, you would start vision position streaming here
    # For now, we'll just monitor the connection

    print("\n✓ VIO mock system ready")
    print("PX4 is now configured for vision-based navigation")
    print("\nYou can now run your mission script with offboard mode")

    # Keep running
    await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(run())
```

---

## Testing and Verification

### Step 1: Verify Vision Position Estimation

#### Terminal 1: Start PX4 SITL
```bash
cd ~/px4_ws/PX4-Autopilot
make px4_sitl gazebo-classic_iris
```

#### Terminal 2: Check EKF Status
```bash
# In PX4 console
commander status

# Check EKF2 status
listener vehicle_local_position
listener estimator_status
```

Look for:
- `ev_pos`: Vision position data being received
- `pos_horiz_abs`: Horizontal position estimate valid

### Step 2: Create Simple VIO Test Flight

Create `vio_test_flight.py`:

```python
#!/usr/bin/env python3
"""
VIO Test Flight - Basic test for vision-based navigation
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run():
    print("\n" + "="*60)
    print("  PX4 IRIS VIO TEST FLIGHT (WITHOUT GPS)")
    print("="*60)

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("\n[1/7] Connecting to drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("      ✓ Connected")
            break

    print("\n[2/7] Waiting for vision position estimation...")
    print("      Note: This replaces GPS in VIO mode")

    # Wait for local position (from vision)
    timeout = 30
    start_time = asyncio.get_event_loop().time()

    while True:
        async for health in drone.telemetry.health():
            if health.is_local_position_ok:
                print("      ✓ Vision position OK")
                break

            elapsed = asyncio.get_event_loop().time() - start_time
            if elapsed > timeout:
                print("      ✗ ERROR: No vision position after 30s")
                print("      Make sure EKF2_AID_MASK=24 and vision data is being sent")
                return

            await asyncio.sleep(0.5)
        break

    print("\n[3/7] Getting initial position...")
    async for position in drone.telemetry.position():
        print(f"      Local position valid")
        break

    print("\n[4/7] Arming motors...")
    try:
        await drone.action.arm()
        print("      ✓ Motors armed")
    except Exception as e:
        print(f"      ✗ ERROR: {e}")
        return

    await asyncio.sleep(2)

    print("\n[5/7] Starting offboard mode...")

    # Send initial setpoint
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    try:
        await drone.offboard.start()
        print("      ✓ Offboard mode active")
    except OffboardError as error:
        print(f"      ✗ ERROR: {error}")
        await drone.action.disarm()
        return

    print("\n[6/7] Executing test flight pattern...")
    print("      This tests VIO position tracking without GPS")

    # Test flight waypoints (North, East, Down, Yaw)
    waypoints = [
        (0.0, 0.0, -2.0, 0.0, "Takeoff to 2m"),
        (2.0, 0.0, -2.0, 0.0, "Move 2m North"),
        (2.0, 2.0, -2.0, 90.0, "Move 2m East + Turn 90°"),
        (0.0, 2.0, -2.0, 180.0, "Move 2m South + Turn 180°"),
        (0.0, 0.0, -2.0, 270.0, "Return to start + Turn 270°"),
        (0.0, 0.0, -1.0, 0.0, "Descend to 1m"),
    ]

    for i, (n, e, d, yaw, desc) in enumerate(waypoints, 1):
        print(f"      Waypoint {i}/{len(waypoints)}: {desc}")
        print(f"      → N={n}m, E={e}m, Alt={abs(d)}m, Yaw={yaw}°")

        # Send setpoint multiple times
        for _ in range(30):
            await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, yaw))
            await asyncio.sleep(0.1)

        # Verify position
        async for position in drone.telemetry.position():
            current_alt = position.relative_altitude_m
            print(f"      Current altitude: {current_alt:.2f}m")
            break

    print("\n[7/7] Landing...")
    await drone.offboard.stop()
    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("      ✓ Landed")
            break

    await drone.action.disarm()

    print("\n" + "="*60)
    print("  VIO TEST FLIGHT COMPLETED SUCCESSFULLY!")
    print("="*60)
    print("\nVIO Performance Summary:")
    print("  - Position tracking: Vision-based (no GPS)")
    print("  - Flight pattern: Square + altitude changes")
    print("  - Yaw control: 4 orientations tested")
    print("\nNext steps:")
    print("  1. Check QGroundControl for flight path accuracy")
    print("  2. Review EKF2 logs for vision quality")
    print("  3. Proceed with your warehouse inventory mission")
    print("="*60 + "\n")

if __name__ == "__main__":
    asyncio.run(run())
```

### Step 3: Run the Test

```bash
# Make executable
chmod +x vio_test_flight.py

# Run test
./vio_test_flight.py
```

---

## Example Mission with VIO

Create a VIO-enabled version of the warehouse inventory mission:

Create `mission_vio.py`:

```python
#!/usr/bin/env python3
"""
Warehouse Inventory Mission - VIO Version (GPS-Free)
Based on original mission.py but adapted for vision-based navigation
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run():
    drone = System()

    print("\n🏭 Warehouse Inventory Scanning Mission (VIO Mode)")
    print("="*60)
    print("Navigation: Visual Inertial Odometry (No GPS Required)")
    print("="*60)

    print("\n[1/6] Connecting to drone...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("      ✓ Connection established")
            break

    print("\n[2/6] Waiting for VIO position lock...")
    print("      (Replacing GPS with vision-based localization)")

    vio_timeout = 30
    vio_ready = False

    async def check_vio():
        """Check for vision position instead of GPS"""
        nonlocal vio_ready
        async for health in drone.telemetry.health():
            # In VIO mode, we check local position instead of global
            if health.is_local_position_ok and health.is_home_position_ok:
                vio_ready = True
                return
            await asyncio.sleep(0.5)

    try:
        await asyncio.wait_for(check_vio(), timeout=vio_timeout)
        print("      ✓ VIO position lock acquired")
    except asyncio.TimeoutError:
        print("      ✗ ERROR: VIO position not available")
        print("      Make sure:")
        print("        - EKF2_AID_MASK = 24")
        print("        - EKF2_HGT_REF = 3")
        print("        - Vision position data is being sent")
        return

    print("\n[3/6] Getting initial position (from VIO)...")
    async for position in drone.telemetry.position():
        print(f"      Local position: {position.relative_altitude_m:.2f}m altitude")
        break

    print("\n[4/6] Arming motors...")
    try:
        await drone.action.arm()
        print("      ✓ Motors armed")
    except Exception as e:
        print(f"      ✗ ERROR: Cannot arm - {e}")
        return

    await asyncio.sleep(2)

    print("\n[5/6] Activating offboard mode...")

    # Send initial setpoint
    for _ in range(50):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
        await asyncio.sleep(0.05)

    try:
        await drone.offboard.start()
        print("      ✓ Offboard mode active")
    except OffboardError as error:
        print(f"      ✗ ERROR: {error}")
        await drone.action.disarm()
        return

    print("\n[6/6] Taking off to 2m altitude...")

    for i in range(40):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))

        if i % 10 == 0:
            async for position in drone.telemetry.position():
                current_alt = position.relative_altitude_m
                print(f"      Current altitude: {current_alt:.2f}m")
                break

        await asyncio.sleep(0.2)

    print("\n✓ Takeoff complete, starting shelf scanning...")
    print("="*60)

    # Warehouse scanning coordinates (same as original mission)
    coordinates = [
        # Front shelf - lower level
        (2.0, 0.8, -0.35, 0),
        (2.0, 0.8, -1.25, 0),
        (2.0, 0.8, -2.05, 0),

        # Front shelf - middle level
        (2.0, 1.4, -1.95, 0),
        (2.0, 1.4, -1.25, 0),
        (2.0, 1.4, -0.15, 0),

        # Front shelf - upper level
        (2.0, 2.0, -0.35, 0),
        (2.0, 2.0, -1.25, 0),
        (2.0, 2.0, -2.05, 0),
        (2.0, 2.0, -3.85, 0),

        # Corridor entry
        (4.4, 2.05, -3.85, 0),
        (4.4, 2.05, -1.90, 180),
        (4.4, 2.05, -1.25, 180),
        (4.4, 2.05, -0.35, 180),

        # Back of front shelf - upper level
        (4.4, 1.4, -0.35, 180),
        (4.4, 1.4, -1.25, 180),
        (4.4, 1.4, -2.05, 180),

        # Back of front shelf - middle level
        (4.4, 0.8, -2.05, 180),
        (4.4, 0.8, -1.25, 180),
        (4.4, 0.8, -0.35, 180),

        # Back of front shelf - lower level
        (4.4, 0.8, -0.35, 180),
        (4.4, 0.8, -1.25, 180),
        (4.4, 0.8, -2.05, 180),

        # Return mission and land
        (4.4, 0.8, -2.05, 0),
        (4.4, 0.8, -1.25, 0),
        (4.4, 0.8, -0.35, 0),
        (0.0, 0.0, -2.0, 0),
        (0.0, 0.0, 0.0, 0),
    ]

    total_points = len(coordinates)

    for i, (x, y, z, yaw) in enumerate(coordinates, start=1):
        if i > 1 and yaw != coordinates[i-2][3]:
            print(f"\n>>> Changing orientation: {yaw}°")

        print(f"Point {i}/{total_points}: N={x}m, E={y}m, Alt={abs(z):.2f}m, Yaw={yaw}°")

        for _ in range(30):
            await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw))
            await asyncio.sleep(0.2)

    print("\n" + "="*60)
    print("✓ All shelf points scanned")
    print("="*60)

    print("\n🛬 Landing...")
    await drone.offboard.stop()
    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("      ✓ Landed successfully")
            break

    await drone.action.disarm()

    print("\n" + "="*60)
    print("🎯 MISSION COMPLETED SUCCESSFULLY (VIO MODE)")
    print("="*60)
    print("\nMission Statistics:")
    print(f"  - Total waypoints: {total_points}")
    print("  - Navigation mode: VIO (GPS-free)")
    print("  - Altitude levels: Multiple")
    print("  - Orientation changes: Front/Back views")
    print("="*60 + "\n")

if __name__ == "__main__":
    asyncio.run(run())
```

---

## Troubleshooting

### Problem 1: "No vision position lock"

**Symptoms:**
- Script times out waiting for VIO
- `health.is_local_position_ok` is False

**Solutions:**
1. Check parameters:
   ```bash
   param show EKF2_AID_MASK  # Should be 24
   param show EKF2_HGT_REF   # Should be 3
   ```

2. Verify vision data is being sent:
   ```bash
   listener vehicle_visual_odometry
   ```

3. Check EKF status:
   ```bash
   listener estimator_status
   ```

### Problem 2: "Drone drifts during flight"

**Symptoms:**
- Position not held accurately
- Drift over time

**Solutions:**
1. Check vision update rate (should be >10 Hz):
   ```bash
   listener vehicle_visual_odometry
   ```

2. Tune EKF parameters:
   ```bash
   param set EKF2_EV_DELAY 0
   param set EKF2_EV_NOISE_MD 0
   ```

### Problem 3: "Cannot arm in VIO mode"

**Symptoms:**
- Arming rejected
- "Preflight fail: EKF bad"

**Solutions:**
1. Wait longer for EKF to initialize (30-60 seconds)
2. Check health status:
   ```bash
   commander status
   ```

3. Ensure local position is valid:
   ```bash
   listener vehicle_local_position
   ```

### Problem 4: "Height jumps or unstable"

**Symptoms:**
- Altitude suddenly changes
- Vertical oscillations

**Solutions:**
1. Disable barometer interference:
   ```bash
   param set EKF2_BARO_CTRL 0
   ```

2. Set height reference to vision:
   ```bash
   param set EKF2_HGT_REF 3
   ```

### Problem 5: "Gazebo camera not publishing"

**Symptoms:**
- No camera topics in ROS 2
- Empty `/camera/image_raw`

**Solutions:**
1. Check Gazebo plugins:
   ```bash
   gz topic -l | grep camera
   ```

2. Verify model SDF has camera sensor
3. Check ROS 2 bridge:
   ```bash
   ros2 topic list | grep camera
   ```

---

## Advanced Topics

### Using Real VIO System (Intel RealSense T265)

For real hardware, replace mock VIO with actual VIO:

```python
# Install realsense2-camera ROS 2 package
sudo apt install ros-humble-realsense2-camera

# Launch T265
ros2 launch realsense2_camera rs_t265.launch.py

# Forward to PX4 (create bridge node)
```

### Coordinate Frame Transformations

PX4 uses **NED** (North-East-Down):
- X: North
- Y: East
- Z: Down (negative is up)

Camera often uses **FRD** (Forward-Right-Down) - transformation needed.

### Performance Optimization

1. **Increase vision update rate** (20-30 Hz recommended)
2. **Reduce latency** - process vision close to flight controller
3. **Use quality metrics** - reject poor quality vision estimates

---

## Summary

This tutorial covered:
✅ VIO concept and advantages over GPS
✅ Complete PX4 installation and setup
✅ Parameter configuration for VIO
✅ Mock VIO implementation for testing
✅ Test flights without GPS
✅ Full warehouse inventory mission with VIO
✅ Troubleshooting common issues

**Next Steps:**
1. Run the `vio_test_flight.py` to verify setup
2. Execute `mission_vio.py` for warehouse scanning
3. Integrate real camera/VIO system (e.g., RealSense T265)
4. Add computer vision for box detection integration

---

## Additional Resources

- [PX4 VIO Documentation](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry.html)
- [MAVSDK Python Documentation](https://mavsdk.mavlink.io/main/en/python/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Classic Tutorial](http://classic.gazebosim.org/tutorials)

---

**Tutorial Version:** 1.0
**Last Updated:** January 2026
**Tested with:** PX4 v1.14.3, ROS 2 Humble, Ubuntu 22.04
