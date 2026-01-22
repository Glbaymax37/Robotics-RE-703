# Articubot One - ROS 2 Mobile Robot Project
## RICARD RIOVALDO SIMATUPANG
## NIM 4222201043
## TEKNIK ROBOTIKA PAGI - B

A comprehensive ROS 2 project featuring an articulated mobile robot with behavior tree-based navigation and control system.

## 📋 Project Overview

This workspace contains two main packages working together to create an intelligent mobile robot system:

1. **articubot_one** - Robot description and configuration package
2. **bt_controller** - Behavior tree-based controller for autonomous navigation

### Key Features

- **Robot URDF/Xacro Models**: Complete robot description with sensors and actuators
- **Behavior Tree Navigation**: Intelligent obstacle avoidance using behavior trees
- **Sensor Integration**: Lidar, camera, and depth camera support
- **Gazebo Simulation**: Full simulation environment support
- **ROS 2 Control**: Modern ros2_control framework integration
- **Navigation Stack**: Nav2 integration for autonomous navigation

---

## 📦 Package Descriptions

### 1. articubot_one

The main robot description and configuration package.

#### Directory Structure
```
articubot_one/
├── config/              # Configuration files
│   ├── ball_tracker_params_robot.yaml
│   ├── ball_tracker_params_sim.yaml
│   ├── drive_bot.rviz
│   ├── gazebo_params.yaml
│   ├── joystick.yaml
│   ├── main.rviz
│   ├── map.rviz
│   ├── mapper_params_online_async.yaml
│   ├── my_controllers.yaml
│   ├── nav2_params.yaml
│   ├── twist_mux.yaml
│   └── view_bot.rviz
├── description/         # Robot URDF/Xacro files
│   ├── camera.xacro
│   ├── depth_camera.xacro
│   ├── face.xacro
│   ├── gazebo_control.xacro
│   ├── inertial_macros.xacro
│   ├── lidar.xacro
│   ├── robot.urdf.xacro
│   ├── robot_core.xacro
│   └── ros2_control.xacro
├── launch/              # Launch files
│   ├── ball_tracker.launch.py
│   ├── camera.launch.py
│   ├── joystick.launch.py
│   ├── launch_robot.launch.py
│   ├── launch_sim.launch.py
│   ├── localization_launch.py
│   ├── navigation_launch.py
│   ├── online_async_launch.py
│   ├── rplidar.launch.py
│   └── rsp.launch.py
├── maps/                # Pre-built maps
├── worlds/              # Gazebo world files
├── CMakeLists.txt
├── package.xml
└── README.md
```

#### Configuration Files

- **RViz Configs**: `drive_bot.rviz`, `main.rviz`, `map.rviz`, `view_bot.rviz` - Different visualization setups
- **Controllers**: `my_controllers.yaml` - Motor and actuator control parameters
- **Navigation**: `nav2_params.yaml` - Navigation stack configuration
- **Sensors**: `ball_tracker_params_*.yaml` - Ball tracking sensor parameters
- **Simulation**: `gazebo_params.yaml` - Gazebo physics and environment
- **Input**: `joystick.yaml` - Joystick/gamepad input configuration
- **Motion Control**: `twist_mux.yaml` - Command velocity multiplexing

#### Robot Description

The robot is defined using URDF/Xacro with modular components:

- **robot_core.xacro** - Base chassis and wheels
- **ros2_control.xacro** - Motor control interface
- **lidar.xacro** - Lidar sensor integration
- **camera.xacro** - RGB camera
- **depth_camera.xacro** - Depth/RGB-D camera
- **gazebo_control.xacro** - Gazebo simulation plugins
- **inertial_macros.xacro** - Inertia calculations
- **face.xacro** - Robot face/display components

#### Launch Files

- **rsp.launch.py** - Robot state publisher (loads URDF)
- **launch_robot.launch.py** - Real robot bringup
- **launch_sim.launch.py** - Gazebo simulation
- **joystick.launch.py** - Joystick teleoperation
- **navigation_launch.py** - Nav2 navigation stack
- **localization_launch.py** - SLAM and localization
- **ball_tracker.launch.py** - Ball detection/tracking
- **rplidar.launch.py** - RPLidar sensor

---

### 2. bt_controller

Behavior tree-based autonomous navigation controller.

#### Directory Structure
```
bt_controller/
├── src/
│   └── bt_main.cpp          # Main controller executable
├── include/
│   └── bt_controller/        # Header files
├── bt_xml/
│   └── simple_bt.xml         # Behavior tree definition
├── CMakeLists.txt
└── package.xml
```

#### Components

**bt_main.cpp** - Main controller with three custom behavior nodes:

1. **CheckObstacle** (Condition Node)
   - Subscribes to `/scan` (Lidar data)
   - Checks front 40 degrees for obstacles
   - Threshold: 0.8 meters
   - Returns SUCCESS if obstacle detected, FAILURE otherwise

2. **MoveForward** (Action Node)
   - Publishes to `/cmd_vel`
   - Linear velocity: 0.5 m/s
   - No rotation

3. **Rotate** (Action Node)
   - Publishes to `/cmd_vel`
   - Angular velocity: 0.8 rad/s
   - Used for obstacle avoidance

**simple_bt.xml** - Behavior tree logic:
```
MainTree (Fallback)
├── Sequence (IF OBSTACLE DETECTED)
│   ├── CheckObstacle
│   └── Rotate
└── MoveForward (ELSE MOVE FORWARD)
```

**Logic Flow:**
- Try sequence: Check for obstacles → If found, rotate
- If sequence fails (no obstacle), move forward
- Continuous 10 Hz execution rate

#### Dependencies

- **rclcpp** - ROS 2 C++ library
- **behaviortree_cpp_v3** - Behavior tree framework
- **geometry_msgs** - Motion commands (Twist)
- **sensor_msgs** - Sensor data (LaserScan)
- **nav_msgs** - Navigation messages
- **tf2** - Coordinate transformations

---

## 🚀 Building the Project

### Prerequisites
- ROS 2 Humble (or compatible version)
- Colcon build system
- Gazebo (for simulation)

### Build Steps

```bash
# Navigate to workspace
cd /home/baymax/dev_ws

# Source ROS environment
source /opt/ros/humble/setup.bash

# Build all packages
colcon build

# Or build specific package
colcon build --packages-select articubot_one
colcon build --packages-select bt_controller

# Source the built packages
source install/setup.bash
```

---

## 🎮 Running the System

### 1. Real Robot Bringup
```bash
# Source environment
source install/setup.bash

# Launch robot
ros2 launch articubot_one launch_robot.launch.py
```

### 2. Gazebo Simulation
```bash
# Source environment
source install/setup.bash

# Launch simulation
ros2 launch articubot_one launch_sim.launch.py
```

### 3. Behavior Tree Controller
```bash
# In another terminal
source install/setup.bash

# Run the controller
ros2 run bt_controller bt_main
```

### 4. Joystick Teleoperation
```bash
# In another terminal
ros2 launch articubot_one joystick.launch.py
```

### 5. Navigation Stack
```bash
# For autonomous navigation
ros2 launch articubot_one navigation_launch.py
```

### 6. Localization & Mapping
```bash
# For SLAM
ros2 launch articubot_one online_async_launch.py
```

---

## 📊 Topics and Services

### Published Topics
- `/cmd_vel` - Command velocity (geometry_msgs/Twist)
- `/robot_description` - URDF description
- Various sensor topics (see launch files)

### Subscribed Topics
- `/scan` - Lidar point cloud (sensor_msgs/LaserScan)
- `/camera/image_raw` - Camera image
- Various control topics

### Parameters
See configuration files in `articubot_one/config/` for detailed parameter settings.

---

## 🛠️ Development Guide

### Adding New Behavior Nodes

To add a new behavior node to the controller:

1. **Create the node class** in `bt_controller/src/bt_main.cpp`
2. **Inherit from appropriate base class**:
   - `ConditionNode` - For checks/sensors
   - `SyncActionNode` - For immediate actions
   - `AsyncActionNode` - For long-running actions
3. **Register with factory**:
   ```cpp
   factory.registerNodeType<YourNode>("YourNode");
   ```
4. **Add to XML tree**: `bt_controller/bt_xml/simple_bt.xml`

### Modifying Robot Description

1. Edit XACRO files in `articubot_one/description/`
2. Rebuild: `colcon build --packages-select articubot_one`
3. Test in RViz or Gazebo

### Updating Controller Behavior

1. Modify tree logic in `simple_bt.xml`
2. Or adjust node parameters/thresholds in `bt_main.cpp`
3. Rebuild and restart controller

---

## 📝 File Reference

### Key Configuration Files

| File | Purpose |
|------|---------|
| `my_controllers.yaml` | Motor control parameters |
| `nav2_params.yaml` | Navigation stack settings |
| `gazebo_params.yaml` | Physics simulation parameters |
| `twist_mux.yaml` | Command velocity priorities |
| `joystick.yaml` | Input device configuration |

### Key XACRO Files

| File | Purpose |
|------|---------|
| `robot_core.xacro` | Chassis and wheeled base |
| `ros2_control.xacro` | Actuator interfaces |
| `lidar.xacro` | Lidar sensor model |
| `camera.xacro` | RGB camera model |

### Launch Files Quick Reference

| Launch File | Purpose |
|-------------|---------|
| `rsp.launch.py` | Robot state publisher |
| `launch_sim.launch.py` | Gazebo simulator |
| `navigation_launch.py` | Nav2 navigation |
| `localization_launch.py` | SLAM/localization |

---

## 🐛 Troubleshooting

### Build Errors
- Ensure all dependencies are installed: `rosdep install --from-paths src --ignore-src -r -y`
- Clear build artifacts: `colcon clean workspace && colcon build`

### Runtime Issues
- Check topics are being published: `ros2 topic list`
- Monitor node activity: `ros2 node list`
- View logs: `ros2 topic echo /topic_name`

### Simulation Issues
- Ensure Gazebo is properly installed
- Check world files exist in `articubot_one/worlds/`
- Verify physics parameters in `gazebo_params.yaml`

---

## 📚 Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/)
- [Behavior Trees Documentation](https://github.com/BehaviorTree/BehaviorTree.CPP)
- [Nav2 Documentation](https://nav2.org/)
- [Gazebo Documentation](http://gazebosim.org/)

---

## 📝 Notes

- Remember to update package descriptions in `package.xml` files
- Update license information in both packages
- Consider adding proper maintainer contact information
- The project follows ROS 2 Humble standards

---

**Last Updated**: January 19, 2026

