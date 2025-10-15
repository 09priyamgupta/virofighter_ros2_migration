# ViroFighter ROS1 to ROS2 Migration Guide

## Phase 1: Environment Setup and Analysis

### Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill installed
- Existing ROS1 Melodic workspace
- `ros1_bridge` package built from source

### Step 1: Create ROS2 Workspace
```bash
mkdir -p ~/virofighter_ros2_ws/src
cd ~/virofighter_ros2_ws
```
### Step 2: Analyze ROS1 Dependencies
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
rosdep db
```

### Step 3: Identify Third-party Packages
| ROS1 Package | ROS2 Equivalent | Status | Action Required |
|--------------|----------------|--------|----------------|
| opencv_apps  | vision_opencv  | Available | Update CMakeLists |
| amcl        | nav2_amcl      | Available | Configuration changes |
| gmapping    | slam_toolbox   | Available | Algorithm migration |
| move_base   | nav2_bt_navigator | Available | Major rewrite needed |

## Phase 2: Custom Package Migration

### Step 4: Create New ROS2 Package Structure
```bash
cd ~/virofighter_ros2_ws/src
ros2 pkg create virofighter_driver --build-type ament_cmake --dependencies rclcpp std_msgs sensor_msgs
```

### Step 5: Migrate Package Configuration

**ROS1 package.xml**
```xml
<package>
  <name>virofighter_driver</name>
  <version>1.0.0</version>
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
</package>
```

**ROS2 package.xml**
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>virofighter_driver</name>
  <version>1.0.0</version>
  <description>ViroFighter driver package for ROS2</description>
  <maintainer email="team@virofighter.com">ViroFighter Team</maintainer>
  <license>Apache-2.0</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
</package>
```

### Step 6: Update Build System

**ROS1 CMakeLists.txt**
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(driver_node src/driver_node.cpp)
target_link_libraries(driver_node ${catkin_LIBRARIES})
```

**ROS2 CMakeLists.txt**
```cmake
cmake_minimum_required(VERSION 3.8)
project(virofighter_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(driver_node src/driver_node.cpp)
ament_target_dependencies(driver_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  driver_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Phase 3: Code Migration Patterns

### Node Initialization

**ROS1 Pattern:**
```cpp
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    ros::spin();
    return 0;
}
```

**ROS2 Pattern:**
```cpp
#include "rclcpp/rclcpp.hpp"

class DriverNode : public rclcpp::Node {
public:
    DriverNode() : Node("driver_node") {
        this->declare_parameter<double>("max_speed", 1.0);
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu_data", 10, std::bind(&DriverNode::imu_callback, this, std::placeholders::_1));
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Process IMU data
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Parameter Handling

**ROS1:**
```cpp
ros::NodeHandle private_nh("~");
double max_speed;
private_nh.param<double>("max_speed", max_speed, 1.0);
```

**ROS2:**
```cpp
this->declare_parameter<double>("max_speed", 1.0);
double max_speed = this->get_parameter("max_speed").as_double();
```

## Phase 4: Launch System Migration

**ROS1 Launch File (control.launch)**
```xml
<launch>
    <node pkg="virofighter_control" type="controller_node" name="controller">
        <param name="p_gain" value="2.0"/>
        <remap from="cmd_vel" to="virofighter/cmd_vel"/>
    </node>
    
    <node pkg="virofighter_navigation" type="planner_node" name="planner"/>
    
    <include file="$(find virofighter_sensors)/launch/sensors.launch"/>
</launch>
```

**ROS2 Launch File (control.launch.py)**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='virofighter_control',
            executable='controller_node',
            name='controller',
            parameters=[{'p_gain': 2.0}],
            remappings=[('cmd_vel', 'virofighter/cmd_vel')]
        ),
        
        Node(
            package='virofighter_navigation',
            executable='planner_node',
            name='planner'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('virofighter_sensors'),
                '/launch/sensors.launch.py'
            ])
        )
    ])
```

## Phase 5: Testing and Validation

### Step 7: Build and Test Individual Packages
```bash
cd ~/virofighter_ros2_ws
colcon build --packages-select virofighter_driver
source install/setup.bash
ros2 launch virofighter_driver test.launch.py
```

### Step 8: Integration Testing with ros1_bridge
```bash
# Terminal 1: ROS1 Core
roscore
```

```bash
# Terminal 2: ROS2 Core
source ~/virofighter_ros2_ws/install/setup.bash
ros2 run ros1_bridge dynamic_bridge
```

```bash
# Terminal 3: Run mixed system
roslaunch virofighter_bringup bringup.launch  # ROS1 nodes
ros2 launch virofighter_navigation nav2.launch.py  # ROS2 nodes
```

## Best Practices, QoS
- Incremental migration using `ros1_bridge`
- QoS tuning for critical topics
- Lifecycle and component nodes in ROS2


## Troubleshooting - Common Issues

1. **Build issues**
```bash
# If colcon build fails:
colcon build --cmake-clean-cache
rosdep install --from-paths src --ignore-src -r -y
```

2. **Communication issues**
```bash
# Check ROS2 topics
ros2 topic list
ros2 topic echo /virofighter/odometry

# Verify ros1_bridge pairings
ros2 run ros1_bridge dynamic_bridge --print-pairs
```

3. **Parameter issues**
```bash
# List parameters for a node
ros2 param list /controller_node

# Get parameter value
ros2 param get /controller_node p_gain
```