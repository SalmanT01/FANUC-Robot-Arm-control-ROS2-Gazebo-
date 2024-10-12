# **Robot Arm Control Using ROS2**

## **Overview**
This repository contains the implementation of a motion control algorithm for the FANUC CRX Robot Arm in the Gazebo simulator using ROS 2. Key steps in the process include converting CAD files into URDF, setting up the ROS 2 workspace, integrating the robot with Gazebo, and developing a motion control algorithm using a ROS 2 C++ node.

## **Steps Overview:**
- **Convert CAD STEP files** to URDF format using Autodesk Fusion 360.
- **Set up** the ROS 2 workspace and Gazebo simulation environment.
- **Integrate the URDF description** with ROS 2 and Gazebo.
- **Implement the motion control algorithm** for the robot in Gazebo using ROS 2 C++.

## **Solution Implementation**

### **1. Conversion of CAD STEP File to URDF**
- The FANUC CRX robot CAD STEP file is downloaded and converted into URDF format using Autodesk Fusion 360.
- 6 revolute joints are defined, the relationship in robot joint links are established to form its kinematic structure.
- Excluded the gripper in the initial implementation due to CAD limitations.
- Used the `fusion2urdf` script in Fusion 360 to automate URDF generation.

### **2. Setting Up the Virtual Environment**
- Set up Ubuntu 22.04 on VMware Workstation to run ROS 2 Humble and Gazebo due to network issues in VirtualBox.
- Installed ROS 2 Humble, Gazebo, and ROS 2 Control packages, which include:
  - `gazebo_ros`, `ros2_control`, `controller_manager`, and others to facilitate robot control in the simulation.

### **3. ROS 2 Xacro Files and Configuration**
- **CRX-10iA_L.xacro** file defines the robot’s physical and visual properties. It includes:
  - `materials.xacro`: Defines material and color properties.
  - `CRX-10iA_L.trans`: Transmission settings for robot joints.
  - Gazebo-specific parameters to handle robot collisions and physical interactions.
- **joint_controller.yaml**: Defines the joint controllers (`joint_trajectory_controller` and `joint_state_broadcaster`).

### **4. Simulation Setup**

#### **4.1 Transferring the Robot Description**
Transferred the robot description folder (generated from Autodesk Fusion 360) to the ROS 2 workspace. This includes:
- URDF and Xacro files.
- Mesh files for visualization and collision.
- Launch files for automating the simulation startup.

#### **4.2 Building the Workspace**
Run the following commands in the ROS 2 workspace to compile and set up the environment:

```bash
colcon build
source install/setup.bash
```

#### **4.3 Launching the Simulation**
Launch the robot simulation in Gazebo using the `gazebo.launch.py` script. This file:
- Loads the robot model and initializes the controller manager.
- Verifies correct visualization and joint connections in Gazebo.

### **5. Motion Control Implementation**

#### **5.1 Creating the ROS 2 C++ Package**
Created the `Robot_Control` package using:

```bash
ros2 pkg create --build-type ament_cmake Robot_Control --dependencies rclcpp std_msgs sensor_msgs trajectory_msgs
```

#### **5.2 Configuring CMakeLists.txt and package.xml**
Configured `CMakeLists.txt` to compile `motion_control.cpp` and linked necessary ROS 2 libraries. Updated `package.xml` with all dependencies.

#### **5.3 Writing the Motion Control Node**
Implemented the `motion_control.cpp` node to publish joint trajectories to control the robot arm.

#### **5.4 Integration with Gazebo**
Modified `gazebo.launch.py` to ensure proper loading and activation of the necessary controllers for joint trajectory control.
