# Robot_Arm_control_Using_ROS2
Motion Control Algorithm for FANUC CRX Robot Arm in Gazebo simulator using ROS2

This project included following steps: 
1. To convert the STEP files of FANUC CRX Robot Arm CAD STEP file into URDF format and generate URDF description folder with XACRO files. 
2. Set up the ROS 2 Workspace + GAZEBO simulation environment.
3. To integrate the URDF description .XACRO of the Robot with ROS2 and Gazebo softwares.
4. Finally, to implement a motion algorithm for Robot Arm motion in Gazebo simulated environment using ROS 2 C++ node.

Solution Implementation
Step 1: Conversion of CAD STEP File to URDF

• I downloaded the CAD STEP file of the FANUC CRX robot from the provided link. To convert the CAD file into URDF format, I used Autodesk Fusion 360 CAD software that allows exporting designs in various formats.

• In Autodesk Fusion 360, I manually defined the revolute joints between the robot's links, establishing the robot's kinematic structure. There are total 6 revolute joint links defined as, “Link1-BaseLink”, “Link2-Link1”, “Link3-Link2”, “Link4-Link3”, “Link5-Link4”and “Link6-Link5”. Unexpectedly, the CAD file of Robot arm did not included the gripper for pick and place operation so I continued working with Robot arm without gripper, and intended to add later after testing Robot Simulation and Control keeping in mind the virtual machine system performance.

• To facilitate the conversion to URDF, I included a “fusion2urdf” script into Fusion 360 utilities add-in plugin, which automates the generation of URDF files from the CAD model. This process output a Robot Description folder containing URDF files, Xacro files, meshes, and necessary launch files.

Step 2: Setting Up the Virtual Environment
• Initially, I tried to use a Linux Ubuntu image in VirtualBox to run ROS 2, but I encountered network issues. To address this, I switched to VMware Workstation, where I installed Ubuntu 22.04 on a virtual machine resolving the network connectivity issue.
ROS 2 and Gazebo Installation:

• Once the Ubuntu environment was stable, I installed ROS 2 Humble using the official ROS 2 installation guide. ROS 2 Humble was chosen for its stability and compatibility with Gazebo and ROS 2 control packages.

• Gazebo was installed to simulate the robot's physical environment and interaction with objects. Gazebo is well-integrated with ROS 2, enabling seamless simulation control via ROS 2 topics, services, and actions.

• Additionally, I installed the ROS 2 Control package, which is essential for managing robot controllers within the ROS 2 framework, including ros2_control, gazebo_ros2_control, gazebo_ros_pkgs, and others that are critical for integrating ROS 2 with Gazebo for robot control. Despite my previous experience in ROS, I decided to use ROS2 for its improved development tools and libraries and more efficient plus secure communication between nodes for Robot Control. ROS2 came with packages which allowed flexible integration with Gazebo and Controllers to communicate joint configurations for Robot movement. The package includes controllers like the joint_trajectory_controller, which I later used to control the robot arm's movements.

• Key dependencies installed included:
  o gazebo_ros: Provides Gazebo plugins for ROS 2.
  o ros2_control: Manages hardware interfaces and controllers for the robot.
  o controller_manager: Facilitates the loading, unloading, and switching of controllers in ROS 2.


Step 3: ROS 2, Xacro Usage, Config File
The CRX-10iA_L.xacro file is an XML-based Xacro file that defines the URDF (Unified Robot Description Format) of the FANUC CRX-10iA_L robot. This file is used to describe the physical and visual aspects of the robot, including its links, joints, and interactions with the simulation environment in ROS 2 and Gazebo. Xacro Includes:
The file includes including other Xacro files:

  1. “materials.xacro” which contains material and color definitions for the robot.
  2. “CRX-10iA_L.trans” has transmission settings or additional transforms related to the robot.
  3. “CRX-10iA_L.xacro” file is specifically related to the Gazebo simulation. In this file gazebo_ros2_plugin, gazebo_ros2_control controller on each individual robot joint link, and physical properties like mass, inertia are defined which are crucial for both visualization and simulation physics. In this file, collision parameter are used to define the collision geometry, which Gazebo uses to detect collisions.

The “CRX-10iA_L.xacro” file plays a crucial role in defining the robot's physical and visual aspects within a ROS 2 and Gazebo simulation. It sets up the robot's links, joints, and inertial properties, which are essential for accurate simulation in Gazebo. The inclusion of Gazebo-specific Xacro files ensures that the robot behaves as expected in the simulation, with proper physics, controllers, and possibly sensors integrated into the simulation environment.

joint_controller.yaml file:
In “joint_controller.ymal” file, two of the controllers “joint_trajectory_controller” and “joint_state_broadcaster” are defined to setup the control manager. It also includes setting the list of joint names which are to be manage by the controller.

Step 4. Simulation Setup:

4.1 Transferring the Robot Description:

The robot description folder generated from Autodesk Fusion 360 (as mentioned in point 1) was transferred to the ROS 2 workspace under the src directory. This folder contained:
  1. URDF files: Describing the robot's physical properties, including links, joints, and associated geometries.
  2. Xacro files: XML macros used to simplify URDF file generation and reuse components within the URDF.
  3. Mesh files: 3D models representing the robot's visual and collision geometry.
  4. Launch files: Scripts to automate the startup of the robot simulation in Gazebo.

4.2 Building the Workspace:

• In ROS 2 workspace, following commands are run to build the workspace and ensure all dependencies were correctly linked:

colcon build
source install/setup.bash

• This step compiled all the packages, including the robot description and control packages, generating the necessary executables and configuration files for running the simulation.

4.3 Launching the Simulation:

• The “gazebo.launch.py” script is provided in the robot description folder. This launch file was configured to: 
    1. load the robot model into the Gazebo simulation and start the controller manager and 
    2. load the appropriate controllers, such as the “joint_trajectory_controller”.

• The successful execution of this launch file was verified by observing the FANUC CRX robot arm in the Gazebo environment, ensuring that all joints and links formed in Fusion 360 were correctly visualized and the robot was ready for control inputs.

Step 5. Motion Control Implementation:
   
5.1 Creating the ROS 2 C++ Package:

• In the ROS 2 workspace, a new C++ package named “Robot_Control” is created using the command:

ros2 pkg create --build-type ament_cmake Robot_Control --dependencies rclcpp std_msgs sensor_msgs trajectory_msgs

• This package was designed to handle the motion control logic for the FANUC CRX robot. Key dependencies included:
  1. rclcpp: The core ROS 2 C++ client library.
  2. std_msgs: Standard ROS 2 message types.
  3. trajectory_msgs: Messages for sending trajectory commands to the joint_trajectory_controller.


5.2 Configuring CMakeLists.txt and package.xml:

• The CMakeLists.txt is modified to compile the “motion_control.cpp” source file and link the necessary ROS 2 libraries. The “ament_target_dependencies” function was used to link the package with the required ROS 2 components:

The package.xml was updated to declare dependencies, ensuring that all required ROS 2 packages were available during build and runtime:

5.3 Writing the Motion Control Node (motion_control.cpp):
• The “motion_control.cpp” file is a ROS 2 C++ node that controls a robot arm in Gazebo by publishing joint trajectories. The node uses a timer to periodically send these trajectories, moving the arm between predefined positions. The “motion_control.cpp” file was created in the “src” directory of the “Robot_Control” package. This file contained the core logic to control the robot's movement.

5.4 Integration with Gazebo:
• The “gazebo.launch.py” file is a ROS 2 launch file written in Python, responsible for starting the Gazebo simulation with the robot and loading the necessary controllers. I modified this script to ensure that the controllers such as “joint_trajectory_controller” and “joint_state_broadcaster” are loaded and active in Gazebo.

![image](https://github.com/user-attachments/assets/45308c74-66c1-439e-8e2d-dcb4c9705a61)

-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
How to Run the simulation:
In order to run the simulation, use following commands.
 
 1. cd/Ros2_Workspace
 2. ros2 launch robot_control gazebo.launch.py (launches simulation)
 3. ros2 run robot_control motion_control (run the Joint motion node)
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



