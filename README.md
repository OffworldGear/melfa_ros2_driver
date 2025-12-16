I have a CR751D and RV-4FL-D robot. The current official MELFA driver for ROS2 was designed for the CR800. My changes allow you to select the CR750 series controller and the RV-4FL-D robot (the parameters are all copied from the RV-4FRL. I will see what needs changing in the future). My CR-751-D required some different handshaking than the CR800. 

All of the changes were done with Gemeni 3.0 Thinking. 

Mitsubishi RV-4FL-D & CR751-D ROS2 Driver Configuration Guide
This document details the complete setup, code modifications, and custom configurations required to run the Mitsubishi RV-4FL-D robot with the CR751-D controller using ROS2 (Humble) and MoveIt 2.
These changes address:
The Handshake: Fixing the UDP synchronization sequence (sending NULL) to prevent immediate disconnects.
Hardware Mismatch: Creating a custom definition for the RV-4FL-D (Standard Reach) as the driver defaults to RV-4FRL (Long Reach).
Timing Mismatches: Differences between the CR751 (7.11ms cycle) and CR800 (3.5ms cycle).
OS Jitter: Running ROS2 inside WSL over a USB Ethernet adapter.
Safety: Preventing "Excessive Speed" errors caused by network latency.
1. The Handshake Logic (Critical Code Change)
The default driver assumes the robot is ready to receive movement commands immediately. However, the CR751 controller requires a specific "Handshake" sequence to establish the UDP link before it accepts motion data. Without this, the controller rejects the first packet and throws an error.
The Fix: Sending the NULL Packet
We modified the hardware_interface.cpp on_activate() function. Instead of immediately entering the control loop, the driver now performs a "Ping-Pong" synchronization:
Send NULL: The driver sends a packet with cmd_type = MXT_CMD_NULL (0). This tells the robot "I am here, but don't move yet."
Wait for Response: The driver blocks until it receives a valid status packet from the robot.
Sync Position: The driver reads the robot's actual joint angles from this response and sets the ROS command variables to match.
Begin Control: Only then does the driver switch cmd_type to MXT_CMD_MOVE (1).
File: melfa_driver/src/hardware_interface.cpp

C++


// In CallbackReturn MELFAPositionHardwareInterface::on_activate(...)
// ... (After socket creation)

// 1. Send NULL packet to wake up the CR751 UDP listener
rt_exc_->cmd_pack.cmd_type = MXT_CMD_NULL; 
rt_exc_->WriteToRobot_CMD_();

// 2. Wait for the Robot to reply (The Handshake)
RCLCPP_INFO(rclcpp::get_logger("MELFAPositionHardwareInterface"), "Waiting for valid packet to sync position...");

// (Blocking loop to read initial position)
// ... [Code implementation ensures we have valid hw_states_ before proceeding]

RCLCPP_INFO(rclcpp::get_logger("MELFAPositionHardwareInterface"), "Packet received! Syncing internal state...");


2. Custom Robot Definition (RV-4FL-D)
The official driver only supports the RV-4FRL (Long Reach, 649mm). We possess the RV-4FL-D (Standard Reach, 505mm). Using the FRL URDF resulted in IK errors and self-collisions because the arm lengths were incorrect.
We created a new robot description package rv4fld by cloning and modifying the rv4frl files.
New Files Created:
File Type
Path
Purpose
Xacro/URDF
melfa_description/urdf/rv4fld/rv4fld.urdf.xacro
Defines the correct link lengths (DH Parameters) for the 505mm reach.
SRDF
melfa_moveit_config/.../config/rv4fld.srdf
Defines planning groups and collision pairs for the standard reach arm.
Controllers
melfa_description/config/rv4fld_controllers.yaml
Maps the specific rv4fld_joint_1...6 names to the joint_trajectory_controller.
Launch
melfa_bringup/launch/rv4fld_control.launch.py
The main entry point to load the specific RV-4FL-D configuration.

3. Hardware & Network Architecture (disregard for other setups)
The "Dual-Adapter" Strategy (For WSL + VM)
To prevent Windows from causing latency by routing packets between the Host (WSL) and the Guest VM (RT Toolbox3) on a single interface, use a physical Ethernet Switch and USB Passthrough.
Topology:
Robot (IP: 192.168.0.20) $\rightarrow$ Switch
WSL Host Adapter (IP: 192.168.0.10) $\rightarrow$ Switch
VM USB Adapter (IP: 192.168.0.100) $\rightarrow$ Switch (Pass this device through to VirtualBox/VMware exclusively).
Settings:
Disable Energy Efficient Ethernet (EEE) on all adapters in Windows Device Manager.
Set Process Priority for vmmem (WSL) to "High" in Windows Task Manager.
4. Robot Controller Configuration (CR751-D)
Robot Program (MXT Command)
The CR751 uses a 7.11ms cycle. We typically set the MXT filter to 50ms to smooth out jitter from the Windows/WSL network stack.
Code (1.prg):

Basic


' MXT <FileNo>, <Type>, <FilterTimeConstant>
' Filter=50 absorbs OS jitter.
MXT 1, 1, 50


5. ROS2 Driver Code Modifications
These changes are required in the C++ source code of melfa_driver. You must rebuild (colcon build) after applying them.
A. Fix: Connection Dropouts (Packet Loss)
File: melfa_driver/src/melfa_rt_exc.cpp
Function: recv_packet_
Standard drivers have a strict timeout (approx 7ms). If Windows/WSL pauses for background tasks, the driver disconnects. We increased the tolerance.

C++


// Change timeout calculation
sTimeOut.tv_sec = 0;
// Increase multiplier from 2 to 10.
// 10 * period allows the OS to "blink" for ~70ms without killing the connection.
sTimeOut.tv_usec = (long)(10 * period * 1000); 


B. Fix: "Excessive Speed" Safety Clamp
File: melfa_driver/src/hardware_interface.cpp
Function: write
If the PC lags, the next packet might request a position far ahead in time. The CR751 will try to move there instantly, triggering an Excessive Speed Error (L01/E.02). We implement a software clamp.

C++


// Add to top of file
const double SAFE_RAD_LIMIT = 0.015; // ~0.85 degrees per cycle limit

// Inside write() function:
// [Logic checks diff between target and current. If > LIMIT, clamps target to current + LIMIT]


6. ROS2 Controller Configuration (YAML)
File: melfa_description/config/rv4fld_controllers.yaml
A. Sync Update Rate
Set the update rate to 140Hz to match the CR751 hardware cycle (1 / 0.00711s).

YAML


controller_manager:
  ros__parameters:
    update_rate: 140  # Changed from default 250/286


B. Fix: "Jump" at End of Movement
By default, the trajectory controller might disable the control loop slightly before the robot comes to a complete halt, causing a physical "thud" or jump.

YAML


rv4fld_controller:
  ros__parameters:
    # Forces the planner to ensure velocity is 0 before finishing
    allow_nonzero_velocity_at_trajectory_end: false
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0








<img src="./doc/figures/MELFA_t.png" width="400" height="98"> <img src="./doc/figures/ROS-AP-logo.png" width="208" height="98">

# __MITSUBISHI ELECTRIC INDUSTRIAL ROBOT MELFA ROS2 DRIVER__
    
## __1. Overview__

MELFA ROS2 Driver, co-developed with [ROS-Industrial Consortium Asia Pacific](https://rosindustrial.org/ric-apac), provides a suite of tools to enable the creation of advance solutions using our industry proven platform. Mitsubishi Electric provides a ROS2 driver, ROS2 GPIO controllers, robot description files and moveit_config packages for each robot; optimized in-house by our developers to ensure high performance. 

Introducing the next generation of intelligent robots, incorporating advanced solutions technology and “e-F@ctory”, technologies and concepts developed and proven using Mitsubishi Electric’s own production facilities that go beyond basic robotic performance to find ways of reducing the TCO in everything from planning and design through to operation and maintenance. 
</br>

[![ROS2 demo](https://markdown-videos.vercel.app/youtube/RP6lIamz9-8?si=v53JdDJ4vaFBPd5M)](https://youtu.be/RP6lIamz9-8?si=v53JdDJ4vaFBPd5M)
[![MEAU demo](https://markdown-videos.vercel.app/youtube/Ks6ji6kw68c?si=KCWWB8-P_m3ofB4V)](https://youtu.be/Ks6ji6kw68c?si=KCWWB8-P_m3ofB4V)
- [Learn more](https://www.mitsubishielectric.com/fa/products/rbt/robot/index.html)
- [Robot catalog](https://www.mitsubishielectric.com/app/fa/download/search.do?kisyu=/robot&mode=catalog)

</br>

## __2. MELFA ROS2 Driver Feature__

MELFA ROS2 Driver consists of six main components: melfa_bringup, melfa_description, melfa_driver, melfa_io_controllers, melfa_msgs, various moveit_config packages.

### __melfa_bringup__

- provides launch files for robot bringup

### __melfa_description__

- contains robot descriptions
- ros2_controllers

### __melfa_driver__

- supports [ros2_control](https://control.ros.org/humble/doc/getting_started/getting_started.html).
- provides __real time communication__<sup>1</sup> hardware interface with our CR800/860-R/Q/D robot controllers via __rtexc api__ <sup>2</sup> from our [__MELFA ethernet SDK__](https://github.com/Mitsubishi-Electric-Asia/melfa_ethernet_sdk). 
- connects to the robot controller via __rtexc api__ to control the robot via __MELFA BASIC VI__<sup>3</sup> __MXT__<sup>4</sup> command. The robot position command, robot state & I/O data are transmitted through this connection. 
- includes quality of life features built into __rtexc api__ such as user configurable disconnection detection and debugging tools.

### __melfa_io_controllers__

- supports [ros2_control](https://control.ros.org/humble/doc/getting_started/getting_started.html).
- user configurable io controllers.
- provides ROS2 controllers for GPIO control

### __melfa_msgs__

- provides ROS2 msgs for MELFA robots

### __melfa_robot-model_moveit_config__

- provides example MoveIt config and launch files for MELFA robots
- supports OMPL, Pilz Industrial Planner, CHOMP and Moveit servo.
- optimized by our developers to ensure high performance in speed and accuracy.

<table>
<head>
</head>
    <tr>
        <th colspan="1">Tier 1 Supported Robots</th>
        <th colspan="4">Robot Controllers</th>
    </tr>
    <tr>
        <th>Robot Model</th>
        <th>CR800-R</th>
        <th>CR800-Q</th>
        <th>CR800-D</th>
        <th>CR860-D/R/Q</th>
    </tr>
    <tr>
        <td>RH-6FRH5520</td>
        <td>&#9711;</td>
        <td>&#9711;</td>
        <td>&#9711;</td>
        <td>&#10005;</td>
    </tr>
    <tr>
        <td>RH-6CRH6020</td>
        <td>&#10005;</td>
        <td>&#10005;</td>
        <td>&#9711;</td>
        <td>&#10005;</td>
    </tr>
    <tr>
        <td>RV-2FR</td>
        <td>&#9711;</td>
        <td>&#9711;</td>
        <td>&#9711;</td>
        <td>&#10005;</td>
    </tr>
    <tr>
        <td>RV-4FR</td>
        <td>&#9711;</td>
        <td>&#9711;</td>
        <td>&#9711;</td>
        <td>&#10005;</td>
    </tr>
    <tr>
        <td>RV-4FRL</td>
        <td>&#9711;</td>
        <td>&#9711;</td>
        <td>&#9711;</td>
        <td>&#10005;</td>
    </tr>
    <tr>
        <td>RV-5AS</td>
        <td>&#10005;</td>
        <td>&#10005;</td>
        <td>&#9711;</td>
        <td>&#10005;</td>
    </tr>
    <tr>
        <td>RV-7FRL</td>
        <td>&#9711;</td>
        <td>&#9711;</td>
        <td>&#9711;</td>
        <td>&#10005;</td>
    </tr>
    <tr>
        <td>RV-8CRL</td>
        <td>&#10005;</td>
        <td>&#10005;</td>
        <td>&#9711;</td>
        <td>&#10005;</td>
    </tr>
    <tr>
        <td>RV-13FRL</td>
        <td>&#9711;</td>
        <td>&#9711;</td>
        <td>&#9711;</td>
        <td>&#10005;</td>
    </tr>
    <tr>
        <td>RV-80FR</td>
        <td>&#10005;</td>
        <td>&#10005;</td>
        <td>&#10005;</td>
        <td>&#9711;</td>
    </tr>
</table>


&#10146; <sup>1</sup>  __real time communication__ frequency is 286Hz for CR800/860-R & CR-800/860-D and 141Hz for CR800/860-Q.

&#10146; <sup>2</sup>  __rtexc api__ stands for Real Time External Control API.

&#10146; <sup>3</sup>  __MELFA BASIC VI__ is our proprietary robot programming language.

&#10146; <sup>4</sup>  __MXT__ is the command to enable __real time external control__.


>Note1: You can download the [CR750/CR751 Series Controller, CR800 Series Controller Ethernet Function Instruction Manual](https://www.mitsubishielectric.com/fa/download/search.page?mode=manual&kisyu=/robot&q=CR750%2FCR751%20Series%20Controller%2C%20CR800%20Series%20Controller%20Ethernet%20Function%20Instruction%20Manual&sort=0&style=0&lang=2&category1=0&filter_discontinued=0&filter_bundled=0) from [Robot Industrial/Collaborative Robot MELFA Manual](https://www.mitsubishielectric.com/fa/download/search.page?mode=manual&kisyu=/robot).</br>


## __3. MELFA ROS2 Driver Usage and Installation__

MELFA ROS2 Driver is designed to interface CR800 robot controllers with the ROS2 so that developers can leverage the contributions from the Open Source Community with an industry proven robot platform. Please select a guide below to get started.
</br>

- [MELFA ROS2 user guide](./doc/melfa_ros2_driver.md) : Usage and Installation of MELFA ROS2.
- [RT Toolbox3 Setup](./doc/rt_toolbox3_setup.md) : Create your first RT Toolbox3 Project File for ROS2.
- [RT Toolbox3 Simulator Setup](./doc/rt_sim_setup.md) : Connect to RT Toolbox3 simulator as if it is a real robot.
- [RT Toolbox3 Real Robot Setup](./doc/rt_real_setup.md): Connect to a MELFA robot.

  
<div> </div>

## __4. Other MELFA ROS2 Related Repositories__

- [MELFA ROS2 8XS](https://github.com/Mitsubishi-Electric-Asia/melfa_ros2_8xs) : Sample package with MELSERVO integration for 6+2-axis articulated robot and 4+2-axis SCARA robot. Accompanied with RT Toolbox3 Project File to try in RT Toolbox3 simulator.
- [MELFA ROS2 Integrated System Simulators](https://github.com/Mitsubishi-Electric-Asia/melfa_ros2_syssim) : Experience MELSOFT System Simulators for Programmable Logic Controllers and Human Machine Interface touch displays operating together seamlessly with a simple ROS2 program. Includes sample packages with MELSOFT project files. 
- [MELFA ROS2 PLC](https://github.com/Mitsubishi-Electric-Asia/melfa_ros2_plc) : Sample program with simple integration for MELSEC iQ-R Controllers.
- [MELFA ROS2 HMI](https://github.com/Mitsubishi-Electric-Asia/melfa_ros2_hmi) : Sample program with simple integration with GOT-HMI (Human Machine Interface) for iQ-platform robot controllers.

<div> </div>

## __5. MELFA Naming Convention__

This section provides a brief introduction to naming conventions of MELFA robots. Below are images from our [robot catalog](https://www.mitsubishielectric.com/app/fa/download/search.do?kisyu=/robot&mode=catalog) describing the naming convention.

For articulated robots (RV), it is fairly straightforward as the variations that contribute to package differences are __Maximum load capacity__, __Series__ and __Arm length__. 

</br>

<img src="./doc/figures/naming_convention_rv.png" width="1000" heigth="500" >

</br>

For SCARA robots (RH), it has more variations that contribute to packages differences such as __Maximum load capacity__, __Series__, __Arm length__ in cm and __Vertical stroke__ in cm.

</br>

<img src="./doc/figures/naming_convention_rh.png" width="1000" heigth="500" >

</br>

__Environment specifications__, __Internal wiring__ and __Controller type__ do not contribute to kinematic variations. However, it is important to take note of __Controller type__ as it may change the __Control frequency__ and/or __I/O controller__ settings.


## __6. Contact us / Technical support__
More Support & Service, please contact us [@MEAP](https://sg.mitsubishielectric.com/fa/en/contact.html) &#9743;. For contributing and reporting, refer to [this](./CONTRIBUTING.md) for development related enquiries.

<div> </div>


