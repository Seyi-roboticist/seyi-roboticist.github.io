---
layout: project
title: "Aurelia X4 UAV: Full-Stack Drone Autonomy"
description: "Led a team of five to build a complete ROS 2 control system for the Aurelia X4 heavy-lift quadcopter. Full sim-to-real pipeline spanning URDF modeling, Gazebo Fortress simulation, ArduPilot SITL, MAVROS flight control, and ArUco-based precision landing with a Sony RX0 II camera. FAA-certified Remote PIC. First team to deliver a working implementation after multiple prior cohorts failed."
date: 2025-05-01
categories: [UAV, ROS2, Autonomy, MAVROS, Computer Vision, ArduPilot]
featured_image: "/assets/images/projects/aurelia/aurelia-x4-hardware.png"
github_url: "https://github.com/Seyi-roboticist/drone_original"

code_files:
  - name: "MAVROS Takeoff Service"
    file: "mavros_takeoff.py"
    language: "python"
    content: |
      #!/usr/bin/env python3
      # Seyi R. Afolayan - Aurelia X4 Flight Services
      # Arms vehicle, switches to GUIDED, commands takeoff to
      # specified altitude via MAVROS service calls

      import rclpy
      from rclpy.node import Node
      from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

      class TakeoffService(Node):
          def __init__(self):
              super().__init__('takeoff_service')
              self.declare_parameter('alt', 10.0)
              self.alt = self.get_parameter('alt') \
                         .get_parameter_value().double_value

              # Service clients
              self.arm_client = self.create_client(
                  CommandBool, '/mavros/cmd/arming')
              self.mode_client = self.create_client(
                  SetMode, '/mavros/set_mode')
              self.takeoff_client = self.create_client(
                  CommandTOL, '/mavros/cmd/takeoff')

          async def execute(self):
              # 1. Switch to GUIDED mode
              mode_req = SetMode.Request()
              mode_req.custom_mode = 'GUIDED'
              await self.mode_client.call_async(mode_req)
              self.get_logger().info('Mode: GUIDED')

              # 2. Arm motors
              arm_req = CommandBool.Request()
              arm_req.value = True
              await self.arm_client.call_async(arm_req)
              self.get_logger().info('Motors armed')

              # 3. Takeoff to target altitude
              takeoff_req = CommandTOL.Request()
              takeoff_req.altitude = self.alt
              await self.takeoff_client.call_async(takeoff_req)
              self.get_logger().info(
                  f'Takeoff to {self.alt}m commanded')

  - name: "ArUco Precision Landing Detector"
    file: "aruco_detector.py"
    language: "python"
    content: |
      #!/usr/bin/env python3
      # ArUco marker detection and 6DOF pose estimation
      # for vision-based precision landing

      import cv2
      import numpy as np
      import rclpy
      from rclpy.node import Node
      from sensor_msgs.msg import Image, CameraInfo
      from geometry_msgs.msg import PoseStamped
      from cv_bridge import CvBridge

      class ArucoDetector(Node):
          def __init__(self):
              super().__init__('aruco_detector')
              self.bridge = CvBridge()
              self.aruco_dict = cv2.aruco.getPredefinedDictionary(
                  cv2.aruco.DICT_4X4_50)
              self.parameters = cv2.aruco.DetectorParameters()
              self.detector = cv2.aruco.ArucoDetector(
                  self.aruco_dict, self.parameters)

              # Load camera intrinsics from calibration
              self.camera_matrix = None
              self.dist_coeffs = None

              self.create_subscription(
                  Image, '/camera/image_raw',
                  self.image_callback, 10)
              self.create_subscription(
                  CameraInfo, '/camera/camera_info',
                  self.caminfo_callback, 10)
              self.pose_pub = self.create_publisher(
                  PoseStamped, '/aruco/pose', 10)

          def image_callback(self, msg):
              frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
              corners, ids, _ = self.detector.detectMarkers(
                  frame)

              if ids is not None and self.camera_matrix \
                                                is not None:
                  rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                      corners, 0.15,  # marker size in meters
                      self.camera_matrix, self.dist_coeffs)

                  # Publish 6DOF pose of first detected marker
                  pose_msg = PoseStamped()
                  pose_msg.header = msg.header
                  pose_msg.pose.position.x = tvecs[0][0][0]
                  pose_msg.pose.position.y = tvecs[0][0][1]
                  pose_msg.pose.position.z = tvecs[0][0][2]
                  self.pose_pub.publish(pose_msg)

  - name: "Gazebo SITL Launch Configuration"
    file: "x4_startup.launch.py"
    language: "python"
    content: |
      # Unified launch file for sim and real hardware
      # Brings up Gazebo, ArduPilot SITL, MAVROS, and
      # optionally RViz in a single command

      from launch import LaunchDescription
      from launch.actions import (DeclareLaunchArgument,
          IncludeLaunchDescription, ExecuteProcess)
      from launch.conditions import IfCondition
      from launch.substitutions import LaunchConfiguration
      from launch_ros.actions import Node

      def generate_launch_description():
          use_sim = LaunchConfiguration('use_sim')
          use_real = LaunchConfiguration('use_real')

          return LaunchDescription([
              DeclareLaunchArgument(
                  'use_sim', default_value='true'),
              DeclareLaunchArgument(
                  'use_real', default_value='false'),

              # Gazebo Fortress with Aurelia X4 model
              IncludeLaunchDescription(
                  'x4_gazebo/x4_world.launch.py',
                  condition=IfCondition(use_sim)),

              # ArduPilot SITL (sim only)
              ExecuteProcess(
                  cmd=['sim_vehicle.py', '-v', 'copter',
                       '--model', 'gazebo-iris',
                       '--console'],
                  condition=IfCondition(use_sim)),

              # MAVROS bridge (both sim and real)
              Node(
                  package='mavros',
                  executable='mavros_node',
                  parameters=[{
                      'fcu_url': 'udp://:14550@',
                      'gcs_url': '',
                      'system_id': 1,
                      'component_id': 1,
                  }]),
          ])
---

## Overview

This was my capstone project for *Robot System Programming* at Johns Hopkins, and it's the one I'm proudest of from grad school. I served as team lead, FAA-certified Remote Pilot-in-Command, and principal software architect for a team of five building a complete ROS 2 autonomy stack for the Aurelia X4, a professional heavy-lift quadcopter with a Pixhawk flight controller running ArduPilot.

What made this project meaningful is the context: multiple previous student cohorts had attempted this same platform and failed to get a working system. The drone had hardware issues from prior semesters, the software stack was fragmented, and nobody had managed to close the loop from simulation to real flight. On top of that, the Aurelia X4 had zero existing ROS or ROS 2 support. No URDF, no Gazebo plugins, no community packages, nothing. Every other popular drone platform (Iris, Crazyflie, Tello) has ready-made ROS integrations you can pull off the shelf. We had to build ours from scratch.

That meant starting from the mechanical specifications and working through the physics ourselves. We measured the airframe dimensions, weighed individual components, and computed inertia tensors for each link to build an accurate URDF/xacro model. Getting the Gazebo simulation to match the real drone's behavior required tuning motor thrust curves, drag coefficients, and mass distributions so the simulated flight controller would produce responses that transferred to the physical aircraft. If the sim model is wrong, the ArduPilot SITL tunes mean nothing on real hardware, so we treated the physics modeling as a first-class engineering task rather than an afterthought.

Our team diagnosed the hardware problems, rebuilt the software architecture from scratch in ROS 2 Humble, and delivered the first stable implementation. We flew the physical drone successfully, something no team before us had accomplished on this platform.

The system spans seven ROS 2 packages covering URDF modeling, Gazebo Fortress simulation, ArduPilot SITL integration, MAVROS flight control, ArUco-based precision landing with a Sony RX0 II camera, and a unified launch system that runs the same codebase in simulation and on real hardware.

## Demo

![Aurelia X4 Physical Hardware](/assets/images/projects/aurelia/aurelia-x4-hardware.png)

*The Aurelia X4 with arms folded during a bench test session at JHU. Visible: CubePilot flight controller, GPS mast, XT60 power connectors, and carbon fiber frame.*

![RViz URDF Model + MAVROS Arming](/assets/images/projects/aurelia/aurelia-x4-rviz-mavros.png)

*Left: RViz visualization showing the X4 URDF model with TF frames for all four propellers, camera, IMU, and drone body. Right: MAVROS terminal output showing the full arming sequence, including GUIDED mode switch, motor arm confirmation, and ROS 2 service calls.*

## System Architecture

<script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>
<script>mermaid.initialize({startOnLoad:true, theme:'dark'});</script>

<pre class="mermaid">
flowchart TB
    subgraph Launch["x4_bringup"]
        A["x4_startup.launch.py"]
    end

    subgraph Sim["Simulation Stack"]
        B["Gazebo Fortress"]
        C["ArduPilot SITL"]
    end

    subgraph Hardware["Real Hardware"]
        D["Pixhawk FC"]
        E["Aurelia X4"]
    end

    subgraph ROS2["ROS 2 Humble"]
        F["MAVROS Bridge"]
        G["x4_service_scripts"]
        H["x4_vision"]
        I["x4_description"]
    end

    A -->|"use_sim:=true"| B
    A -->|"use_real:=true"| D
    B <-->|"MAVLink"| C
    D <-->|"MAVLink"| E
    C <-->|"MAVLink"| F
    D <-->|"MAVLink"| F
    F --> G
    G -->|"Arm / Takeoff / Waypoints"| F
    H -->|"ArUco Pose"| G
    I -->|"URDF + Meshes"| B
</pre>

## Why This Project Matters

### The History

The Aurelia X4 had been assigned to student teams in prior semesters. Despite significant effort, none of them managed to deliver a working system. The hardware had accumulated damage from previous attempts, the software was a patchwork of incomplete packages, and there was no clear path from simulation to real flight. When our team inherited the project, we essentially started from a broken drone and an empty ROS 2 workspace.

### What We Did Differently

Rather than trying to patch what existed, we made the decision to rebuild from the ground up. I structured the project around seven cleanly separated ROS 2 packages, each with a single responsibility:

**x4_description** contains the URDF/xacro model we built from scratch by working through the Aurelia X4's mechanical specifications. Since no existing model existed anywhere, we measured the physical airframe, weighed each structural component and actuator, and derived the inertia matrices by hand. The model includes mesh files for visualization, collision geometries for Gazebo's physics engine, and sensor frames for the GPS, IMU, and camera. This package is shared between simulation and real hardware, so everything that follows is built on the same geometric truth.

**x4_gazebo** handles the Ignition Gazebo Fortress simulation environment, including world files, plugin configurations, and simulation-specific launch files. We used the ArduPilot Gazebo plugin to connect the simulated airframe to ArduPilot's SITL (Software in the Loop), which means the flight controller running in simulation is identical to the one running on the physical Pixhawk.

**x4_bringup** is the entry point. A single launch file brings up the entire system with a `use_sim` or `use_real` flag, which means the same ROS 2 graph runs in both contexts. This sim-to-real parity was a deliberate design choice, and it's what let us iterate quickly in Gazebo before committing to real flights.

**x4_service_scripts** wraps common flight operations (takeoff, landing, waypoint navigation, loiter) into simple ROS 2 service calls through MAVROS. Instead of manually issuing MAVLink commands, a teammate or test script can just run `ros2 run x4_service_scripts mavros_takeoff alt:=10.0`.

**x4_vision** streams video from the Sony RX0 II camera, detects ArUco markers using OpenCV, and publishes 6DOF pose estimates for downstream precision landing logic. The pipeline loads camera intrinsics from calibration, runs marker detection in real time, and estimates the marker's position and orientation relative to the drone.

**x4_interfaces** defines custom ROS 2 message and service types for communication between packages.

**x4_moveit** provides a high-level MoveIt 2 integration for flight planning, using a floating virtual joint to represent the drone's 6DOF freedom in the workspace.

## Flight Control via MAVROS

We chose MAVROS over ArduPilot's native DDS interface (Micro-XRCE-DDS) because at the time of development, DDS support in ArduPilot Copter 4.5 was limited to only mode switching and arming. MAVROS gave us access to the full MAVLink command set, including takeoff, landing, waypoint navigation, and velocity commands, all exposed as standard ROS 2 services and topics.

The flight control pipeline works as follows. A service script sends a `SetMode` request to switch the vehicle to GUIDED mode. Then an arming command enables the motors. From there, the script can issue a `CommandTOL` (takeoff/land) or publish `SET_POSITION_TARGET_LOCAL_NED` messages for waypoint following. Each step includes response validation so the system confirms the vehicle has actually transitioned before moving to the next command.

For real hardware flights, the only change is the MAVROS `fcu_url` parameter, which switches from the SITL UDP endpoint to a serial connection to the physical Pixhawk. Everything else in the ROS 2 graph remains identical.

## Vision-Based Precision Landing

The `x4_vision` package implements an ArUco marker detection pipeline for precision landing. The Sony RX0 II camera is mounted on the drone's underside and streams frames over USB as a UVC device. The detection pipeline runs in three stages:

First, the raw frame is undistorted using camera intrinsics from a calibration file. Then OpenCV's ArUco detector identifies markers from the `DICT_4X4_50` dictionary in the corrected image. Finally, for each detected marker, `estimatePoseSingleMarkers()` computes the marker's 6DOF pose (position and orientation) relative to the camera frame using the known marker size (15 cm) and the calibrated intrinsics.

The resulting pose is published as a `PoseStamped` message on `/aruco/pose`, which downstream nodes can use to command lateral corrections during the landing approach. The marker's translation vector $\mathbf{t} = [t_x,\; t_y,\; t_z]^T$ gives the 3D offset from the camera to the marker center, and the rotation vector $\mathbf{r}$ (Rodrigues form) encodes the marker's orientation. For landing, the key signals are $t_x$ and $t_y$ (lateral error to correct) and $t_z$ (altitude above the marker).

## The Physical Flights

Getting from simulation to real flight involved more than just changing a launch parameter. I had to diagnose and repair hardware damage from previous semesters, recalibrate the Pixhawk's sensors (accelerometer, compass, GPS), verify motor spin directions, and perform multiple ground tests before we were cleared for flight. As the FAA-certified Remote Pilot-in-Command (Part 107), I was responsible for all flight safety decisions, airspace compliance, and go/no-go calls.

We successfully demonstrated takeoff, stable hover, waypoint navigation, and landing on the physical Aurelia X4. The transition from simulation was smooth because the ArduPilot SITL and real Pixhawk run the same firmware, and our ROS 2 stack was designed from the start to be hardware-agnostic.

## Results

| Milestone | Status |
|---|---|
| First team to achieve stable flight on this platform | Achieved |
| Built complete ROS 2 stack from zero (no prior support) | Achieved |
| URDF model derived from mechanical specs and physical measurements | Achieved |
| Sim-to-real with same codebase | Achieved |
| MAVROS takeoff, hover, waypoint, land | Achieved (sim + real) |
| ArUco detection and 6DOF pose estimation | Achieved |
| Gazebo Fortress + ArduPilot SITL integration | Achieved |
| FAA Part 107 compliance for all flights | Maintained |

## Team

I led this project and served as Remote PIC for all flights. The team included Sameer Khan, James Kaluna, Xinhao Chen, and Lavinia Kong. Our faculty advisor was Professor Simon Leonard at LCSR.

## Links

- [GitHub Repository](https://github.com/Seyi-roboticist/drone_original)
