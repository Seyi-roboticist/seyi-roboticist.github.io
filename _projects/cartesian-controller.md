---
layout: project
title: "Real-Time Cartesian Controller for UR5/UR5e"
description: "A three-package ROS 2 control framework for real-time end-effector positioning — SVD-based damped pseudo-inverse Jacobian with PID feedback achieving ±0.7mm accuracy at 500Hz on Universal Robots hardware."
date: 2025-10-01
categories: [Manipulation, Controls, ROS2, C++]
featured_image: "/assets/images/projects/cartesian-controller/featured.jpg"
github_url: "https://github.com/Seyi-roboticist/_controller_"
demo_url: "https://www.youtube.com/watch?v=lPNE6-0R59k"

gallery:
  - type: "video"
    file: "https://www.youtube.com/embed/FevBLPXetxo"
    description: "RViz visualization — real-time trajectory tracking with TF frame overlay"
  - type: "video"
    file: "https://www.youtube.com/embed/lPNE6-0R59k"
    description: "Gazebo simulation — Cartesian position control with dynamic target updates"
  - type: "video"
    file: "https://www.youtube.com/embed/UKBMwUgmN18"
    description: "Hardware demonstration on real UR5 robot"

code_files:
  - name: "Cartesian Controller (Core IK Loop)"
    file: "cartesian_controller.cpp"
    language: "cpp"
    download_url: "https://github.com/Seyi-roboticist/_controller_/blob/main/robotics_packages/controllers/cartesian_controller/src/cartesian_controller.cpp"
    content: |
      // SVD-based damped pseudo-inverse Jacobian
      // with Tikhonov regularization for singularity handling
      bool CartesianController::calculateJointVelocities(
          const KDL::Twist &cartesian_error,
          KDL::JntArray &joint_velocities)
      {
          KDL::JntArray joint_positions = getCurrentJointPositions();
          joint_velocities.resize(joint_positions.rows());

          KDL::Jacobian jacobian(joint_positions.rows());
          jac_solver_->JntToJac(joint_positions, jacobian);

          // Extract position-only rows of the Jacobian (3x6)
          Eigen::MatrixXd jac_position =
              jacobian.data.block(0, 0, 3, jacobian.columns());

          // SVD decomposition for numerical stability
          Eigen::JacobiSVD<Eigen::MatrixXd> svd(
              jac_position, Eigen::ComputeThinU | Eigen::ComputeThinV);

          // Damped pseudo-inverse: sigma / (sigma^2 + lambda^2)
          Eigen::MatrixXd s_inv = Eigen::MatrixXd::Zero(
              svd.matrixV().cols(), svd.matrixU().cols());
          Eigen::VectorXd s = svd.singularValues();

          for (Eigen::Index i = 0; i < s.size(); ++i)
              s_inv(i, i) = s(i) / (s(i)*s(i) + damping_factor_*damping_factor_);

          Eigen::MatrixXd J_pinv =
              svd.matrixV() * s_inv * svd.matrixU().transpose();

          Eigen::VectorXd qdot = velocity_scaling_factor_ * J_pinv * vel;
          return true;
      }

  - name: "SE3 Sensor Hardware Interface"
    file: "hardware_interface.cpp"
    language: "cpp"
    download_url: "https://github.com/Seyi-roboticist/_controller_/blob/main/robotics_packages/hardware_interfaces/se3_sensor_driver/src/hardware_interface.cpp"
    content: |
      // ros2_control-compliant sensor plugin
      // TCP/IP socket bridge to TF lookup server
      hardware_interface::return_type SE3SensorHardware::read(
          const rclcpp::Time &, const rclcpp::Duration &)
      {
          for (size_t sensor_idx = 0; sensor_idx < info_.sensors.size(); ++sensor_idx)
          {
              size_t base_idx = sensor_idx * 7;  // [x y z qx qy qz qw]

              // Read serialized PoseStamped over TCP socket
              uint32_t message_size{0};
              ::read(sockfd_, &message_size, sizeof(message_size));

              rclcpp::SerializedMessage msg(message_size);
              // ... buffered read with reconnection logic ...

              rclcpp::Serialization<geometry_msgs::msg::PoseStamped> deser;
              geometry_msgs::msg::PoseStamped pose;
              deser.deserialize_message(&msg, &pose);

              hw_sensor_states_[base_idx + 0] = pose.pose.position.x;
              hw_sensor_states_[base_idx + 1] = pose.pose.position.y;
              hw_sensor_states_[base_idx + 2] = pose.pose.position.z;
              // ... orientation quaternion ...
          }
          return hardware_interface::return_type::OK;
      }

  - name: "TF Lookup Server (Socket Bridge)"
    file: "tf_lookup_server.cpp"
    language: "cpp"
    download_url: "https://github.com/Seyi-roboticist/_controller_/blob/main/robotics_packages/hardware_interfaces/se3_sensor_driver/src/tf_lookup_server.cpp"
    content: |
      // Multithreaded ROS 2 node bridging TF system
      // to hardware interface via TCP/IP sockets
      void TFLookupServer::timerCallback()
      {
          auto robot_transform = tf_buffer_->lookupTransform(
              robot_parent_frame_, robot_sensor_frame_,
              tf2::TimePointZero);

          geometry_msgs::msg::PoseStamped robot_pose;
          robot_pose.pose.position.x = robot_transform.transform.translation.x;
          // ... fill pose from transform ...

          rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serialization;
          rclcpp::SerializedMessage msg;
          serialization.serialize_message(&robot_pose, &msg);

          // Send size-prefixed serialized message over TCP
          uint32_t message_size = msg.get_rcl_serialized_message().buffer_length;
          write(client_sockfd_, &message_size, sizeof(message_size));
          write(client_sockfd_, msg.get_rcl_serialized_message().buffer,
                msg.get_rcl_serialized_message().buffer_length);
      }

components:
  - name: "UR5e Robot Arm"
    quantity: 1
    description: "6-DOF collaborative manipulator — hardware validation platform"
  - name: "UR5 Robot Arm"
    quantity: 1
    description: "6-DOF manipulator — initial development and testing"
  - name: "ROS 2 Humble"
    quantity: 1
    description: "Middleware framework with ros2_control integration"
  - name: "KDL"
    quantity: 1
    description: "Kinematics and Dynamics Library — FK, Jacobian solvers"
  - name: "Eigen3"
    quantity: 1
    description: "Linear algebra — SVD decomposition for pseudo-inverse"
---

## Overview

A professional-grade ROS 2 control framework for real-time Cartesian position control of Universal Robots manipulators. The system uses external SE3 sensor feedback and inverse kinematics to achieve precise end-effector positioning, running a 500Hz control loop with ±0.7mm measured accuracy across the full UR5e workspace.

This isn't a wrapper around MoveIt or an off-the-shelf planner — it's a from-scratch implementation of the complete control pipeline: sensor hardware interface, socket-based data bridge, PID error computation, SVD-based Jacobian inversion, and velocity command generation. Every layer is designed for real-time performance with deterministic timing.

## System Architecture

The system is organized as three ROS 2 packages following a clean separation of concerns:

```
robotics_packages/
├── hardware_interfaces/
│   └── se3_sensor_driver/          # ros2_control sensor plugin
│       ├── hardware_interface.cpp  # TCP socket ↔ ros2_control bridge
│       └── tf_lookup_server.cpp    # TF → serialized pose over TCP
├── controllers/
│   └── cartesian_controller/       # ros2_control controller plugin
│       └── cartesian_controller.cpp  # PID + Jacobian IK
└── applications/
    └── ur5e_cartesian_control/     # Launch, config, URDF/xacro
        ├── ur5e.launch.py          # Unified launch for sim/real
        └── ur5e_controllers.yaml   # Tuned controller parameters
```

The data flow is: **SE3 Sensors** → **TF Lookup Server** (TCP/IP socket bridge) → **SE3 Sensor Hardware Interface** (ros2_control plugin) → **Cartesian Controller** (PID + damped pseudo-inverse Jacobian) → **Joint velocity commands** → **UR5/UR5e robot**.

## Inverse Kinematics: Damped Pseudo-Inverse Jacobian

The core of the controller is a position-only Cartesian IK solver using SVD decomposition of the manipulator Jacobian. The position-only approach extracts the top 3 rows of the 6×n Jacobian matrix, solving only for translational velocity while ignoring orientation — a deliberate design choice that simplifies the controller for tasks where end-effector orientation is unconstrained.

Tikhonov regularization (damped least squares) handles singularities gracefully. Each singular value σᵢ is replaced with σᵢ/(σᵢ² + λ²), where λ is the damping factor. This trades off tracking accuracy for joint velocity smoothness when the manipulability index drops near singular configurations — the arm slows down rather than generating unbounded joint velocities.

## PID Control with Anti-Windup

The Cartesian error computation implements a full PID controller with configurable gains per axis. The integral term includes anti-windup clamping to prevent excessive buildup during sustained errors or when the arm is physically blocked. The derivative term provides damping to reduce overshoot during fast target transitions.

Tuned parameters for the UR5e:
- **Kp**: [2.2, 2.2, 2.2] — proportional gains
- **Ki**: [0.02, 0.02, 0.02] — integral gains  
- **Kd**: [0.5, 0.5, 0.5] — derivative gains
- **λ**: 0.05 — Jacobian damping factor
- **Velocity scaling**: 0.07

## SE3 Sensor Hardware Interface

The `se3_sensor_driver` package implements a ros2_control `SensorInterface` plugin that reads SE3 pose data over TCP sockets. The architecture includes a separate **TF Lookup Server** node — a multithreaded ROS 2 node with dedicated callback groups for timer-driven TF lookups and socket accept/write operations.

The socket protocol uses size-prefixed ROS 2 serialized messages: a 4-byte message length header followed by the CDR-serialized `PoseStamped` payload. The hardware interface handles reconnection with configurable retry attempts and delay, and uses non-blocking sockets to prevent the real-time control loop from stalling on network I/O.

## Deployment Modes

A single unified launch file supports three deployment modes:

- **Real robot**: Direct connection to UR5/UR5e via TCP/IP with the UR ROS 2 driver
- **Gazebo simulation**: Ignition Gazebo with `ign_ros2_control` plugin
- **Fake hardware**: ros2_control mock hardware for unit testing and CI

Dynamic target updates during operation are supported through TF — publish a static transform to `target_sensor_frame` and the controller smoothly tracks the new position.

## Performance

| Metric | Value |
|---|---|
| Control loop rate | 500 Hz |
| Position accuracy | ±0.7 mm |
| Convergence time | < 3 sec (50 cm move) |
| Workspace | Full UR5e envelope |
| Singularity handling | Graceful degradation via Tikhonov damping |

## Links

- [GitHub Repository](https://github.com/Seyi-roboticist/_controller_)
- [Visualization Demo](https://www.youtube.com/watch?v=FevBLPXetxo)
- [Simulation Demo](https://www.youtube.com/watch?v=lPNE6-0R59k)
- [Hardware Demo (UR5)](https://www.youtube.com/shorts/UKBMwUgmN18)
