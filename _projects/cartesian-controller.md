---
layout: project
title: "Real-Time Cartesian Controller for UR5/UR5e"
description: "A three-package ROS 2 control framework for real-time end-effector positioning. SVD-based damped pseudo-inverse Jacobian with PID feedback achieving ±0.7mm accuracy at 500Hz on Universal Robots hardware."
date: 2025-10-01
categories: [Manipulation, Controls, Xacro, ROS2, C++, Python]
featured_image: "/assets/images/projects/cartesian-controller/featured.jpg"
github_url: "https://github.com/Seyi-roboticist/_controller_"
demo_url: "https://www.youtube.com/watch?v=lPNE6-0R59k"

gallery:
  - type: "image"
    file: "/assets/images/projects/cartesian-controller/ur5_realtime_control.gif"
    description: "UR5 robot tracking Cartesian targets in real time at 500Hz"
  - type: "video"
    file: "https://www.youtube.com/embed/FevBLPXetxo"
    description: "RViz visualization of real-time trajectory tracking with TF frame overlay"
  - type: "video"
    file: "https://www.youtube.com/embed/lPNE6-0R59k"
    description: "Gazebo simulation showing Cartesian position control with dynamic targets"
  - type: "video"
    file: "https://www.youtube.com/embed/UKBMwUgmN18"
    description: "Live hardware demonstration on a real UR5 robot"

code_files:
  - name: "Sample Code Snippet: Damped Pseudo-Inverse Jacobian IK"
    file: "cartesian_controller.cpp"
    language: "cpp"
    download_url: "https://github.com/Seyi-roboticist/_controller_/blob/main/robotics_packages/controllers/cartesian_controller/src/cartesian_controller.cpp"
    content: |
      // SVD-based damped pseudo-inverse Jacobian
      // Tikhonov regularization for singularity handling
      bool CartesianController::calculateJointVelocities(
          const KDL::Twist &cartesian_error,
          KDL::JntArray &joint_velocities)
      {
          KDL::JntArray joint_positions = getCurrentJointPositions();
          joint_velocities.resize(joint_positions.rows());

          KDL::Jacobian jacobian(joint_positions.rows());
          jac_solver_->JntToJac(joint_positions, jacobian);

          // Extract position rows of the Jacobian (3x6)
          Eigen::MatrixXd jac_position =
              jacobian.data.block(0, 0, 3, jacobian.columns());

          Eigen::JacobiSVD<Eigen::MatrixXd> svd(
              jac_position, Eigen::ComputeThinU | Eigen::ComputeThinV);

          // Damped pseudo-inverse: sigma_i / (sigma_i^2 + lambda^2)
          Eigen::MatrixXd s_inv = Eigen::MatrixXd::Zero(
              svd.matrixV().cols(), svd.matrixU().cols());
          Eigen::VectorXd s = svd.singularValues();

          for (Eigen::Index i = 0; i < s.size(); ++i)
              s_inv(i, i) = s(i) / (s(i)*s(i)
                  + damping_factor_*damping_factor_);

          Eigen::MatrixXd J_pinv =
              svd.matrixV() * s_inv * svd.matrixU().transpose();

          Eigen::VectorXd qdot =
              velocity_scaling_factor_ * J_pinv * vel;
          return true;
      }

  - name: "Sample Code Snippet: PID with Anti-Windup"
    file: "cartesian_controller.cpp"
    language: "cpp"
    download_url: "https://github.com/Seyi-roboticist/_controller_/blob/main/robotics_packages/controllers/cartesian_controller/src/cartesian_controller.cpp"
    content: |
      // Full PID control law: u = Kp*e + Ki*integral(e) + Kd*de/dt
      KDL::Twist CartesianController::calculateCartesianError(
          const KDL::Frame &current, const KDL::Frame &target,
          const rclcpp::Duration &period)
      {
          KDL::Vector position_error = target.p - current.p;
          double dt = period.seconds();

          // Derivative term
          KDL::Vector error_derivative = KDL::Vector::Zero();
          if (dt > 0.0)
              error_derivative = (position_error - last_error) / dt;
          last_error = position_error;

          // Integral with anti-windup clamping
          error_integral = error_integral + position_error * dt;
          double integral_limit = 1.0;
          for (int i = 0; i < 3; i++) {
              if (std::abs(error_integral(i)) > integral_limit)
                  error_integral(i) = integral_limit
                      * (error_integral(i) > 0 ? 1.0 : -1.0);
          }

          // PID output per axis
          for (int i = 0; i < 3; i++) {
              double control =
                  position_gain_[i] * position_error(i)
                  + ki_[i] * error_integral(i)
                  + kd_[i] * error_derivative(i);
              // ... apply to twist ...
          }
      }

  - name: "Sample Code Snippet: SE3 Sensor Hardware Interface"
    file: "hardware_interface.cpp"
    language: "cpp"
    download_url: "https://github.com/Seyi-roboticist/_controller_/blob/main/robotics_packages/hardware_interfaces/se3_sensor_driver/src/hardware_interface.cpp"
    content: |
      // ros2_control SensorInterface plugin
      // Reads SE3 pose data over TCP sockets
      hardware_interface::return_type SE3SensorHardware::read(
          const rclcpp::Time &, const rclcpp::Duration &)
      {
          if (!connected_ && !connectToServer())
              return hardware_interface::return_type::ERROR;

          for (size_t idx = 0; idx < info_.sensors.size(); ++idx)
          {
              size_t base = idx * 7;  // [x y z qx qy qz qw]

              uint32_t message_size{0};
              ::read(sockfd_, &message_size, sizeof(message_size));

              rclcpp::SerializedMessage msg(message_size);
              // Buffered read with reconnection logic ...

              geometry_msgs::msg::PoseStamped pose;
              deserialization.deserialize_message(&msg, &pose);

              hw_sensor_states_[base + 0] = pose.pose.position.x;
              hw_sensor_states_[base + 1] = pose.pose.position.y;
              hw_sensor_states_[base + 2] = pose.pose.position.z;
              hw_sensor_states_[base + 3] = pose.pose.orientation.x;
              hw_sensor_states_[base + 4] = pose.pose.orientation.y;
              hw_sensor_states_[base + 5] = pose.pose.orientation.z;
              hw_sensor_states_[base + 6] = pose.pose.orientation.w;
          }
          return hardware_interface::return_type::OK;
      }
---

## What This Is

I built a complete real-time Cartesian position controller for Universal Robots arms from scratch. Not a MoveIt wrapper. Not an off-the-shelf planner. The full pipeline: sensor hardware interface, socket-based data bridge, PID error computation, SVD-based Jacobian inversion, and velocity command generation. Every layer is designed for real-time performance with deterministic timing.

The controller runs at 500Hz and achieves ±0.7mm positional accuracy across the full UR5e workspace. I validated it in Gazebo simulation first, then deployed it on real UR5 and UR5e hardware.

<div align="center">
  <img src="/assets/images/projects/cartesian-controller/ur5_realtime_control.gif" alt="UR5 robot tracking Cartesian targets in real time" width="720">
  <p><em>UR5 tracking dynamic Cartesian position targets using the controller at 500Hz.</em></p>
</div>

## Watch It Move

[![Simulation Demo](https://img.youtube.com/vi/lPNE6-0R59k/maxresdefault.jpg)](https://www.youtube.com/watch?v=lPNE6-0R59k)
**Simulation Demo**: UR5e tracking dynamic Cartesian targets in Gazebo with RViz visualization overlay.

[![Live Hardware Demo](https://img.youtube.com/vi/UKBMwUgmN18/maxresdefault.jpg)](https://www.youtube.com/shorts/UKBMwUgmN18)
**Live Demo**: Real UR5 robot executing Cartesian position commands with the controller running on hardware.

[![RViz Visualization](https://img.youtube.com/vi/FevBLPXetxo/maxresdefault.jpg)](https://www.youtube.com/watch?v=FevBLPXetxo)
**Visualization Demo**: TF frames, target tracking, and error convergence visualized in real time.

## System Architecture

<div style="width:100%; overflow-x:auto; margin: 2rem 0;">

```
 ┌─────────────────────────────────────────────────────────────────────┐
 │                        SYSTEM OVERVIEW                             │
 └─────────────────────────────────────────────────────────────────────┘

 ┌──────────────┐      ┌───────────────────┐      ┌─────────────────┐
 │              │      │                   │      │                 │
 │  SE3 Sensor  │─────▶│  TF Lookup Server │─────▶│  SE3 Hardware   │
 │  (External)  │ TF   │  (ROS 2 Node)     │ TCP  │  Interface      │
 │              │      │                   │ /IP  │  (ros2_control) │
 └──────────────┘      └───────────────────┘      └────────┬────────┘
                                                           │
                        ┌──────────────────────────────────┘
                        │  SE3 Pose Data [x y z qx qy qz qw]
                        ▼
              ┌───────────────────────┐
              │                       │
              │  Cartesian Controller │
              │  (ros2_control)       │
              │                       │
              │  ┌─────────────────┐  │
              │  │ PID Controller  │  │
              │  │ Kp, Ki, Kd      │  │
              │  │ + Anti-Windup   │  │
              │  └────────┬────────┘  │
              │           │           │
              │  ┌────────▼────────┐  │
              │  │ Jacobian IK     │  │
              │  │ SVD + Tikhonov  │  │
              │  │ Damped Pseudo-  │  │
              │  │ Inverse         │  │
              │  └────────┬────────┘  │
              │           │           │
              └───────────┼───────────┘
                          │  Joint Velocity Commands
                          ▼
              ┌───────────────────────┐
              │                       │
              │   UR5 / UR5e Robot    │
              │   (Real or Gazebo)    │
              │                       │
              └───────────────────────┘
```

</div>

The system is organized into three ROS 2 packages:

```
robotics_packages/
├── hardware_interfaces/
│   └── se3_sensor_driver/            # ros2_control sensor plugin
│       ├── hardware_interface.cpp    # TCP socket <-> ros2_control bridge
│       ├── tf_lookup_server.cpp      # TF to serialized pose over TCP
│       └── urdf/se3_sensor.xacro     # Parameterized sensor URDF macro
├── controllers/
│   └── cartesian_controller/         # ros2_control controller plugin
│       ├── cartesian_controller.cpp  # PID + Jacobian IK
│       └── cartesian_controller.hpp  # Full class declaration
└── applications/
    └── ur5e_cartesian_control/       # Launch, config, URDF
        ├── ur5e.launch.py            # Unified launch (sim + real)
        ├── ur5e_controllers.yaml     # Tuned PID and IK parameters
        └── urdf/ur5e_with_sensors.xacro  # Robot + sensor integration
```

## How the Inverse Kinematics Works

The core of the controller is a position-only Cartesian IK solver using SVD decomposition of the manipulator Jacobian. I extract the top 3 rows of the 6×n Jacobian matrix, solving only for translational velocity while ignoring orientation. This is a deliberate design choice for tasks where end-effector orientation is unconstrained.

Tikhonov regularization (damped least squares) handles singularities gracefully. Each singular value σᵢ is replaced with σᵢ/(σᵢ² + λ²), where λ is the damping factor. This trades tracking accuracy for joint velocity smoothness when the manipulability index drops near singular configurations. In practice, the arm slows down instead of generating unbounded joint velocities.

## PID Controller Design

The Cartesian error computation uses a full PID controller with configurable gains per axis. I added anti-windup clamping on the integral term to prevent excessive buildup during sustained errors or when the arm is physically blocked. The derivative term provides damping to reduce overshoot during fast target transitions.

The gains I tuned for the UR5e after extensive testing:

| Parameter | Value | Purpose |
|---|---|---|
| Kp | [2.2, 2.2, 2.2] | Proportional gains |
| Ki | [0.02, 0.02, 0.02] | Integral gains |
| Kd | [0.5, 0.5, 0.5] | Derivative gains |
| λ | 0.05 | Jacobian damping factor |
| Velocity scaling | 0.07 | Motion speed limit |

## SE3 Sensor Hardware Interface

The `se3_sensor_driver` package implements a ros2_control `SensorInterface` plugin that reads SE3 pose data over TCP sockets. The architecture includes a separate TF Lookup Server node, which is a multithreaded ROS 2 node with dedicated callback groups for timer-driven TF lookups and socket accept/write operations.

The socket protocol uses size-prefixed ROS 2 serialized messages: a 4-byte message length header followed by the CDR-serialized `PoseStamped` payload. The hardware interface handles reconnection with configurable retry attempts and delay, and uses non-blocking sockets to prevent the real-time control loop from stalling on network I/O.

## Deployment Modes

A single unified launch file (`ur5e.launch.py`) supports three deployment modes:

| Mode | Command | Use Case |
|---|---|---|
| Real Robot | `use_fake:=false robot_ip:=<IP>` | Production deployment on UR5/UR5e |
| Gazebo Sim | `use_fake:=true use_gazebo:=true` | Physics simulation and testing |
| Fake Hardware | `use_fake:=true` | Unit testing and CI pipelines |

Dynamic target updates during operation work through TF. You publish a static transform to `target_sensor_frame` and the controller smoothly tracks the new position in real time.

## Performance Results

| Metric | Measured Value |
|---|---|
| Control loop rate | 500 Hz |
| Position accuracy | ±0.7 mm |
| Convergence time | < 3 seconds for 50cm movements |
| Workspace | Full UR5e operational envelope |
| Singularity behavior | Graceful slowdown via Tikhonov damping |

## Links

**View on GitHub**: [https://github.com/Seyi-roboticist/_controller_](https://github.com/Seyi-roboticist/_controller_)

**Simulation Demo**: [https://www.youtube.com/watch?v=lPNE6-0R59k](https://www.youtube.com/watch?v=lPNE6-0R59k)

**Live Demo**: [https://www.youtube.com/shorts/UKBMwUgmN18](https://www.youtube.com/shorts/UKBMwUgmN18)

**Visualization Demo**: [https://www.youtube.com/watch?v=FevBLPXetxo](https://www.youtube.com/watch?v=FevBLPXetxo)
