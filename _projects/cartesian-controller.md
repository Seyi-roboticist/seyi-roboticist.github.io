---
layout: project
title: "Real-Time Cartesian Controller for UR5/UR5e"
description: "A three-package ROS 2 control framework for real-time end-effector positioning. SVD-based damped pseudo-inverse Jacobian with PID feedback achieving ±0.7mm accuracy at 500Hz on Universal Robots hardware."
date: 2025-10-01
categories: [Manipulation, Controls, Xacro, ROS2, C++, Python]
featured_image: "/assets/images/projects/cartesian-controller/featured.jpg"
github_url: "https://github.com/Seyi-roboticist/_controller_"
demo_url: "https://www.youtube.com/watch?v=lPNE6-0R59k"

code_files:
  - name: "Damped Pseudo-Inverse Jacobian IK"
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

  - name: "PID with Anti-Windup"
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

  - name: "SE3 Sensor Hardware Interface"
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

I built a complete real-time Cartesian position controller for Universal Robots arms from scratch. Not a MoveIt wrapper. Not an off-the-shelf planner. The full pipeline: sensor hardware interface, socket-based data bridge, PID error computation, SVD-based Jacobian inversion, and velocity command generation. Every layer is designed for real-time performance with deterministic timing.

The controller runs at 500Hz and achieves ±0.7mm positional accuracy across the full UR5e workspace. I validated it in Gazebo simulation first, then deployed it on real UR5 and UR5e hardware.

## Demos

[![Simulation Demo](https://img.youtube.com/vi/lPNE6-0R59k/maxresdefault.jpg)](https://www.youtube.com/watch?v=lPNE6-0R59k)

**Simulation**: UR5e tracking dynamic Cartesian targets in Gazebo with RViz visualization overlay.

[![Live Hardware Demo](https://img.youtube.com/vi/UKBMwUgmN18/maxresdefault.jpg)](https://www.youtube.com/shorts/UKBMwUgmN18)

**Live Demo**: Real UR5 robot executing Cartesian position commands on hardware.

[![RViz Visualization](https://img.youtube.com/vi/FevBLPXetxo/maxresdefault.jpg)](https://www.youtube.com/watch?v=FevBLPXetxo)

**Visualization**: TF frames, target tracking, and error convergence in real time.

## System Architecture

<script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>
<script>mermaid.initialize({startOnLoad:true, theme:'dark'});</script>

<pre class="mermaid">
flowchart LR
    A["SE3 Sensor"] -->|TF Broadcast| B["TF Lookup Server"]
    B -->|TCP/IP Socket| C["SE3 Hardware Interface"]
    C -->|Pose Data| D["Cartesian Controller"]
    D -->|Joint Velocity Cmds| E["UR5 / UR5e"]

    subgraph Controller["Controller Core"]
        direction TB
        F["PID: Kp + Ki + Kd + Anti-Windup"] --> G["Jacobian IK: SVD + Tikhonov Damping"]
    end
</pre>

Three ROS 2 packages: `se3_sensor_driver` (hardware interface reading pose data over TCP), `cartesian_controller` (PID + Jacobian IK), and `ur5e_cartesian_control` (launch, config, URDF).

## How It Works

The core is a position-only Cartesian IK solver using SVD decomposition of the manipulator Jacobian. I extract the top 3 rows of the 6x6 Jacobian, solving only for translational velocity while ignoring orientation. Tikhonov regularization handles singularities gracefully: each singular value is replaced with sigma_i/(sigma_i^2 + lambda^2). The arm slows down near singularities instead of generating unbounded joint velocities.

The Cartesian error computation uses a full PID controller with anti-windup clamping on the integral term. Tuned gains for the UR5e: Kp=2.2, Ki=0.02, Kd=0.5, damping lambda=0.05, velocity scaling=0.07. A single unified launch file supports real robot, Gazebo simulation, and fake hardware for CI testing.

## Results

| Metric | Value |
|---|---|
| Control loop rate | 500 Hz |
| Position accuracy | ±0.7 mm |
| Convergence time | < 3 sec (50cm moves) |
| Workspace | Full UR5e envelope |
| Singularity behavior | Graceful slowdown via Tikhonov damping |

## Links

[View on GitHub](https://github.com/Seyi-roboticist/_controller_) | [Simulation Demo](https://www.youtube.com/watch?v=lPNE6-0R59k) | [Live Demo](https://www.youtube.com/shorts/UKBMwUgmN18) | [RViz Visualization](https://www.youtube.com/watch?v=FevBLPXetxo)
