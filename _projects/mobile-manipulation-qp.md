---
layout: project
title: "Mobile Manipulation with Quadratic Programming"
description: "A full-stack ROS 2 mobile manipulation system combining the Clearpath Husky A200 base, Universal Robots UR5e arm, and Robotiq 2F-85 gripper — unified through QP-based whole-body motion planning, modular xacro architecture, and seamless sim-to-real transfer via ros2_control."
status: ongoing
date: 2025-02-01
categories: [Mobile Robotics, Manipulation, Motion Planning, ROS2, C++, Python]
featured_image: "/assets/images/projects/mobile-manipulation-qp/featured.png"
github_url: "https://github.com/Seyi-roboticist/mobile_manipulation_qp"

code_files:
  - name: "Composable URDF — Top-Level Bringup"
    file: "mobile_manipulation.urdf.xacro"
    language: "xml"
    download_url: "https://github.com/Seyi-roboticist/mobile_manipulation_qp"
    content: |
      <robot name="mobile_manipulation_robot"
             xmlns:xacro="http://www.ros.org/wiki/xacro">

        <!-- Include subsystem descriptions -->
        <xacro:include filename="$(find husky_description)/
          urdf/husky.urdf.xacro"/>
        <xacro:include filename="$(find arm_description)/
          urdf/ur5e.urdf.xacro"/>
        <xacro:include filename="$(find gripper_description)/
          urdf/gripper.urdf.xacro"/>

        <!-- Mount arm to mobile base top plate -->
        <joint name="arm_mount_joint" type="fixed">
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <parent link="top_plate_link"/>
          <child  link="arm_world"/>
        </joint>

        <!-- Attach gripper to arm tool flange -->
        <joint name="gripper_mount_joint" type="fixed">
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <parent link="tool0"/>
          <child  link="gripper_base_link"/>
        </joint>

      </robot>
---

This project builds a complete mobile manipulation system from the ground up in ROS 2 Humble. The robot combines three subsystems — a Clearpath Husky A200 mobile base for locomotion, a Universal Robots UR5e 6-DOF arm for manipulation, and a Robotiq 2F-85 adaptive gripper for grasping — into a single coordinated platform controlled through quadratic programming.

The core challenge is whole-body motion planning: simultaneously commanding the base velocity and arm joint trajectories so the end-effector reaches its target while the base positions itself optimally, all without collisions. I formulate this as a QP problem that balances task-space tracking against joint limits, velocity bounds, and obstacle constraints in real time.

## System Architecture

<script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>
<script>mermaid.initialize({startOnLoad:true, theme:'dark'});</script>

<pre class="mermaid">
flowchart LR
    subgraph Perception["Perception"]
        A["IMU / GPS"]
        B["Encoders"]
        C["F/T Sensor"]
    end

    subgraph Planning["QP Motion Planner"]
        D["Task-Space Objective"]
        E["Joint & Velocity Constraints"]
        F["Collision Avoidance"]
    end

    subgraph Control["ros2_control"]
        G["Joint Trajectory Controller"]
        H["Diff Drive Controller"]
    end

    subgraph Hardware["Hardware Interface"]
        I["UR5e Driver"]
        J["Husky Driver"]
        K["Robotiq Driver"]
    end

    A --> D
    B --> D
    C --> D
    D --> G
    E --> G
    F --> H
    G --> I
    G --> K
    H --> J
</pre>

## Hardware Platform

| Component | Model | Role |
|-----------|-------|------|
| Mobile Base | Clearpath Husky A200 | 4-wheel differential drive, all-terrain UGV |
| Manipulator | Universal Robots UR5e | 6-DOF collaborative arm, force/torque sensing |
| Gripper | Robotiq 2F-85 | Adaptive parallel-jaw, 85mm stroke |
| Compute | Onboard PC | ROS 2 Humble, Ubuntu 22.04 |

## How It Works

### Whole-Body QP Formulation

The system treats the mobile base and arm as a single kinematic chain. At each control cycle, the QP solver takes the desired end-effector pose and computes optimal joint velocities for all actuated joints (base wheels + arm joints) simultaneously:

$$\min_{\dot{q}} \; \| J(q)\dot{q} - \dot{x}_{\text{des}} \|^2 + \lambda \|\dot{q}\|^2$$

subject to joint position limits, velocity bounds, and collision constraints. The regularization term $\lambda$ prevents excessive joint velocities near singularities — similar to the damped least-squares approach I use in my Cartesian controller, but extended here to coordinate the base and arm together.

### Modular URDF Architecture

Each subsystem — base, arm, gripper — is defined as an independent xacro module. The top-level bringup xacro composes them via fixed joints: the UR5e mounts to the Husky's `top_plate_link`, and the Robotiq attaches to the arm's `tool0` flange. This produces a single unified URDF tree with consistent TF frames across simulation and real hardware.

### Sim-to-Real via ros2_control

The system uses `ros2_control` hardware abstractions so the same controller code runs in Gazebo Ignition simulation and on the physical robot. A launch argument (`use_sim:=true/false`) switches between the Gazebo hardware interface and real drivers. The Gazebo simulation includes accurate physics for the Husky drivetrain, UR5e dynamics, and gripper contact forces.

## Key Capabilities

**Coordinated Base-Arm Planning** — The QP planner simultaneously commands base velocity and arm joint trajectories, enabling the robot to navigate and manipulate in a single coordinated motion rather than stop-move-reach sequences.

**Modular Reconfigurability** — Swapping the arm model (UR5 vs UR5e), gripper, or sensor payload requires changing only the relevant xacro include and mount joint. The rest of the system adapts automatically.

**Seamless Simulation** — Full Gazebo Ignition environment with physics-accurate models for all three subsystems. Identical ROS 2 topics, services, and actions between sim and real hardware.

**Dev Container Workflow** — Reproducible development environment using Docker with ROS 2 Humble, pre-configured VS Code extensions, X11 forwarding for RViz/Gazebo, and GPU passthrough for simulation performance.

## Tech Stack

ROS 2 Humble · Gazebo Ignition · ros2_control · colcon · xacro / URDF · vcstool · Docker · C++17 · Python 3 · CMake · ament · QP Solver
