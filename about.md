---
layout: default
title: About
permalink: /about/
---

<link rel="stylesheet" href="https://cdn.jsdelivr.net/gh/devicons/devicon@v2.15.1/devicon.min.css">

<div class="about-hero">
  <div class="container">
    <h1>About Me</h1>
    <p class="about-intro">Full-stack roboticist with depth in control systems, perception, and autonomous decision-making. I build robots that manipulate, navigate, and fly — from the math on the whiteboard to the machine on the floor.</p>
    <p class="about-intro">My work lives at the intersection of precision motion control, computer vision, and real-time autonomy. Whether it's a robot arm hitting sub-millimeter accuracy at 500Hz, an autonomous UAV navigating waypoints, or a mobile robot playing hockey — I care about systems that perform under real-world constraints.</p>
  </div>
</div>

<div class="container">

<h2>What I Build</h2>

<div class="skills-grid">
  <div class="skill-category">
    <h3>🦾 Robotic Manipulation</h3>
    <p>This is where I started and what I keep coming back to. Real-time Cartesian control on UR5/UR5e platforms — singularity-robust inverse kinematics, 500Hz servo loops, and sub-millimeter tracking. Currently designing a custom robot arm from scratch: mechanical design, electronics, and full ROS 2 integration.</p>
    <div class="skill-tags">
      <span class="skill-tag">Jacobian IK (SVD + Tikhonov)</span>
      <span class="skill-tag">500Hz Control Loops</span>
      <span class="skill-tag">±0.7mm Accuracy</span>
      <span class="skill-tag">ros2_control</span>
      <span class="skill-tag">UR5/UR5e</span>
      <span class="skill-tag">Custom Arm Design</span>
    </div>
  </div>

  <div class="skill-category">
    <h3>🛩️ Flight Controls & GNC</h3>
    <p>Full autopilot design from first principles — 6DOF modeling, trim analysis, linearization, and successive loop closure for both fixed-wing and rotary-wing UAVs. Dual EKF state estimation with GPS smoothing. Dual FAA and Transport Canada certified remote pilot.</p>
    <div class="skill-tags">
      <span class="skill-tag">6DOF Modeling</span>
      <span class="skill-tag">Successive Loop Closure</span>
      <span class="skill-tag">PID/PIR</span>
      <span class="skill-tag">Stability Margins</span>
      <span class="skill-tag">EKF Sensor Fusion</span>
      <span class="skill-tag">FAA Part 107</span>
    </div>
  </div>

  <div class="skill-category">
    <h3>🚗 Mobile Robotics</h3>
    <p>Autonomous mobile robots that navigate, plan, and interact. Built JHockey — an autonomous mobile robot for a robotics hockey competition at Hopkins. Whole-body coordination for mobile manipulation using quadratic programming.</p>
    <div class="skill-tags">
      <span class="skill-tag">Autonomous Navigation</span>
      <span class="skill-tag">Motion Planning</span>
      <span class="skill-tag">Mobile Manipulation</span>
      <span class="skill-tag">QP Optimization</span>
      <span class="skill-tag">JHockey</span>
    </div>
  </div>

  <div class="skill-category">
    <h3>👁️ Perception & Machine Learning</h3>
    <p>Computer vision, deep learning, and 3D reconstruction. Implemented Neural Radiance Fields (NeRF) for novel view synthesis. Experience with machine perception pipelines, sim-to-real transfer, and vision-based autonomy.</p>
    <div class="skill-tags">
      <span class="skill-tag">Computer Vision (OpenCV)</span>
      <span class="skill-tag">Deep Learning</span>
      <span class="skill-tag">NeRF</span>
      <span class="skill-tag">Machine Perception</span>
      <span class="skill-tag">Sim-to-Real Transfer</span>
    </div>
  </div>

  <div class="skill-category">
    <h3>📡 State Estimation & Sensor Fusion</h3>
    <p>Turning noisy sensor data into reliable state estimates. EKF design for attitude estimation, GPS smoothing with Gauss-Markov error models, and multi-sensor fusion for navigation in GPS-denied environments including underwater.</p>
    <div class="skill-tags">
      <span class="skill-tag">Extended Kalman Filters</span>
      <span class="skill-tag">GPS Smoothing</span>
      <span class="skill-tag">IMU/Accel/Mag Fusion</span>
      <span class="skill-tag">GPS-Denied Navigation</span>
    </div>
  </div>

  <div class="skill-category">
    <h3>🏭 Industrial Automation</h3>
    <p>Safety-critical control systems for 24/7 production environments. PLC programming, fault-tolerant interlocks, motor control, and deterministic sequencing for manufacturing lines across roofing, telecom, and datacenter industries.</p>
    <div class="skill-tags">
      <span class="skill-tag">Allen-Bradley PLCs</span>
      <span class="skill-tag">Safety Interlocks</span>
      <span class="skill-tag">VFD Motor Control</span>
      <span class="skill-tag">Continuous Production</span>
    </div>
  </div>
</div>

<hr>

<h2>Languages & Tools</h2>

<div style="display:flex; flex-wrap:wrap; justify-content:center; gap:2rem; padding:2rem 0;">
  <div style="display:flex; flex-direction:column; align-items:center; gap:0.5rem;">
    <i class="devicon-cplusplus-plain" style="font-size:2.8rem;"></i>
    <span style="font-size:0.8rem;">C++</span>
  </div>
  <div style="display:flex; flex-direction:column; align-items:center; gap:0.5rem;">
    <i class="devicon-python-plain" style="font-size:2.8rem;"></i>
    <span style="font-size:0.8rem;">Python</span>
  </div>
  <div style="display:flex; flex-direction:column; align-items:center; gap:0.5rem;">
    <i class="devicon-matlab-plain" style="font-size:2.8rem;"></i>
    <span style="font-size:0.8rem;">MATLAB</span>
  </div>
  <div style="display:flex; flex-direction:column; align-items:center; gap:0.5rem;">
    <i class="devicon-linux-plain" style="font-size:2.8rem;"></i>
    <span style="font-size:0.8rem;">Linux</span>
  </div>
  <div style="display:flex; flex-direction:column; align-items:center; gap:0.5rem;">
    <i class="devicon-git-plain" style="font-size:2.8rem;"></i>
    <span style="font-size:0.8rem;">Git</span>
  </div>
  <div style="display:flex; flex-direction:column; align-items:center; gap:0.5rem;">
    <i class="devicon-docker-plain" style="font-size:2.8rem;"></i>
    <span style="font-size:0.8rem;">Docker</span>
  </div>
  <div style="display:flex; flex-direction:column; align-items:center; gap:0.5rem;">
    <i class="devicon-bash-plain" style="font-size:2.8rem;"></i>
    <span style="font-size:0.8rem;">Bash</span>
  </div>
  <div style="display:flex; flex-direction:column; align-items:center; gap:0.5rem;">
    <i class="devicon-cmake-plain" style="font-size:2.8rem;"></i>
    <span style="font-size:0.8rem;">CMake</span>
  </div>
  <div style="display:flex; flex-direction:column; align-items:center; gap:0.5rem;">
    <i class="devicon-latex-original" style="font-size:2.8rem;"></i>
    <span style="font-size:0.8rem;">LaTeX</span>
  </div>
  <div style="display:flex; flex-direction:column; align-items:center; gap:0.5rem;">
    <i class="devicon-opencv-plain" style="font-size:2.8rem;"></i>
    <span style="font-size:0.8rem;">OpenCV</span>
  </div>
  <div style="display:flex; flex-direction:column; align-items:center; gap:0.5rem;">
    <i class="devicon-pytorch-original" style="font-size:2.8rem;"></i>
    <span style="font-size:0.8rem;">PyTorch</span>
  </div>
</div>

<div style="display:flex; flex-wrap:wrap; justify-content:center; gap:1rem; padding:0 0 2rem 0;">
  <span class="skill-tag">ROS 2</span>
  <span class="skill-tag">Gazebo</span>
  <span class="skill-tag">Isaac Sim</span>
  <span class="skill-tag">MAVROS</span>
  <span class="skill-tag">ros2_control</span>
  <span class="skill-tag">Simulink</span>
  <span class="skill-tag">KDL</span>
  <span class="skill-tag">Eigen</span>
</div>

<hr>

<h2>Education</h2>

<div class="skills-grid">
  <div class="skill-category">
    <h3>Johns Hopkins University</h3>
    <p><strong>M.S.E. Robotics</strong> — Control, Perception & Autonomy</p>
    <div class="skill-tags">
      <span class="skill-tag">LCSR Distinguished Scholar</span>
    </div>
  </div>

  <div class="skill-category">
    <h3>University of Prince Edward Island</h3>
    <p><strong>B.Sc. Sustainable Design Engineering</strong> — Mechatronics</p>
    <div class="skill-tags">
      <span class="skill-tag">First in Class</span>
      <span class="skill-tag">Engineers PEI Award</span>
    </div>
  </div>
</div>

<hr>

<h2>Certifications</h2>

<div class="skills-grid">
  <div class="skill-category">
    <h3>✈️ FAA Part 107 Remote Pilot</h3>
    <p>United States</p>
  </div>

  <div class="skill-category">
    <h3>✈️ Transport Canada Remote Pilot (Advanced)</h3>
    <p>Canada</p>
  </div>
</div>

<hr>

<h2>Teaching</h2>

<div class="skills-grid">
  <div class="skill-category">
    <h3>Algorithms for Sensor-Based Robotics</h3>
    <p>EN.601.463/663 — Johns Hopkins</p>
    <div class="skill-tags">
      <span class="skill-tag">82+ Students</span>
      <span class="skill-tag">UR5 Hardware Validation</span>
      <span class="skill-tag">Motion Planning</span>
    </div>
  </div>

  <div class="skill-category">
    <h3>Design & Analysis of Dynamic Systems</h3>
    <p>EN.530.343 — Johns Hopkins</p>
    <div class="skill-tags">
      <span class="skill-tag">Control Theory</span>
      <span class="skill-tag">Servo Systems</span>
      <span class="skill-tag">Stability Analysis</span>
    </div>
  </div>

  <div class="skill-category">
    <h3>Robot Sensors and Actuators</h3>
    <p>EN.530.430 — Johns Hopkins</p>
    <div class="skill-tags">
      <span class="skill-tag">Sensor Calibration</span>
      <span class="skill-tag">Embedded Systems</span>
      <span class="skill-tag">Arduino</span>
    </div>
  </div>
</div>

</div>
