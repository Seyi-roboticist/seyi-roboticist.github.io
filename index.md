---
layout: default
title: Home
---

<link rel="stylesheet" href="https://cdn.jsdelivr.net/gh/devicons/devicon@latest/devicon.min.css">

<style>
  .hero-personal {
    padding: 5rem 0 3rem;
  }
  .hero-tagline {
    letter-spacing: 0.05em;
    opacity: 0.85;
  }
  .intro-section p {
    font-size: 1.15rem;
    line-height: 1.8;
    max-width: 720px;
    margin: 0 auto;
  }
  .tools-icon-grid {
    display: flex;
    flex-wrap: wrap;
    justify-content: center;
    gap: 2.2rem;
    padding: 2.5rem 0 1.5rem;
  }
  .tool-icon {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 0.5rem;
    transition: transform 0.2s ease;
  }
  .tool-icon:hover {
    transform: translateY(-4px);
  }
  .tool-icon i {
    font-size: 2.8rem;
    opacity: 0.9;
  }
  .tool-icon span {
    font-size: 0.75rem;
    opacity: 0.7;
    letter-spacing: 0.03em;
  }
  .extra-tools {
    display: flex;
    flex-wrap: wrap;
    justify-content: center;
    gap: 0.6rem;
    padding: 0.5rem 0 2rem;
  }
  .section-divider {
    border: none;
    border-top: 1px solid rgba(128,128,128,0.15);
    margin: 3rem 0;
  }
  .skill-category h3 {
    margin-bottom: 0.8rem;
  }
  .skill-category p {
    font-size: 0.92rem;
    line-height: 1.6;
    opacity: 0.8;
    margin-bottom: 1rem;
  }
</style>

<div class="hero-personal">
  <div class="container">
    <div class="hero-content">
      <div class="hero-info-wrapper">
        <div class="hero-text">
          <h1 class="hero-name">Oluwaseyi R. Afolayan</h1>
          <p class="hero-title">Full-Stack Roboticist</p>
          <p class="hero-tagline">Manipulation · Flight Controls · Perception · Autonomy · State Estimation</p>
        </div>

        <div class="hero-actions">
          <a href="{{ '/about/' | relative_url }}" class="btn-primary">About Me</a>
          <a href="{{ '/projects/' | relative_url }}" class="btn-secondary">View Projects</a>
        </div>
      </div>
    </div>
  </div>
</div>

<div class="intro-section">
  <div class="container">
    <p>I build robots that manipulate, navigate, and fly. From sub-millimeter arm control at 500Hz to autonomous UAV flight systems to deep learning for 3D reconstruction — I work across the full robotics stack, from dynamics on the whiteboard to machines in the real world.</p>
  </div>
</div>

<hr class="section-divider">

<div class="skills-section">
  <div class="container">
    <h2>What I Build</h2>
    <div class="skills-grid">

      <div class="skill-category">
        <h3>🦾 Robotic Manipulation</h3>
        <p>Real-time Cartesian control on UR5/UR5e — singularity-robust inverse kinematics at 500Hz with sub-millimeter tracking accuracy.</p>
        <div class="skill-tags">
          <span class="skill-tag">Jacobian IK</span>
          <span class="skill-tag">500Hz Servo Loops</span>
          <span class="skill-tag">±0.7mm Accuracy</span>
          <span class="skill-tag">ros2_control</span>
          <span class="skill-tag">SVD + Tikhonov</span>
        </div>
      </div>

      <div class="skill-category">
        <h3>🛩️ Flight Controls & GNC</h3>
        <p>Full autopilot design from first principles — 6DOF modeling, successive loop closure, and dual EKF state estimation for fixed-wing and rotary-wing UAVs.</p>
        <div class="skill-tags">
          <span class="skill-tag">6DOF Modeling</span>
          <span class="skill-tag">Successive Loop Closure</span>
          <span class="skill-tag">PID/PIR</span>
          <span class="skill-tag">Stability Margins</span>
          <span class="skill-tag">EKF Sensor Fusion</span>
        </div>
      </div>

      <div class="skill-category">
        <h3>👁️ Perception & Deep Learning</h3>
        <p>Computer vision, neural scene representations, and learned perception pipelines. NeRF for 3D reconstruction and sim-to-real transfer for deployment.</p>
        <div class="skill-tags">
          <span class="skill-tag">Computer Vision</span>
          <span class="skill-tag">Deep Learning (PyTorch)</span>
          <span class="skill-tag">NeRF</span>
          <span class="skill-tag">Machine Perception</span>
          <span class="skill-tag">Sim-to-Real</span>
        </div>
      </div>

      <div class="skill-category">
        <h3>🤖 Autonomy & Navigation</h3>
        <p>Full-stack ROS 2 autonomy — from Gazebo simulation through real hardware deployment on aerial, ground, and underwater platforms.</p>
        <div class="skill-tags">
          <span class="skill-tag">ROS 2</span>
          <span class="skill-tag">Waypoint Navigation</span>
          <span class="skill-tag">Trajectory Tracking</span>
          <span class="skill-tag">Mobile Robotics</span>
          <span class="skill-tag">Underwater Autonomy</span>
        </div>
      </div>

      <div class="skill-category">
        <h3>📡 State Estimation</h3>
        <p>Fusing noisy, heterogeneous sensor data into reliable state estimates — EKF design, GPS smoothing, and navigation in GPS-denied environments.</p>
        <div class="skill-tags">
          <span class="skill-tag">Extended Kalman Filters</span>
          <span class="skill-tag">GPS Smoothing</span>
          <span class="skill-tag">IMU/Accel/Mag Fusion</span>
          <span class="skill-tag">GPS-Denied Navigation</span>
        </div>
      </div>

      <div class="skill-category">
        <h3>🏭 Industrial Automation</h3>
        <p>Safety-critical control systems for 24/7 production — PLC programming, fault-tolerant interlocks, and deterministic sequencing across manufacturing industries.</p>
        <div class="skill-tags">
          <span class="skill-tag">Allen-Bradley PLCs</span>
          <span class="skill-tag">Safety Interlocks</span>
          <span class="skill-tag">VFD Motor Control</span>
          <span class="skill-tag">Embedded Real-Time</span>
        </div>
      </div>

    </div>
  </div>
</div>

<hr class="section-divider">

<div class="skills-section">
  <div class="container">
    <h2>Languages & Tools</h2>

    <div class="tools-icon-grid">
      <div class="tool-icon">
        <i class="devicon-cplusplus-plain"></i>
        <span>C++</span>
      </div>
      <div class="tool-icon">
        <i class="devicon-python-plain"></i>
        <span>Python</span>
      </div>
      <div class="tool-icon">
        <i class="devicon-matlab-plain"></i>
        <span>MATLAB</span>
      </div>
      <div class="tool-icon">
        <i class="devicon-pytorch-original"></i>
        <span>PyTorch</span>
      </div>
      <div class="tool-icon">
        <i class="devicon-opencv-plain"></i>
        <span>OpenCV</span>
      </div>
      <div class="tool-icon">
        <i class="devicon-ros-original"></i>
        <span>ROS 2</span>
      </div>
      <div class="tool-icon">
        <i class="devicon-gazebo-plain"></i>
        <span>Gazebo</span>
      </div>
      <div class="tool-icon">
        <i class="devicon-linux-plain"></i>
        <span>Linux</span>
      </div>
      <div class="tool-icon">
        <i class="devicon-git-plain"></i>
        <span>Git</span>
      </div>
      <div class="tool-icon">
        <i class="devicon-docker-plain"></i>
        <span>Docker</span>
      </div>
      <div class="tool-icon">
        <i class="devicon-cmake-plain"></i>
        <span>CMake</span>
      </div>
      <div class="tool-icon">
        <i class="devicon-bash-plain"></i>
        <span>Bash</span>
      </div>
      <div class="tool-icon">
        <i class="devicon-latex-original"></i>
        <span>LaTeX</span>
      </div>
    </div>

    <div class="extra-tools">
      <span class="skill-tag">Isaac Sim</span>
      <span class="skill-tag">MAVROS</span>
      <span class="skill-tag">ros2_control</span>
      <span class="skill-tag">Simulink</span>
      <span class="skill-tag">KDL</span>
      <span class="skill-tag">Eigen</span>
      <span class="skill-tag">Pixhawk</span>
      <span class="skill-tag">NVIDIA Jetson</span>
    </div>
  </div>
</div>

<hr class="section-divider">

<div class="projects-showcase">
  <div class="container">
    <div class="section-header">
      <h2>Portfolio</h2>
      <p class="section-subtitle">A curated collection of my research and designs</p>
    </div>

    <div class="projects-grid-featured">
      {% for project in site.projects limit: 6 %}
        <div class="project-card-featured">
          <div class="project-media">
            {% if project.featured_image %}
              <img src="{{ project.featured_image | relative_url }}" alt="{{ project.title }}">
            {% else %}
              <div class="project-placeholder">
                <i class="fas fa-robot"></i>
              </div>
            {% endif %}
            <div class="project-overlay">
              <a href="{{ project.url | relative_url }}" class="project-link">
                <i class="fas fa-arrow-right"></i>
              </a>
            </div>
          </div>
          <div class="project-info">
            <h3><a href="{{ project.url | relative_url }}">{{ project.title }}</a></h3>
            <p>{{ project.description | truncate: 100 }}</p>
          </div>
        </div>
      {% endfor %}
    </div>

    <div class="showcase-actions">
      <a href="{{ '/projects/' | relative_url }}" class="btn-primary">View All Projects</a>
    </div>
  </div>
</div>
