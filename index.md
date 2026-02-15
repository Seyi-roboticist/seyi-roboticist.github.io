---
layout: default
title: Home
---

<div class="hero-personal">
  <div class="container">
    <div class="hero-content">
      <div class="hero-info-wrapper">
        <div class="hero-text">
          <h1 class="hero-name">Oluwaseyi R. Afolayan</h1>
          <p class="hero-title">Controls & Autonomy Engineer</p>
          <p class="hero-tagline">Flight Controls · State Estimation · Real-Time Systems · UAV Autonomy</p>
        </div>

        <div class="hero-actions">
          <a href="/about/" class="btn-primary">About Me</a>
          <a href="/projects/" class="btn-secondary">View Projects</a>
        </div>
      </div>
    </div>
  </div>
</div>

<div class="intro-section">
  <div class="container">
    <p>Roboticist and Controls Engineer specializing in real-time flight control, perception, sensor fusion, and safety-critical autonomy. From aviation systems maintenance to leading UAV projects at Johns Hopkins and industrial automation at The RDI Group, I design systems that perform reliably in dynamic environments.</p>
  </div>
</div>

<div class="skills-section">
  <div class="container">
    <h2>Technical Expertise</h2>
    <div class="skills-grid">
      <div class="skill-category">
        <h3>Controls & Estimation</h3>
        <div class="skill-tags">
          <span class="skill-tag">PID/PIR Tuning</span>
          <span class="skill-tag">Sensor Fusion</span>
          <span class="skill-tag">EKF/UKF</span>
          <span class="skill-tag">Successive Loop Closure</span>
          <span class="skill-tag">Stability Analysis</span>
          <span class="skill-tag">State-Space Methods</span>
        </div>
      </div>

      <div class="skill-category">
        <h3>Perception & Machine Learning</h3>
        <div class="skill-tags">
          <span class="skill-tag">Computer Vision (OpenCV)</span>
          <span class="skill-tag">Deep Learning Fundamentals</span>
          <span class="skill-tag">Machine Perception</span>
          <span class="skill-tag">Reinforcement Learning Concepts</span>
          <span class="skill-tag">Sim-to-Real Transfer</span>
        </div>
      </div>

      <div class="skill-category">
        <h3>Robotics Frameworks & Tools</h3>
        <div class="skill-tags">
          <span class="skill-tag">ROS 2</span>
          <span class="skill-tag">Gazebo</span>
          <span class="skill-tag">NVIDIA Isaac Sim</span>
          <span class="skill-tag">MAVROS</span>
          <span class="skill-tag">ros2_control</span>
        </div>
      </div>

      <div class="skill-category">
        <h3>Programming & Languages</h3>
        <div class="skill-tags">
          <span class="skill-tag">C++</span>
          <span class="skill-tag">Python</span>
          <span class="skill-tag">MATLAB/Simulink</span>
          <span class="skill-tag">Git</span>
          <span class="skill-tag">Linux</span>
          <span class="skill-tag">Docker</span>
          <span class="skill-tag">Bash</span>
        </div>
      </div>

      <div class="skill-category">
        <h3>Hardware & Platforms</h3>
        <div class="skill-tags">
          <span class="skill-tag">UR5/UR5e Manipulators</span>
          <span class="skill-tag">Allen-Bradley PLCs</span>
          <span class="skill-tag">NVIDIA Jetson</span>
          <span class="skill-tag">Pixhawk Flight Controllers</span>
          <span class="skill-tag">VFD Systems</span>
          <span class="skill-tag">Embedded Real-Time Systems</span>
        </div>
      </div>

      <div class="skill-category">
        <h3>Other</h3>
        <div class="skill-tags">
          <span class="skill-tag">LaTeX</span>
          <span class="skill-tag">Simulation-to-Hardware Pipelines</span>
          <span class="skill-tag">Basic Computer Vision & ML Pipelines</span>
        </div>
      </div>
    </div>
  </div>
</div>

<!-- Keep the project showcase grid (update with your projects in _projects/) -->
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
      <a href="/projects/" class="btn-primary">View All Projects</a>
    </div>
  </div>
</div>

<footer class="site-footer">
  <div class="container">
    <p>© {{ 'now' | date: "%Y" }} Oluwaseyi R. Afolayan. Built with Jekyll.</p>
  </div>
</footer>
