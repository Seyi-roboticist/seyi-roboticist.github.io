---
layout: project
title: "Fixed-Wing UAV Autopilot & State Estimation"
description: "Complete fixed-wing UAV autopilot built from first principles. Successive loop closure PID architecture, dual EKFs for attitude and GPS smoothing, wind disturbance rejection, and closed-loop waypoint tracking validated in nonlinear 6DOF Simulink simulation. Every Jacobian derived by hand."
date: 2024-11-28
status: completed
categories: [Controls, Flight, MATLAB, State-Estimation]
featured_image: "/assets/images/projects/fixedwing-uav/uavsim_simulation.png"

gallery:
  - type: "image"
    file: "/assets/images/projects/fixedwing-uav/uavsim_block_diagram.png"
    description: "Full Simulink block diagram: trajectory commands, autopilot flight control, forces and moments, kinematics and dynamics, sensor models, state estimation feedback loop, and wind/gusting subsystem."
  - type: "image"
    file: "/assets/images/projects/fixedwing-uav/uavsim_flight.png"
    description: "Closed-loop flight simulation output: altitude, airspeed, pitch, and roll tracking with 3D visualization of the UAV navigating waypoints under wind disturbance."
  - type: "image"
    file: "/assets/images/projects/fixedwing-uav/uavsim_simulation.png"
    description: "Full UAVSIM display: 200-second closed-loop run with gusting winds enabled, showing command tracking across all channels with state estimate feedback."

code_files:
  - name: "Attitude EKF (Roll & Pitch Estimator)"
    file: "attitude_ekf.m"
    language: "matlab"
    content: |
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % Attitude EKF - Estimate roll (phi) and pitch (theta)
      % from gyroscope prediction + accelerometer correction
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Q matrix - process noise from gyro
      Q_att = diag([P.sigma_noise_gyro^2, P.sigma_noise_gyro^2]);
      
      % R matrix - measurement noise from accel, scaled by 10^m
      m = 4.5;
      R_att = 10^m * diag([P.sigma_noise_accel^2, ...
              P.sigma_noise_accel^2, P.sigma_noise_accel^2]);
      
      persistent xhat_att P_att
      if(time == 0)
          xhat_att = [0; 0];  % [phi; theta] - start level
          P_att = diag([5*pi/180, 5*pi/180].^2);  % +/-5 deg
      end
      
      % === PREDICTION (N sub-steps) ===
      N = 10;
      for i = 1:N
          % State dynamics: Euler angle kinematics
          f_att = [p_gyro + q_gyro*sin(xhat_att(1))*tan(xhat_att(2)) ...
                   + r_gyro*cos(xhat_att(1))*tan(xhat_att(2));
                   q_gyro*cos(xhat_att(1)) - r_gyro*sin(xhat_att(1))];
      
          % Jacobian A
          A_att = getAjacobian(xhat_att, [p_gyro q_gyro r_gyro]);
      
          % Propagate state and covariance
          xhat_att = xhat_att + (P.Ts/N) * f_att;
          P_att = P_att + (P.Ts/N)*(A_att*P_att + P_att*A_att' + Q_att);
          P_att = real(0.5*P_att + 0.5*P_att');
      end
      
      % === MEASUREMENT UPDATE ===
      y_att = [ax_accel; ay_accel; az_accel];
      
      h_att = [q_gyro*Va_hat*sin(xhat_att(2)) + P.gravity*sin(xhat_att(2));
               r_gyro*Va_hat*cos(xhat_att(2)) ...
               - p_gyro*Va_hat*sin(xhat_att(2)) ...
               - P.gravity*cos(xhat_att(2))*sin(xhat_att(1));
               -q_gyro*Va_hat*cos(xhat_att(2)) ...
               - P.gravity*cos(xhat_att(2))*cos(xhat_att(1))];
      
      C_att = getCjacobian(xhat_att, [p_gyro q_gyro r_gyro Va_hat]);
      
      % Kalman gain and correction
      L_att = P_att*C_att'/(C_att*P_att*C_att' + R_att);
      xhat_att = xhat_att + L_att*(y_att - h_att);
      P_att = (eye(2) - L_att*C_att)*P_att;
      xhat_att = mod(xhat_att + pi, 2*pi) - pi;

  - name: "GPS Smoother EKF (Propagated Velocity Model)"
    file: "gps_smoother_ekf.m"
    language: "matlab"
    content: |
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % GPS Smoother EKF - 6-state position/velocity estimator
      % Uses accelerometer-propagated velocity between 1Hz GPS
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      persistent xhat_gps P_gps pn_gps_prev pe_gps_prev
      
      if (time == 0)
          xhat_gps = [pn_gps; pe_gps; -alt_gps; ...
                      Vn_gps; Ve_gps; Vd_gps];
          P_gps = diag([2^2, 2^2, 2^2, 0.5^2, 0.5^2, 0.5^2]);
          pn_gps_prev = pn_gps;
          pe_gps_prev = pe_gps;
      end
      
      % Detect new GPS measurement
      new_gps = (pn_gps ~= pn_gps_prev || pe_gps ~= pe_gps_prev);
      
      % Body-to-NED rotation + gravity compensation
      Rb_ned = eulerToRotationMatrix(phi_hat, theta_hat, psi_hat)';
      accel_ned = Rb_ned * [ax_accel; ay_accel; az_accel] + [0; 0; P.gravity];
      
      % System matrices
      A = [zeros(3,3), eye(3); zeros(3,3), zeros(3,3)];
      Q = diag([0.1^2, 0.1^2, 0.1^2, 0.2^2, 0.2^2, 0.2^2]);
      
      % === PROPAGATE with accel-based velocity model ===
      xhat_gps = xhat_gps + P.Ts * [xhat_gps(4:6); accel_ned];
      P_gps = P_gps + P.Ts * (A*P_gps + P_gps*A' + Q);
      
      % === CORRECT at 1Hz GPS rate ===
      if new_gps
          C = eye(6);
          R = diag([2 2 2 0.1 0.1 0.1].^2);
          y = [pn_gps; pe_gps; -alt_gps; Vn_gps; Ve_gps; Vd_gps];
          
          L = P_gps*C'/(C*P_gps*C' + R);
          xhat_gps = xhat_gps + L*(y - xhat_gps);
          P_gps = (eye(6) - L*C)*P_gps;
          P_gps = (P_gps + P_gps')/2;
          
          pn_gps_prev = pn_gps;
          pe_gps_prev = pe_gps;
      end

  - name: "Jacobian Helper Functions"
    file: "jacobians.m"
    language: "matlab"
    content: |
      % A Jacobian for attitude EKF (2x2)
      function A = getAjacobian(xhat_att, gyro)
          phi = xhat_att(1); theta = xhat_att(2);
          p = gyro(1); q = gyro(2); r = gyro(3);
          A = [q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta), ...
               (r*cos(phi) + q*sin(phi))*(sec(theta))^2;
               -r*cos(phi) - q*sin(phi), 0];
      end
      
      % C Jacobian for attitude EKF (3x2)
      function C = getCjacobian(xhat_att, inputs)
          phi = xhat_att(1); theta = xhat_att(2);
          p = inputs(1); q = inputs(2);
          r = inputs(3); Va = inputs(4);
          g = 9.81;
          C = [0, g*cos(theta) + Va*q*cos(theta);
               -g*cos(phi)*cos(theta), ...
               g*sin(phi)*sin(theta) - Va*r*sin(theta) - Va*p*cos(theta);
               g*cos(theta)*sin(phi), ...
               Va*q*sin(theta) + g*cos(phi)*sin(theta)];
      end
---

<style>
  .math-derivation {
    background: var(--surface-color, #1a1a2e);
    border: 1px solid var(--border-color, #333);
    border-left: 3px solid var(--primary-color, #6366f1);
    border-radius: 8px;
    padding: 1.5rem 2rem;
    margin: 1.5rem 0;
    overflow-x: auto;
  }
  .math-derivation h4 {
    color: var(--accent-color, var(--primary-color));
    font-size: 0.85rem;
    text-transform: uppercase;
    letter-spacing: 0.06em;
    margin-bottom: 1rem;
  }
  .system-block {
    background: linear-gradient(135deg, rgba(99,102,241,0.05) 0%, rgba(52,211,153,0.05) 100%);
    border: 1px solid rgba(99,102,241,0.15);
    border-radius: 10px;
    padding: 1.8rem;
    margin: 2rem 0;
  }
  .system-block h3 {
    color: var(--text-primary);
    margin-bottom: 1rem;
    font-size: 1.1rem;
  }
  .ekf-step {
    display: flex;
    align-items: flex-start;
    gap: 1rem;
    margin: 1rem 0;
  }
  .ekf-step-num {
    flex-shrink: 0;
    width: 28px;
    height: 28px;
    background: var(--primary-color, #6366f1);
    color: #fff;
    border-radius: 50%;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 0.8rem;
    font-weight: 700;
  }
  .note-box {
    background: rgba(52,211,153,0.06);
    border: 1px solid rgba(52,211,153,0.2);
    border-radius: 8px;
    padding: 1rem 1.4rem;
    margin: 1rem 0;
    font-size: 0.92rem;
  }
</style>

## Overview

I built a complete autopilot and state estimation system for a fixed-wing UAV from first principles. No off-the-shelf autopilot stacks, no borrowed controllers. Every transfer function, every Jacobian, every covariance matrix was derived by hand, implemented in MATLAB/Simulink, and validated against a nonlinear 6DOF simulation under wind disturbances.

The system has five tightly coupled subsystems: a nonlinear plant model with full aerodynamic forces, a successive loop closure PID control architecture, a 2-state Extended Kalman Filter for attitude estimation, a 6-state EKF GPS smoother with accelerometer-propagated velocity, and a 3-state EKF that estimates aerodynamic drag from camera-only measurements. I derived every Jacobian by hand, tuned every noise matrix with physical reasoning, and closed the loop around my own state estimates.

---

## Simulink Architecture

I organized the simulation as a modular Simulink block diagram with six major subsystems connected in a closed feedback loop:

1. **Trajectory Commands** generates waypoint-based reference signals for altitude, airspeed, and course
2. **Autopilot: Flight Control** implements nested PID controllers via successive loop closure, producing 4 control surface deflections ($$\delta_e, \delta_a, \delta_r, \delta_t$$)
3. **Forces & Moments** computes aerodynamic forces and propulsion from control inputs, wind, and current state
4. **Kinematics & Dynamics** integrates the full nonlinear 6DOF equations of motion (12 states)
5. **Sensors** simulates noisy gyroscope, accelerometer, GPS (1Hz), barometer, magnetometer, and pitot tube
6. **Autopilot: State Estimation** runs dual EKFs feeding clean estimates back to the flight controller

A feedback switch lets me toggle between truth and estimated states, so I can systematically evaluate how estimator quality impacts closed-loop performance.

---

## Control Architecture: Successive Loop Closure

The autopilot uses successive loop closure with bandwidth separation. Fast inner loops handle attitude, slower outer loops handle trajectory. No gain scheduling magic. Just clean PID design with proper bandwidth separation at each stage.

<div class="system-block">
<h3>Inner Loops (Attitude Rate + Attitude)</h3>

**Pitch channel:** PID with rate feedback commands elevator deflection $$\delta_e$$ from pitch error. The inner rate loop provides damping, the outer attitude loop tracks $$\theta_c$$.

**Roll channel:** Same structure. PID commands aileron deflection $$\delta_a$$ from roll error, with roll-rate gyro feedback for damping.

Both have anti-windup. I tuned them by checking gain and phase margins at each stage.
</div>

<div class="system-block">
<h3>Outer Loops (Trajectory)</h3>

**Altitude hold:** Commands pitch angle $$\theta_c$$ from altitude error. Bandwidth separated from the pitch inner loop by at least 5x.

**Airspeed hold:** Commands throttle $$\delta_t$$ from airspeed error.

**Course hold:** Commands roll angle $$\phi_c$$ from heading error, with crosswind compensation and crab angle correction.
</div>

I derived transfer functions for all four channels and tuned the PID gains through systematic iteration, verifying margins at each stage.

---

## Extended Kalman Filter: The Math

All three EKFs in this project follow the same structure. For a nonlinear system:

<div class="math-derivation">
<h4>General EKF Equations</h4>

**State dynamics:**

$$\dot{\hat{x}} = f(\hat{x}, u)$$

**Prediction step** (propagate state and covariance):

$$\hat{x}_{k+1} = \hat{x}_k + T_s \cdot f(\hat{x}_k, u_k)$$

$$P_{k+1} = P_k + T_s \left( A P_k + P_k A^\top + Q \right)$$

where $$A = \frac{\partial f}{\partial x}\bigg\rvert_{\hat{x}}$$ is the Jacobian of the dynamics.

**Correction step** (when measurements arrive):

$$L = P C^\top \left( C P C^\top + R \right)^{-1}$$

$$\hat{x} \leftarrow \hat{x} + L \left( y - h(\hat{x}) \right)$$

$$P \leftarrow (I - LC) P$$

where $$C = \frac{\partial h}{\partial x}\bigg\rvert_{\hat{x}}$$ is the Jacobian of the measurement model, and $$L$$ is the Kalman gain.
</div>

---

## EKF 1: Drag Estimation from Camera-Only Measurements

This EKF estimates the aerodynamic drag coefficient of a vertically launched projectile using only elevation angle measurements from a ground camera 200m away. I wanted to demonstrate how an EKF can infer states that are never directly measured, purely through the coupling encoded in the system dynamics.

<div class="math-derivation">
<h4>State Vector and Dynamics</h4>

$$\mathbf{x} = \begin{bmatrix} z \\ \dot{z} \\ D \end{bmatrix}, \quad \dot{\mathbf{x}} = f(\mathbf{x}) = \begin{bmatrix} \dot{z} \\ -g \mp \frac{1}{m}D\dot{z}^2 \\ 0 \end{bmatrix}$$

where the sign depends on the direction of travel: $$-\frac{1}{m}D\dot{z}^2$$ when $$\dot{z} \geq 0$$ (ascending), $$+\frac{1}{m}D\dot{z}^2$$ when $$\dot{z} < 0$$ (descending).
</div>

<div class="math-derivation">
<h4>A Matrix (Jacobian of Dynamics)</h4>

$$A = \frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix} \frac{\partial \dot{z}}{\partial z} & \frac{\partial \dot{z}}{\partial \dot{z}} & \frac{\partial \dot{z}}{\partial D} \\[6pt] \frac{\partial \ddot{z}}{\partial z} & \frac{\partial \ddot{z}}{\partial \dot{z}} & \frac{\partial \ddot{z}}{\partial D} \\[6pt] \frac{\partial \dot{D}}{\partial z} & \frac{\partial \dot{D}}{\partial \dot{z}} & \frac{\partial \dot{D}}{\partial D} \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 \\[4pt] 0 & \mp\frac{2}{m}D\dot{z} & \mp\frac{1}{m}\dot{z}^2 \\[4pt] 0 & 0 & 0 \end{bmatrix}$$

</div>

<div class="math-derivation">
<h4>Measurement Model and C Matrix</h4>

The camera measures the elevation angle to the projectile:

$$y = h(\mathbf{x}) = \tan^{-1}\!\left(\frac{z}{r_{\text{cam}}}\right), \quad r_{\text{cam}} = 200 \text{ m}$$

Taking the derivative:

$$\frac{\partial h}{\partial z} = \frac{1}{1 + \left(\frac{z}{r_{\text{cam}}}\right)^2} \cdot \frac{1}{r_{\text{cam}}} = \frac{r_{\text{cam}}}{r_{\text{cam}}^2 + z^2}$$

Therefore:

$$C = \begin{bmatrix} \frac{r_{\text{cam}}}{r_{\text{cam}}^2 + z^2} & 0 & 0 \end{bmatrix}$$
</div>

<div class="note-box">
<strong>What makes this interesting:</strong> The camera only observes one scalar: the elevation angle. Yet the EKF successfully estimates all three states, including drag, which is never directly measured. It works because the A and C matrices encode the physics. Changes in elevation angle inform height. Height changes over time inform velocity. Deviations from a pure ballistic trajectory inform drag. The Kalman gain automatically weights these inferences against the statistical uncertainties in P, Q, and R.

My resulting drag estimate converged to $$D = 1.285 \times 10^{-4}$$ N/(m/s)$$^2$$ with a 1-sigma uncertainty of $$\pm 4.649 \times 10^{-6}$$ N/(m/s)$$^2$$. The filter figured out drag from angle measurements alone.
</div>

---

## EKF 2: Attitude Estimation (Roll & Pitch)

The attitude EKF estimates roll ($$\phi$$) and pitch ($$\theta$$) by fusing gyroscope measurements in the prediction step with accelerometer measurements in the correction step. I derived every matrix by hand and tuned the noise parameters using physical reasoning about the sensor characteristics and modeling simplifications.

<div class="math-derivation">
<h4>State Vector and Euler Angle Kinematics</h4>

$$\hat{x}_{\text{att}} = \begin{bmatrix} \phi \\ \theta \end{bmatrix}, \quad f_{\text{att}} = \begin{bmatrix} p + q\sin\phi\tan\theta + r\cos\phi\tan\theta \\ q\cos\phi - r\sin\phi \end{bmatrix}$$

where $$p, q, r$$ are the raw gyroscope measurements (body-frame angular rates).
</div>

<div class="math-derivation">
<h4>A Matrix (2x2 Dynamics Jacobian)</h4>

$$A_{\text{att}} = \begin{bmatrix} q\cos\phi\tan\theta - r\sin\phi\tan\theta & (r\cos\phi + q\sin\phi)\sec^2\theta \\[4pt] -r\cos\phi - q\sin\phi & 0 \end{bmatrix}$$
</div>

<div class="math-derivation">
<h4>Measurement Model (Accelerometer)</h4>

Under the assumption that translational dynamics are much slower than rotational dynamics, the accelerometer measures the gravity vector resolved into the body frame plus centripetal terms:

$$h_{\text{att}} = \begin{bmatrix} q\hat{V}_a\sin\theta + g\sin\theta \\[4pt] r\hat{V}_a\cos\theta - p\hat{V}_a\sin\theta - g\cos\theta\sin\phi \\[4pt] -q\hat{V}_a\cos\theta - g\cos\theta\cos\phi \end{bmatrix}$$
</div>

<div class="math-derivation">
<h4>C Matrix (3x2 Measurement Jacobian)</h4>

$$C_{\text{att}} = \begin{bmatrix} 0 & g\cos\theta + \hat{V}_a q\cos\theta \\[4pt] -g\cos\phi\cos\theta & g\sin\phi\sin\theta - \hat{V}_a r\sin\theta - \hat{V}_a p\cos\theta \\[4pt] g\cos\theta\sin\phi & \hat{V}_a q\sin\theta + g\cos\phi\sin\theta \end{bmatrix}$$
</div>

### Noise Tuning

For process noise, I used the gyro noise variance directly since gyro noise maps almost one-to-one to attitude rate error:

$$Q = \begin{bmatrix} \sigma_{\text{gyro}}^2 & 0 \\ 0 & \sigma_{\text{gyro}}^2 \end{bmatrix}$$

The measurement noise was the tricky part. The raw accelerometer noise variance ($$\sigma^2_{\text{accel}} = 6.02 \times 10^{-4}$$ m$$^2$$/s$$^4$$) is way too small because the measurement model ignores aerodynamic effects, assumes translational dynamics are much slower than rotational, and throws away a bunch of other real-world dynamics. I had to scale R by $$10^{4.5}$$:

$$R = 10^{4.5} \times \begin{bmatrix} \sigma_{\text{accel}}^2 & 0 & 0 \\ 0 & \sigma_{\text{accel}}^2 & 0 \\ 0 & 0 & \sigma_{\text{accel}}^2 \end{bmatrix}$$

This effectively tells the EKF: "trust the gyro-based prediction more than the accelerometer correction." The bigger R compensates for all the modeling simplifications I made in the measurement model.

### Numerical Implementation

I use **10 sub-steps** ($$N = 10$$) per sample period for the prediction to keep the covariance propagation numerically stable. After each update, I symmetrize: $$P \leftarrow \frac{1}{2}(P + P^\top)$$ and force real. The state estimate gets wrapped to $$[-\pi, \pi]$$ to prevent angle discontinuities.

---

## EKF 3: GPS Smoother (Propagated Velocity Model)

The GPS smoother solves a real problem: 1Hz GPS is too slow for flight control. Raw 1Hz updates create discrete jumps in position and velocity, causing stepped responses in heading and altitude tracking. I built a 6-state EKF that propagates between GPS fixes using accelerometer data rotated into the NED frame, producing smooth estimates at the full sensor rate.

<div class="math-derivation">
<h4>State Vector and Propagation Model</h4>

$$\hat{x}_{\text{gps}} = \begin{bmatrix} \hat{p}_n \\ \hat{p}_e \\ \hat{p}_d \\ \hat{v}_n \\ \hat{v}_e \\ \hat{v}_d \end{bmatrix}_{6 \times 1}, \quad f(\hat{x}, u) = \begin{bmatrix} \hat{v}_n \\ \hat{v}_e \\ \hat{v}_d \\ a_n^{\text{NED}} \\ a_e^{\text{NED}} \\ a_d^{\text{NED}} \end{bmatrix}$$

where the NED-frame acceleration is computed from the body-frame accelerometer readings rotated by the attitude estimate and compensated for gravity:

$$\mathbf{a}^{\text{NED}} = (R_b^{\text{NED}})^\top \begin{bmatrix} a_x \\ a_y \\ a_z \end{bmatrix} + \begin{bmatrix} 0 \\ 0 \\ g \end{bmatrix}$$

This is the **propagated velocity model**: rather than assuming constant velocity between GPS fixes (which would give zero acceleration), I integrate body-frame accelerometer readings to propagate velocity at the full sensor rate.
</div>

<div class="math-derivation">
<h4>A Matrix (6x6 State Transition Jacobian)</h4>

Since position derivative equals velocity and velocity derivative equals acceleration (which enters as an input, not a state), the A matrix is clean:

$$A = \begin{bmatrix} 0_{3\times3} & I_{3\times3} \\ 0_{3\times3} & 0_{3\times3} \end{bmatrix}_{6\times6} = \begin{bmatrix} 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \end{bmatrix}$$
</div>

<div class="math-derivation">
<h4>C Matrix (Direct GPS Observation)</h4>

GPS provides direct measurements of all 6 states (position and velocity):

$$C = I_{6\times6}$$
</div>

### Measurement Gating

GPS corrections only fire when a new measurement actually arrives (roughly every 100 iterations at the simulation rate). I detect this by comparing current $$p_n, p_e$$ values against stored previous values using persistent variables. Simple but effective.

### Noise Parameters

$$Q = \text{diag}(0.1^2, 0.1^2, 0.1^2, 0.2^2, 0.2^2, 0.2^2)$$

$$R = \text{diag}(2^2, 2^2, 2^2, 0.1^2, 0.1^2, 0.1^2)$$

The R accounts for roughly 2m of GPS position error over a few seconds and 0.1 m/s GPS velocity noise.

---

## Closing the Loop

The final system feeds all my state estimates back to the autopilot:

| **State**         | **Source**                          |
|-------------------|------------------------------------|
| $$\phi, \theta$$  | Attitude EKF                       |
| $$\psi$$          | Filtered magnetometer              |
| $$p, q, r$$       | Filtered gyroscopes                |
| $$h$$             | Barometric altimeter               |
| $$V_a$$           | Pitot tube                         |
| Position, velocity | GPS Smoother EKF                  |

With the feedback switch set to estimates instead of truth, the UAV is simultaneously learning its attitude AND regulating flight commands. The first 10-15 seconds show transient oscillations as the attitude EKF converges, then the controller tracks waypoints smoothly.

---

## Results

I validated the complete system over 200-second simulations with gusting winds enabled:

- **Altitude tracking:** Smooth transitions between commanded altitudes with minimal overshoot after the initial transient
- **Attitude estimation:** Roll and pitch errors within +/-5 degrees (1-sigma), consistent with the initial covariance $$P_0$$
- **GPS smoothing:** Sub-2m position error variation with smooth curvature between 1Hz measurements, validating the propagated velocity model
- **Wind rejection:** Crosswind compensation maintained stable flight under steady wind and gusting conditions
- **Estimator-to-truth transfer:** Minimal performance degradation when switching from truth feedback to estimated feedback
