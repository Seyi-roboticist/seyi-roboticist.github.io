---
layout: project
title: "Bidirectional EST Path Planner for UR5"
description: "Custom Bidirectional Expansive Space Tree (BiEST) motion planner integrated with MoveIt 2 for collision-free trajectory generation on the UR5 manipulator. Weighted roulette-wheel sampling, goal-biased exploration, and iterative backtracking across dual trees — tested in RViz, Gazebo, Isaac Sim, and on physical hardware."
date: 2024-12-01
categories: [Motion Planning, C++, ROS2, MoveIt, Manipulation]
featured_image: "https://github.com/Seyi-roboticist/OluwaseyiR.github.io/assets/143431845/cdd1d19e-3765-42c3-9cce-d4a27d502e23"

code_files:
  - name: "BiEST Core Algorithm"
    file: "assignment3_context.cpp"
    language: "cpp"
    content: |
      // Bidirectional EST — alternating tree expansion with bridge detection
      ASBRContext::path ASBRContext::est(
          const vertex& q_init, const vertex& q_goal)
      {
          std::vector<vertex> V_root{q_init}, V_goal{q_goal};
          std::vector<index>  parent{0}, parent_goal{0};
          std::vector<weight> w{1.0}, w_goal{1.0};
          const int MAX_ITERATIONS{11300};
          double nearThreshold{13};
          long bridgeRootIdx{-1}, bridgeGoalIdx{-1};

          for (uint16_t i{1}; i < MAX_ITERATIONS; ++i) {
              if (random_bool()) {
                  if (extend(V_root, parent, w, V_goal,
                      bridgeRootIdx, bridgeGoalIdx,
                      nearThreshold, q_goal))
                      break;
              } else {
                  if (extend(V_goal, parent_goal, w_goal, V_root,
                      bridgeGoalIdx, bridgeRootIdx,
                      nearThreshold, q_init))
                      break;
              }
          }
          // Bridge the two half-paths
          path P_root = search_path(V_root, parent, 0, bridgeRootIdx);
          path P_goal = search_path(V_goal, parent_goal, 0, bridgeGoalIdx);
          P_root.insert(P_root.end(), P_goal.rbegin(), P_goal.rend());
          return P_root;
      }

  - name: "Weighted Sampling & Goal Biasing"
    file: "assignment3_context.cpp"
    language: "cpp"
    content: |
      // Roulette-wheel selection — favors less-explored regions
      ASBRContext::index ASBRContext::select_config_from_tree(
          const std::vector<weight>& w)
      {
          static std::uniform_real_distribution<> uniform_distro(
              0.0, std::accumulate(w.begin(), w.end(), 0.0));
          double random_point{uniform_distro(generator)};
          double cum_weight{0.0};
          index i{0};
          for (; i < w.size(); ++i) {
              cum_weight += w[i];
              if (random_point <= cum_weight) break;
          }
          return (i >= w.size()) ? w.size() - 1 : i;
      }

      // Sample nearby with 18.5% goal bias
      ASBRContext::vertex ASBRContext::sample_nearby(
          const vertex& q, const vertex& q_goal)
      {
          static std::normal_distribution<> gauss(0, M_PI);
          static std::uniform_real_distribution<double> coin(0, 1);
          const double GOAL_BIAS{0.185};

          if (coin(generator) < GOAL_BIAS && !state_collides(q_goal))
              return q_goal;

          vertex q_rand(q.size());
          for (std::size_t i{0}; i < q.size(); ++i)
              q_rand[i] = std::fmod(q[i] + gauss(generator) + M_PI,
                                     2 * M_PI) - M_PI;
          return state_collides(q_rand) ? q : q_rand;
      }

  - name: "Tree Extension & Bridge Detection"
    file: "assignment3_context.cpp"
    language: "cpp"
    content: |
      // Extend one tree toward the other, check for bridge connection
      bool ASBRContext::extend(
          std::vector<vertex>& V_from,
          std::vector<index>& parent_from,
          std::vector<weight>& w_from,
          const std::vector<vertex>& V_to,
          long& bridgeFromIdx, long& bridgeToIdx,
          double nearThreshold, const vertex& q_target)
      {
          index idx = select_config_from_tree(w_from);
          vertex q_rand = sample_nearby(V_from[idx], q_target);
          index idx_near = find_nearest_configuration(V_from, q_rand);

          if (is_local_path_collision_free(V_from[idx_near], q_rand)) {
              V_from.push_back(q_rand);
              parent_from.push_back(idx_near);
              w_from.push_back(1.0 / (1.0 + w_from[idx_near]));

              index idx_to = find_nearest_configuration(V_to, q_rand);
              if (euclidean_norm(q_rand, V_to[idx_to]) < nearThreshold
                  && is_local_path_collision_free(q_rand, V_to[idx_to])) {
                  bridgeFromIdx = V_from.size() - 1;
                  bridgeToIdx = idx_to;
                  return true;  // Trees connected
              }
          }
          return false;
      }
---

## Overview

This project implements a custom Bidirectional Expansive Space Tree (BiEST) motion planner for the UR5 6-DOF manipulator, integrated as a MoveIt 2 planning plugin within ROS 2. The planner grows two trees simultaneously — one from the start configuration and one from the goal — and attempts to bridge them through shared configuration space. It was developed for the graduate-level *Algorithms for Sensor-Based Robotics* course at Johns Hopkins University and tested across RViz, Gazebo, NVIDIA Isaac Sim, and physical UR5 hardware.

The core algorithm combines three techniques for efficient exploration: weighted roulette-wheel sampling that favors under-explored regions of the tree, Gaussian perturbation in joint space with wrap-around for the UR5's $[-\pi, \pi]$ joint limits, and an 18.5% goal-biasing probability that periodically steers exploration toward the target configuration. Once both trees grow close enough (within a configurable threshold), a bridge connection is attempted with collision checking along the local path. The final trajectory is recovered by backtracking through each tree's parent chain and concatenating the two half-paths.

## Demo

![BiEST Path Planner Demo](https://github.com/Seyi-roboticist/OluwaseyiR.github.io/assets/143431845/cdd1d19e-3765-42c3-9cce-d4a27d502e23)

## System Architecture

<script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>
<script>mermaid.initialize({startOnLoad:true, theme:'dark'});</script>

<pre class="mermaid">
flowchart TB
    A["MoveIt 2 Planning Request"] --> B["solve()"]
    B --> C["est(q_init, q_goal)"]
    C --> D{"Alternate Trees"}
    D -->|"Tree 1"| E["extend(V_root → V_goal)"]
    D -->|"Tree 2"| F["extend(V_goal → V_root)"]
    E --> G["select_config_from_tree(w)"]
    F --> G
    G --> H["sample_nearby(q, q_target)"]
    H --> I["find_nearest_configuration()"]
    I --> J{"Local Path Collision Free?"}
    J -->|"Yes"| K["Add to Tree + Update Weights"]
    J -->|"No"| D
    K --> L{"Bridge Distance < Threshold?"}
    L -->|"Yes"| M["search_path() + Concatenate"]
    L -->|"No"| D
    M --> N["Interpolated Trajectory → RViz"]
</pre>

## Algorithm Details

### Bidirectional Tree Growth

The BiEST planner maintains two trees rooted at the start and goal configurations. On each iteration, the algorithm alternates which tree expands — a deterministic toggle that proved more efficient than random selection (72ms vs 351ms planning time for complex scenes). Each expansion attempt follows this pipeline:

1. **Select** a vertex from the active tree using weighted roulette-wheel sampling
2. **Sample** a nearby configuration with Gaussian noise ($\sigma = \pi$) and optional goal bias
3. **Find** the nearest vertex in the tree to the new sample (Euclidean norm in C-space)
4. **Validate** the local path between nearest and sample via discretized collision checking
5. **Check** if the new vertex bridges to the opposing tree within the threshold distance

### Weighted Exploration

Each vertex in the tree carries a weight that inversely decays with its parent's weight:

$$w_{\text{new}} = \frac{1}{1 + w_{\text{parent}}}$$

This means vertices spawned from heavily-explored regions get lower weights, while vertices in sparse regions retain higher weights. The roulette-wheel selector draws from the cumulative weight distribution, naturally biasing expansion toward under-explored areas of the configuration space.

### Goal Biasing

With probability $p = 0.185$, the sampler bypasses Gaussian perturbation and directly returns the target configuration (the opposing tree's root). This acts as a "greediness dial" — too low and the trees wander aimlessly, too high and the planner gets stuck trying to force direct connections through cluttered environments. The 18.5% value was empirically tuned across multiple obstacle scenarios.

### Gaussian Sampling with Joint-Limit Wrapping

New configurations are generated by adding Gaussian noise ($\mu = 0, \sigma = \pi$) to each joint independently, then wrapping the result into $[-\pi, \pi]$:

$$q_i^{\text{rand}} = \left((q_i + \mathcal{N}(0, \pi) + \pi) \bmod 2\pi\right) - \pi$$

This ensures full workspace coverage while respecting the UR5's joint limits. If the resulting configuration collides, the sampler retries up to 1000 times before returning the original configuration.

### Collision Checking

Local paths between configurations are validated by discretized interpolation at resolution $\Delta t = 0.005$ (200 checkpoints per segment). At each interpolated configuration, MoveIt's `isStateColliding()` queries the planning scene against all registered obstacles. This fine resolution catches narrow passages that coarser checks would miss.

### Path Recovery

Once a bridge is found, the algorithm backtrackd through each tree's parent array using iterative backtracking (not recursive, to avoid stack overflow on deep trees). A visited-node check prevents cycles. The two half-paths are concatenated — the root path forward, the goal path reversed — producing the complete collision-free trajectory.

## Results

| Metric | Value |
|---|---|
| Planning success rate | 95%+ across test scenarios |
| Planning time (simple scene) | ~72 ms |
| Planning time (Bugatti obstacle) | ~351 ms (random) / ~72 ms (alternating) |
| Collision check resolution | 200 points per segment ($\Delta t = 0.005$) |
| Goal bias probability | 18.5% |
| Bridge threshold | 13 rad (C-space Euclidean) |
| Max iterations | 11,300 |
| Tested platforms | RViz, Gazebo, Isaac Sim, physical UR5 |

## Links

- [Project Page](https://github.com/Seyi-roboticist/OluwaseyiR.github.io/tree/main/Projects/UR5_PathPlanning)
