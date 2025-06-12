---
layout: page
title: about
permalink: /about/
---

# About Horus

**Horus** is an end-to-end autonomy stack for ArduPilot/PX4 drones, built with ROS 2. It enables 3D drone navigation in cluttered environments using onboard perception, planning, and control.

The project is in early development, with a long-term vision of robust autonomous drone operation through dense and dynamic spaces.

---

## System Overview

Horus is divided into three primary subsystems:

- **Perception** – Sensors and environment modeling
- **Task Planning** – High-level decision-making and mission control
- **Motion Control** – Navigation, trajectory planning, and execution

---

## 🧠 Subsystems Breakdown

### 👁️ Perception

The perception module is responsible for sensing and mapping the environment. Current and planned components include:

- `Mapper`: Converts point clouds to voxel grids
- **Planned**:
  - ORB-SLAM3 for visual odometry and tracking
  - Object detection for semantic context

📁 Code: `src/perception/nodes`

---

### 🧠 Task Planning

Task planning manages mission logic, currently under development. The vision includes:

- A Finite State Machine (FSM) architecture
- Integration with an LLM-based agent that generates FSMs from natural language prompts

---

### 🚀 Motion Control

The most developed part of Horus, motion control is structured hierarchically:

- `GlobalPlanner`: Chooses global waypoints (currently random; PRM-based planner in progress)
- `LocalPlanner`: Avoids dynamic and local obstacles en route to the next waypoint
- `TrajectoryController`: Executes motion via pure pursuit path tracking

📁 Code: `src/motion_control/nodes`

---

## 🧪 Testing & Hardware

- **Unit Tests**: Located in `test/tests.cpp`, powered by `gtest`
- **Hardware Platform**: Testing on Raspberry Pi 5 + Pixhawk 2.4.8
  ![Hardware Setup](assets/hardware.jpg)

---

## 🔧 Roadmap

Planned improvements include:

- [ ] ORB-SLAM3 integration
- [ ] FSM-based task planning with natural language input
- [ ] Probabilistic Roadmap (PRM) for global planning
- [ ] Obstacle inflation for safety
- [ ] Faster point cloud voxelization (currently 0.5–2.5s per cloud)

---

## 🎥 Demo

![FSM Activated](assets/fsm_activated.png)  
[▶️ Watch Demo](https://www.youtube.com/watch?v=EEtwEGAUKy8)

---

## 🤝 Contributing

We’re actively building Horus and welcome contributors!  
Join us by opening issues, starting discussions, or submitting pull requests.

---


