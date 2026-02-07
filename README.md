# Failure-Aware 3D Object Tracking in Simulation (ROS2)

## Overview
This project implements a **failure-aware multi-object 3D perception and tracking system** for mobile robots using **simulated RGB-D sensors**.  
The system is designed to operate under **realistic sensing conditions**, including noisy depth, occlusions, missed detections, and latency constraints.

Rather than assuming perfect perception, the pipeline explicitly **detects perception failures** and applies **recovery strategies** to maintain robust object tracking.

---

## System Architecture

RGB Image  →  2D Object Detection  
Depth Image →  Depth Validation & Projection  
Camera Intrinsics →  3D Position Estimation  
↓  
Multi-Object Tracking (Kalman Filter + Data Association)  
↓  
Failure Detection & Recovery Logic  
↓  
3D Object Tracks + State (`OK / OCCLUDED / LOST / RECOVERING`)

---

## Features

- RGB-D based **3D object localization**
- **Multi-object tracking** with identity preservation
- Explicit **failure detection**:
  - occlusions
  - depth dropouts
  - detector confidence collapse
  - latency violations
- **Recovery mechanisms** using motion prediction and cautious re-identification
- **Latency and FPS profiling**
- Visual debugging using **RViz**
- Simulation-based **noise injection** for robustness testing

---

## Tech Stack

- ROS2
- Gazebo / Isaac Sim / PyBullet (simulation)
- OpenCV
- NumPy
- YOLOv8 (2D detection)
- Kalman Filter–based tracker
- Python

---

## ROS2 Nodes (Planned)

- `rgbd_projection_node`
  - Projects 2D detections into 3D using depth + intrinsics
- `detection_node`
  - Runs object detection and publishes bounding boxes
- `tracking_node`
  - Maintains object tracks and handles data association
- `failure_monitor_node`
  - Detects perception failures and triggers recovery logic

---

## Failure States

Each tracked object is assigned one of the following states:

- `TRACKING_OK` – normal tracking
- `OCCLUDED` – temporarily hidden but predictable
- `LOST` – tracking unreliable
- `RECOVERING` – attempting re-identification

---

## Evaluation Metrics

- End-to-end latency
- Detection FPS
- Track stability under noise
- Failure frequency per scenario
- Recovery success rate

---

## Current Status

- [x] Project scaffold created
- [ ] Simulation setup
- [ ] RGB-D stream integration
- [ ] 2D detection
- [ ] 3D projection
- [ ] Multi-object tracking
- [ ] Failure detection
- [ ] Recovery logic
- [ ] Profiling and analysis

---

## Future Work

- Stereo-based depth estimation
- Jetson deployment and TensorRT optimization
- Closed-loop navigation or manipulation integration

---

## Motivation

Real-world robots operate under imperfect sensing conditions.  
This project focuses on **robust system behavior**, not idealized perception accuracy.

