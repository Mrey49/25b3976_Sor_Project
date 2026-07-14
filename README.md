# ROBOTRACE — Autonomous Vision-Based Line Follower

## Problem Statement

In modern robotics, one of the fundamental capabilities of autonomous systems is visual navigation without external positioning systems.

From warehouse robots to self-driving cars, machines must be able to perceive their environment, interpret its structure, and take corrective action in real time — all without relying on GPS, pre-built maps, or human guidance.

Line following is one of the oldest and most instructive versions of this problem. A robot equipped with nothing but a camera must continuously answer a simple but unforgiving question: *am I still on the line — and if not, how do I get back?*

Doing this reliably requires more than a single frame of image processing. The robot must extract a clean signal from noisy pixels, convert that signal into a meaningful error term, correct for it smoothly enough to stay stable at speed, recover gracefully when the line disappears from view, and recognize when its mission — a completed lap — is actually done.

Before any of this can be trusted on a real robot, engineers need a simulation environment that faithfully reproduces camera perception, differential-drive dynamics, and track geometry — so that the vision pipeline and control logic can be tested safely and repeatedly.

The objective of this project is to build a complete autonomous line-following system that combines OpenCV-based perception, proportional error control, failure recovery, and lap/marker tracking to complete a full autonomous lap in a realistic ROS 2 + Gazebo simulation environment.

---

## The Story

Somewhere inside a quiet simulated world, a robot opens its eyes for the first time.

It doesn't know where it is. It doesn't know where it's going.

There is no map, no GPS, no human guidance. It has been placed on a track it has never seen.

All it has is one sense: a single forward-facing camera.

At first, everything is just noise — a blur of color and shadow. But gradually, a pattern emerges: a line on the ground, curving ahead into the unknown.

That line becomes its only guide.

The robot rolls forward, watching the line drift slightly left, then right, correcting itself frame by frame. When the line briefly vanishes under glare or around a sharp turn, it doesn't stop — it remembers where the line last was, slows down, and searches until it finds its way back.

Markers along the track tell it how far it has come. Eventually, the start line reappears beneath it, and the robot understands: the lap is complete.

It comes to a smooth, controlled stop.

No human ever touched the controls.

No pre-programmed path was given.

The only input was a single camera feed — everything else was accomplished through perception, feedback control, and recovery logic, frame after frame.

This project is not just about following a line — it is about building the smallest complete loop of autonomy: **see, understand, act, recover, improve.** The same feedback principle scales from a toy track all the way up to warehouse AGVs and self-driving vehicles.

---

## Objective

Develop a complete ROS 2 software stack capable of autonomously detecting, following, and completing a lap around a track using only a forward-facing camera — inside a realistic Gazebo simulation.

Overall pipeline:

```
Camera Input → ROI Crop → Color Threshold → Contour Detection → Centroid
   → Error Computation (center_of_image - center_of_line)
   → Control Output (angular.z = -Kp * error, linear.x = constant speed)
   → Robot Motion → Updated Camera Frame
   → [Line Lost? → Recovery: last-known error → amplified correction → reduced speed → re-search]
   → Marker Detection → Lap Counting → Completion Check → Safe Shutdown
```
---

## System Overview

The project consists of three major components.

### 1. Robot (SDF Model)
A simulated differential-drive robot in Gazebo, defined by `custom_turtlebot3.sdf`. It provides a differential-drive base, an RGB camera, physics-based motion, and a diff-drive plugin for velocity control — establishing the robot's physical behavior in simulation, from how it moves to what it sees.

### 2. World & Environment
A Gazebo world file (`world.sdf`) defining the track layout, the robot's spawn position, and its initial yaw orientation — controlling whether the robot can actually see the track properly the moment it wakes up.

### 3. Follower Node (Brain)
A ROS 2 Python node (`follower_node.py`) responsible for reading camera images, detecting the track via OpenCV, computing the error from center, generating motion commands, and handling both recovery and mission-completion logic. This is the perceptual and decision-making core of the robot.

---

## What You Need To Implement

This repository contains several TODOs distributed across four files. Complete these implementations to obtain a fully autonomous, recovery-capable line-following robot.

### 1. Robot Model — `follower/models/custom_turtlebot3/model.sdf`

**TODO 1 — Physics Tuning**
Adjust wheel radius, wheel separation, friction, and contact stability so the robot drives predictably without slipping or oscillating.

**TODO 2 — Camera Configuration**
Tune camera position, tilt, field of view, update rate, and noise model so the track is visible and trackable under simulated sensor noise.

**TODO 3 — Control Plugin**
Configure the diff-drive plugin parameters to match the tuned physical model.

### 2. World Definition — `follower/worlds/new_track.sdf` & `follower/worlds/sor_track.sdf` 

**TODO 1 — Spawn Placement**
Set the robot's spawn position (x, y) and initial yaw orientation so it starts correctly aligned with the track. This directly determines whether the robot can even see the line at startup.

### 3. Follower Node — `follower/follower/follower_node.py`

**TODO 1 — Contour Classification & Centroid Extraction**
Tell the track line apart from a lap marker by contour size, and extract each one's centroid, correctly mapped back to the full frame.

**TODO 2 — Tracking Error & Line-Loss Handling**
Compute error from the line's offset to image center. When the line disappears, amplify the last known error and hold forward motion until reacquired.

**TODO 3 — Lap Completion Detection**
Detect a completed lap from marker crossings, debounced and only while centered, then trigger the finalization countdown.

**TODO 4 — Proportional Steering & Command Gating**
Convert error into a corrective angular velocity, and only publish movement while the follower is active.

### 4. Build Configuration — `follower/setup.py`

**TODO 1 — Register the Console Entry Point**
Add the console script entry:

```
follower = follower.<your_file>:main
```

Without this, `ros2 run` will not work and the node will not start.

---

## Perception & Control Details

**Perception Pipeline**

```
Raw Image → ROI → Color Threshold → Contours → Centroid → Error
```

**Control Strategy**

```
error = center_of_image - center_of_line
angular.z = -Kp * error
linear.x = constant speed
```

**Failure Handling** — when the line is lost:

1. Use the last known error
2. Amplify the correction
3. Reduce forward motion
4. Search for the track again

**Mission Completion**

* Detect markers
* Track laps
* Confirm completion
* Execute safe shutdown

---

## Running the Project

### Prerequisites

**Install TurtleBot3 Packages**

```bash
sudo apt install -y ros-jazzy-turtlebot3
sudo apt install -y ros-jazzy-turtlebot3-simulations
sudo apt install -y ros-jazzy-turtlebot3-msgs
```

**Install cv_bridge and Image Transport**

```bash
sudo apt install -y ros-jazzy-cv-bridge
sudo apt install -y ros-jazzy-image-transport
sudo apt install -y ros-jazzy-image-transport-plugins
```

**Install Python Dependencies**

```bash
# OpenCV for Python
pip install opencv-python

# NumPy
pip install "numpy<2"

# colcon build tool
sudo apt install -y python3-colcon-common-extensions
```

**Build the workspace:**

```bash
cd ~/ros2_line_follower
colcon build
```

For every new terminal, source the workspace:

```bash
source install/setup.bash
```

### Step 1 — Launch the Simulation

Open a terminal and launch Gazebo with the track world and the spawned robot. 
```bash
ros2 launch follower new_track.launch.py
```
Wait until the world fully loads and the robot model appears before running Step 2.

### Step 2 — Run the Follower Node

Open a new terminal (workspace already sourced) and run:

```bash
ros2 run follower follower
```

### Step 2 — start the service
```bash
ros2 service call /start_follower std_srvs/srv/Empty
```

The robot will begin reading camera frames, detecting the track, and autonomously following the line — correcting for drift, recovering from lost track detection, counting laps via markers, and stopping safely once the lap is complete.

> You can also launch with the `sor_track.launch.py` 

## Bonus Challenges

Once you've completed all the required TODOs and the robot can reliably complete a full autonomous lap, try extending the project with the following challenges.

### Bonus Challenge 1 — PID Control Upgrade

**Objective:** Currently, the control strategy is a simple proportional (P) controller. Upgrade it to a full PID controller for smoother, more stable tracking at higher speeds.

**What to implement:**
- Add integral and derivative terms to the error computation
- Tune gains for stability across straightaways and sharp turns
- Compare lap times and stability against the P-only baseline

### Bonus Challenge 2 — Live Telemetry Dashboard & Adaptive Speed

**Objective:** Currently, the robot runs at a constant forward speed and provides no visibility into its internal state. Add real-time telemetry and adaptive speed control.

**What to implement:**
- Publish/display live error, speed, and lap-count telemetry (e.g. via a dashboard or RViz overlay)
- Adjust forward speed dynamically based on curvature or error magnitude — slowing through turns, accelerating on straights
- Support a custom, more complex track design to stress-test the adaptive behavior

## Deliverables

### 1. Source Code
- Completed implementations for all TODOs
- Functional ROS 2 package
- Updated launch, world, and SDF files

### 2. Demonstration Video
Show:
- Startup and track detection
- Steady-state line tracking
- Recovery from lost track
- A complete autonomous lap
- Safe shutdown

### 3. Report
Briefly describe:
- Vision pipeline design (ROI, thresholding, contour/centroid extraction)
- Control strategy and tuning process
- Recovery logic design
- Lap/marker detection and completion logic
- Challenges encountered

## Final Message

A line-following robot can't rely on a map, a GPS fix, or a human at the wheel. Every successful lap depends on the seamless integration of clean perception, stable feedback control, and graceful recovery when things go wrong.

By completing this project, you will implement each stage of that loop — from raw pixels to a centroid, from a centroid to an error term, from an error term to a motion command — and gain practical experience with the same perception-and-control principles that scale from a toy track to real-world autonomous vehicles.

The objective is not simply to follow a line drawn on the ground, but to understand how a robot turns a single noisy camera feed into a continuous, self-correcting loop of perception and action: **see, understand, act, recover, improve.**
