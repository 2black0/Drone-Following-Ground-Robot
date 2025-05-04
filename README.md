# 🛰️ Drone Following Ground Robot - Webots R2025a

This simulation project demonstrates how a drone (quadcopter) can autonomously follow a ground robot using GPS and IMU-based relative positioning in the Webots R2025a environment. It features keyboard-controlled movements, real-time data exchange via emitter/receiver, PID-based drone stabilization, and optional ArUco tracking.

## 📁 Project Structure

```

.
├── controllers
│   ├── data-logger              # Tools for plotting error and analyzing logs
│   ├── ground_controller        # Sends position & yaw to the drone via emitter
│   └── mavic_controller         # Main drone controller (keyboard + follow logic)
├── protos                       # Custom Mavic2Pro 3D model for Webots
├── worlds                       # Webots simulation worlds (4-wheeled robot + drone)
└── README.md

```

## 🚦 Workflow Overview

### 🛻 Ground Robot (`ground_controller`)
- Controlled via keyboard:
  - `1` forward, `2` left turn, `3` right turn, `4` stop.
- Publishes its yaw, position (x, y), and altitude via an `Emitter`.
- Data is logged into `logger_mobile.csv`.

### 🚁 Drone (`mavic_controller`)
- Receives position & yaw from the ground robot using a `Receiver`.
- PID-controlled altitude, yaw, roll, and pitch.
- Supports multiple modes:
  - `T`: Takeoff
  - `L`: Land
  - `G`: Toggle gimbal stabilization
  - `WASD`: Move manually
  - `Shift + WASD`: Change altitude/yaw
  - `H`: Go Home
  - `F`: Enable ArUco-based tracking
  - `P`: Enable follow mode based on GPS from ground robot
  - `R`: Return to Home with altitude preset
- Data is logged into `logger_drone.csv`.

### 📈 Logging and Visualization
- CSV logs are stored in the `controllers/data-logger/` folder.
- `plotting.py` and `plot-error.py` scripts help visualize drone and ground robot movement and error trajectories.

## 🌍 World Files
Located in `worlds/` folder:
- `drone_world.wbt`: Main simulation environment.
- `4_wheels_robot_rev*.wbt`: Variants of ground robot environment setup.

## 🔧 Requirements
- Webots R2025a
- Python dependencies: `numpy`, `opencv-python`, `simple-pid`

## 🚀 Getting Started

### 1. Launch Webots
```bash
webots worlds/drone_world.wbt
```

### 2. Run Controllers

From Webots, assign the following:

* Ground robot → `ground_controller`
* Drone (Mavic2Pro) → `mavic_controller`

### 3. Monitor Output

Check terminal for status messages and use keys for control. Logs will be generated automatically.

## 📌 Notes

* The folder `mavic2_controller` is deprecated and used for testing only.
* This simulation does not require ROS and is fully contained within Webots.

## 📜 License

This project is licensed under the MIT License.

---