# DifferentialBot RRT Simulator

A 2D simulation of a **differential-drive robot** performing autonomous navigation using the **Rapidly-exploring Random Tree (RRT)** path planning algorithm, built with **PyGame** and **Pymunk**.

The robot explores a hallway-style map, detects obstacles via an ultrasonic sensor, builds a live occupancy map, and follows the RRT-computed path using a heading-based PID controller — all fused through a Kalman Filter.

---

## Screenshots

<p align="center">
  <img src="Screenshot 2026-02-05 192552.png" width="48%" alt="Simulation mid-navigation"/>
  &nbsp;
  <img src="Screenshot 2026-02-05 192653.png" width="48%" alt="Robot reaching goal"/>
</p>

> **Left:** Robot mid-navigation following the RRT path (cyan dots) through a hallway map with obstacles (orange). The sidebar shows real-time telemetry and a minimap.  
> **Right:** Robot arrives near the goal (green square) after successfully navigating the environment.

---

## Features

- **RRT Path Planning** — Rapidly-exploring Random Tree generates a collision-free path from start to goal on-the-fly
- **Differential Drive Physics** — Realistic kinematic model using Pymunk's rigid body simulation
- **Sensor Fusion** — GPS and IMU data fused through a **Kalman Filter** for smooth position estimation
- **Ultrasonic Sensor** — Cone-shaped proximity detection triggers obstacle avoidance and replanning
- **Live Occupancy Mapping** — The robot dynamically records detected obstacle positions
- **PID Heading Controller** — Steers the robot toward each RRT waypoint by adjusting individual wheel speeds
- **Dark Sci-Fi UI** — Real-time sidebar with minimap, robot state telemetry, and Pause/Resume button
- **Embedded hardware stub** — `WheelMotorDrivers` and `UARTReceiver` include commented-out MicroPython (Raspberry Pi Pico) code for real deployment

---

## Project Structure

```
Codes/
├── main.py              # Entry point: simulation loop, rendering, UI
├── RRT_Search.py        # RRT algorithm (node tree, steering, path filtering)
├── Control.py           # High-level controller: path following + replanning logic
├── PIDController.py     # Heading-based PID that adjusts left/right wheel speeds
├── KalmanFilter.py      # 4-state Kalman Filter (x, y, vx, vy) fusing GPS + IMU
├── SensorModules.py     # GPS, IMU and Ultrasonic sensor simulation classes
├── Mapping.py           # Occupancy map builder and path comparison utilities
├── Obstacles.py         # Static obstacle definitions (hallway layout + alternatives)
├── Lines.py             # Trail renderer (GPS trace, Kalman trace, RRT path)
├── WheelMotorDrivers.py # Motor command layer (simulation + Pico GPIO stubs)
└── UARTReceiver.py      # Sensor data aggregator (simulation + UART stubs)
```

---

## Architecture Overview

```
┌──────────────┐     GPS + IMU     ┌─────────────────┐
│ SensorModules│ ────────────────► │  KalmanFilter   │
│  GPS / IMU   │                   │  (x,y,vx,vy)    │
│  Ultrasonic  │                   └────────┬────────┘
└──────┬───────┘                            │ fused state
       │ distance to obstacle               ▼
       │                          ┌──────────────────┐
       └─────────────────────────►│    Controller    │
                                  │  (Control.py)    │
                                  │                  │
                                  │  ┌────────────┐  │
                                  │  │ RRT Search │  │
                                  │  └────────────┘  │
                                  │  ┌────────────┐  │
                                  │  │    PID     │  │
                                  │  └────────────┘  │
                                  └────────┬─────────┘
                                           │ wheel speeds
                                           ▼
                                  ┌──────────────────┐
                                  │ WheelMotorDrivers│
                                  │  Bot (Pymunk)    │
                                  └──────────────────┘
```

**Data flow each frame:**
1. `UARTReceiver` pulls fresh GPS coordinates, IMU angle, and ultrasonic distance
2. `KalmanFilter` predicts & updates the state estimate
3. `Controller` checks for obstacle proximity → triggers backward motion + replanning if needed
4. `RRTPatchSearch` generates a new tree from current position to goal when needed
5. `PIDController` computes left/right wheel speeds to steer toward the next waypoint
6. `WheelMotorDrivers` applies speeds to the Pymunk physics body
7. `Mapping` records new obstacle detections into the occupancy map

---

## Requirements

- Python **3.10+**
- [pygame](https://www.pygame.org/) — rendering and event loop
- [pymunk](http://www.pymunk.org/) — 2D rigid body physics
- [numpy](https://numpy.org/) — matrix math for the Kalman Filter

Install all dependencies:

```bash
pip install pygame pymunk numpy
```

---

## Running the Simulation

```bash
cd Codes
python main.py
```

| Control | Action |
|---|---|
| `PAUSE SIMULATION` button | Pause / Resume the physics and control loop |
| Close window | Exit the simulation |

---

## Configuration

Key parameters are defined in `main.py` and the module constructors:

| Parameter | Location | Default | Description |
|---|---|---|---|
| `destination` | `main.py` | `(61, 37)` | Goal position in grid units |
| `tile_size` | `main.py` | `10` px | Grid cell size |
| `MAP_WIDTH / HEIGHT` | `main.py` | `800 × 600` | Simulation canvas size |
| `max_iterations` | `RRT_Search.py` | `3000` | Max RRT tree expansion steps |
| `distance_at_node` | `Control.py` | `2` | RRT step size in grid units |
| `process_variance` | `KalmanFilter.py` | `1e-4` | Kalman process noise |
| `measurement_variance_gps/imu` | `KalmanFilter.py` | `1e-2` | Sensor noise covariance |
| Obstacle layout | `Obstacles.py` | `hallway_obstacles` | Swap for `obstacles1`, `obstacles2`, `obstacles3` |

---

## Map Layouts

Three alternative obstacle layouts are available in `Obstacles.py`:

| Function | Description |
|---|---|
| `hallway_obstacles` *(default)* | Hallway map with horizontal/vertical walls and multiple doorway gaps |
| `obstacles1` | L-shaped corridors with horizontal barriers |
| `obstacles2` | Empty map (no obstacles) |
| `obstacles3` | Sparse walls — good for testing open-field RRT |

Switch layouts in `main.py`:
```python
obstacles = Obstacles.hallway_obstacles(tile_size)  # Change this line
```

---

## Hardware Deployment (Raspberry Pi Pico)

The codebase is structured for portability. Both `WheelMotorDrivers.py` and `UARTReceiver.py` contain commented-out **MicroPython** code for Raspberry Pi Pico GPIO control (L298N motor driver via PWM pins).

To deploy:
1. Uncomment the `from machine import Pin` block and GPIO logic in `WheelMotorDrivers.py`
2. Implement real UART read logic in `UARTReceiver.py`'s `refresh_data()` method
3. Replace the sensor simulation calls with actual hardware reads

---

## Algorithm Details

### RRT (Rapidly-exploring Random Tree)
1. Initialize tree with robot's current grid position
2. Sample a random node in the map space
3. Find the nearest existing tree node
4. Steer toward the random node by `distance_at_node` steps
5. If the new node is collision-free (not in `Map.obstacles_coordinates`), add it
6. If within 2 grid units of the goal, terminate and extract the path
7. Path is extracted by traversing parent pointers from the goal-nearest node back to root

### Kalman Filter
State vector: `[x, y, vx, vy]`  
- **Predict:** propagates state with constant-velocity model  
- **Update:** fuses GPS (position) and IMU (velocity) measurements  
- Separate measurement variances allow tuning sensor trust

### PID Heading Controller
- Computes `atan2` angle toward the next waypoint
- If heading error `> π/15` rad: spin left (turn right)
- If heading error `< -π/15` rad: spin right (turn left)
- Otherwise: drive forward at full speed

---

## License

This project is open source. Feel free to use, modify, and build upon it.

---

*Built with PyGame + Pymunk · RRT Path Planning · Kalman Filtering · Differential Drive Kinematics*
