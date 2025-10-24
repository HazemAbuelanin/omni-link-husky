# Omni Link Husky Simulation

This project provides a lightweight Python application for driving a Clearpath Husky robot in a PyBullet simulation while exposing simple REST endpoints over Flask. It is useful for experimenting with differential drive control, prototyping motion behaviors, and integrating with other systems that can send HTTP commands. Everything runs in a single Python process, so it is easy to understand, extend, and debug without needing a ROS stack.

## Features

- **Realtime physics** powered by PyBullet running at 240 Hz.
- **REST API control** for commanding linear and angular velocity with optional duration.
- **Convenience helpers** for common motions such as forward, backward, and turning.
- **Live pose telemetry** exposed through the `/pose` endpoint.
- **Simple reset & stop endpoints** for quickly recovering from tests.

## Requirements

- Python 3.9+
- [PyBullet](https://pybullet.org)
- [Flask](https://flask.palletsprojects.com/)

Install the core simulator dependencies with pip:

```bash
pip install pybullet flask
```

The optional Omni Link bridges under `robot_link/` also require:

```bash
pip install requests paho-mqtt
```

> **Note:** PyBullet opens a GUI window by default. On headless systems you may need to use a virtual display such as Xvfb.

## Running the Simulator

1. (Optional) Create and activate a virtual environment.

   ```bash
   python -m venv .venv
   source .venv/bin/activate
   ```

2. Launch the simulation and REST server:

   ```bash
   python husky_drive.py
   ```

   To try the obstacle-course variant, run:

   ```bash
   python husky_obstacle_course.py
   ```

3. A PyBullet window will open showing the Husky model (and, if you launched `husky_obstacle_course.py`, a hand-built driving course). The Flask server listens on `http://127.0.0.1:5000`. Update the `app.run()` call in the script if you need to expose the API on a different interface. Use the sliders in the **Params** tab to move the camera and change the robot's viewpoint while you experiment.

4. Send HTTP requests to control the robot. You can use `curl`, `httpie`, Postman, or any HTTP-capable client.

For headless environments, start an X virtual framebuffer (e.g., `xvfb-run python husky_drive.py`) or modify `setup_world()` to use `p.DIRECT` instead of `p.GUI`.

## REST API

| Method & Endpoint | Description | Example Payload |
| ----------------- | ----------- | --------------- |
| `GET /health`     | Returns `{ "ok": true }` for quick liveness checks. | _None_ |
| `GET /pose`       | Returns the latest pose estimate in meters/radians. | _None_ |
| `POST /drive`     | Command linear (`vx`) and angular (`wz`) velocity with an optional duration. Positive `vx` drives forward; positive `wz` rotates counter-clockwise. | `{ "vx": 0.6, "wz": 0.0, "duration": 2.0 }` |
| `POST /stop`      | Immediately zeroes all velocity commands. | _None_ |
| `POST /reset`     | Resets the Husky pose/velocity and clears the current command. | _None_ |
| `POST /forward`   | Convenience wrapper for driving forward (`speed` in m/s). | `{ "speed": 0.5, "duration": 1.0 }` |
| `POST /backward`  | Convenience wrapper for reversing. | `{ "speed": 0.4, "duration": 1.0 }` |
| `POST /turn_left` | Convenience wrapper that rotates counter-clockwise (`rate` in rad/s). | `{ "rate": 1.2, "duration": 0.8 }` |
| `POST /turn_right`| Convenience wrapper that rotates clockwise (`rate` in rad/s). | `{ "rate": 1.2, "duration": 0.8 }` |

Example `curl` request:

```bash
curl -X POST http://127.0.0.1:5000/drive \
  -H "Content-Type: application/json" \
  -d '{"vx": 0.6, "wz": 0.0, "duration": 2.0}'
```

## Configuration

Key simulation constants are defined at the top of both simulator entry points (e.g., `husky_drive.py` and `husky_obstacle_course.py`), including wheel geometry, acceleration limits, damping factors, and obstacle layout. Adjust them to tune vehicle behavior or test different dynamics.

## Omni Link Agent Configuration

- **Main Task:** You are an agent who controls the movements of a mobile robot.
- **Available Commands:**
  - `move_forward_at_[number]_m/s_for_[number]_seconds`
  - `move_backward_at_[number]_m/s_for_[number]_seconds`
  - `turn_right_at_[number]_rad/s_for_[number]_seconds`
  - `turn_left_at_[number]_rad/s_for_[number]_seconds`
  - `stop`
- **User Name:** none
- **Agent Name:** Husky
- **Agent Personality:** friendly, professional
- **Custom Instructions:** none

## Project Structure

```
.
├── README.md                  # Project overview and usage
├── husky_drive.py             # Main simulation + REST server script
├── husky_obstacle_course.py   # Alternate entry point with a PyBullet obstacle course
└── robot_link/                # Omni Link bridges (MQTT, TCP, remote polling) and helpers
```

The `robot_link` helpers translate Omni Link natural-language commands into REST calls against the simulator. Start `robot_link/link_mqtt.py`, `robot_link/link_tcp.py`, or `robot_link/link_remote.py` after configuring the appropriate connection details to bridge those commands into the running Husky instance.

## Troubleshooting

- **No GUI appears:** Ensure you are running in an environment with an available display or configure PyBullet for headless rendering (see the headless note above).
- **Robot does not move:** Check that commands are within the clamped limits (`MAX_VX`, `MAX_WZ`) and that the `/drive` endpoint is receiving float values. The JSON body must contain numbers, not strings.
- **Camera controls are awkward:** Use the on-screen sliders in the PyBullet UI to adjust yaw and pitch, or change the default values at the top of `husky_drive.py`.
- **Need to restart:** Use the `/reset` endpoint or restart the script to clear any unexpected state.

