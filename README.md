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

To get started, clone the repository:
git clone <repo_link>

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

## 🔗 OmniLink Bridge Setup (MQTT → Simulator Control)

This section explains how to connect the **OmniLink Agent UI** (web UI) with the **Husky Fleet Simulator** by using **MQTT**. The OmniLink UI communicates over **WebSockets**, so we must configure Mosquitto to support `protocol websockets` on port **9001**.

---

### 1) Configure & Start the Mosquitto Broker

The default Mosquitto broker listens on `1883` (TCP only), which **will not work** with the OmniLink UI.  
We must enable **WebSockets** on port **9001**.

#### Stop any running Mosquitto instance:
```bash
sudo systemctl stop mosquitto.service
```
Edit the Mosquitto configuration:
```bash
sudo nano /etc/mosquitto/mosquitto.conf
```
Add (or verify) these lines:
```bash
allow_anonymous true      # Required for local UI connections (avoid rc=5 auth errors)
listener 9001
protocol websockets       # Enables ws:// communication required by the OmniLink UI
```

Note: Using allow_anonymous true is acceptable for local development.
For production use, configure authentication.

Restart the broker:

```bash
sudo systemctl start mosquitto.service
```

✅ Verification:

Run in another terminal:
```bash
sudo netstat -tulpn | grep 9001
```
You should see Mosquitto listening on port 9001 (websockets).

2) Launch the OmniLink Bridge

This script receives messages from the OmniLink UI via MQTT and converts them into REST API calls to control the robots.

```bash
export HUSKY_API_URL=http://127.0.0.1:5000     # Address of the simulator REST API
```
Run the bridge:
```bash
cd robot_link/
python link_mqtt.py
```
Expected output:
```bash
[OmniLinkMQTT] Connected to localhost:9001 (transport=websockets)
```
Leave this terminal running.
It listens for control messages.

In the OmniLink UI, navigate to the Connection Settings and verify the following fields:

    BROKER/WEBSOCKET URL: ws://localhost:9001

    COMMAND TOPIC: olink/commands

2. Define Command Templates

Go to Settings → Commands and enter only these five templates. The simplified structure guarantees an exact match with the Python code, as the variables ([number]) are separated by underscores or the unit shorthand (m/s, rad/s).
Action	Single-Robot Template
Move Forward	move_forward_at_[number]_m/s_for_[number]_seconds
Move Backward	move_backward_at_[number]_m/s_for_[number]_seconds
Turn Right	turn_right_at_[number]_rad/s_for_[number]_seconds
Turn Left	turn_left_at_[number]_rad/s_for_[number]_seconds
Stop	stop

3. Test the End-to-End Control Loop

    In the OmniLink UI (voice or text), issue a command that forces the LLM to use the template structure:

        "Move forward at 0.5 m/s for 3 seconds." (The Agent should generate the string: move_forward_at_0.5_m/s_for_3_seconds)

    Verification: The robot will move and the bridge terminal will log the successful event match.
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

