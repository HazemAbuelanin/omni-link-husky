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

To get started with the omnilink bridge: 

Phase 1: Environment Setup (The Backend)

This phase establishes the local server and the necessary communication protocols.

1. Start the Husky Fleet Simulator (Flask API)

    Navigate to the root directory of the repository (omni-link-husky-fleet/).

    Run the simulator:
    Bash

    python husky_fleet.py

    (Terminal 1): This terminal must remain open. The Flask API runs on http://127.0.0.1:5000 and defaults to controlling husky_0.

2. Configure and Start the MQTT Broker (Mosquitto)

The OmniLink Agent UI requires the WebSockets protocol on port 9001.

    Stop the Default Service: Ensure no background instance is blocking the port.
    Bash

sudo systemctl stop mosquitto.service

Enable WebSockets: Edit the main Mosquitto configuration file.
Bash

sudo nano /etc/mosquitto/mosquitto.conf

Add/Verify Configuration: Ensure the following lines are present to enable the required protocol and port, and to prevent the authentication error (rc=5).
Code snippet

allow_anonymous true # CRITICAL: Allows connection from the local bridge client
listener 9001
protocol websockets # CRITICAL: Required to match the OmniLink UI setting (ws://...)

Start the Broker Service:
Bash

    sudo systemctl start mosquitto.service

    (Terminal 2): The broker is now listening correctly.

3. Launch the OmniLink Bridge Script

The Python script receives MQTT messages and translates them into Flask API calls for husky_0.

    Set Environment Variables:
    Bash

export HUSKY_API_URL=http://127.0.0.1:5000
export HUSKY_ROBOT_ID=husky_0 # Set the single robot target

Navigate to Bridge Directory:
Bash

cd robot_link/

Run the Bridge:
Bash

    python link_mqtt.py

    (Terminal 3): A successful connection will show: [OmniLinkMQTT] Connected localhost:9001 (transport=websockets).

Phase 2: OmniLink Agent Configuration (Revised Templates)

In the OmniLink UI website, you must use the simplified templates provided below.

1. Configure Connection Settings

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

