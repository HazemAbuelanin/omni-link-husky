#!/usr/bin/env python3
import time
import threading
from typing import Tuple
from flask import Flask, request

import pybullet as p
import pybullet_data

# ---------------- Simulation Tunables ----------------
HZ = 240                         # physics rate
DT = 1.0 / HZ
WHEEL_RADIUS = 0.095             # Husky wheels ~9.5 cm
WHEEL_BASE = 0.55                # left-right distance
MAX_VX = 1.2                     # m/s safety clamp
MAX_WZ = 2.0                     # rad/s safety clamp
ACCEL_VX = 2.5                   # m/s^2 accel ramp
ACCEL_WZ = 5.0                   # rad/s^2 accel ramp
MOTOR_TORQUE = 24.0              # Nm per wheel
LINEAR_DAMP = 0.05               # natural decay
ANGULAR_DAMP = 0.08

BOX_SCALE = 4.0                  # obstacle size
BOX_MASS = 25.0

# ---------------- Shared Command State ----------------
state_lock = threading.Lock()
cmd_vx = 0.0         # desired linear velocity (m/s)
cmd_wz = 0.0         # desired angular velocity (rad/s)
cmd_until = None     # epoch seconds; if set, command auto-zeros after this time

# For telemetry
_last_pose = (0.0, 0.0, 0.0)  # x, y, yaw
_running = True

# Camera control state
camera_distance = 3.0
camera_yaw = 35.0
camera_pitch = -25.0
camera_yaw_slider_id = None
camera_pitch_slider_id = None

# Debug overlay handles
odometry_text_id = None

# ---------------- PyBullet Setup ----------------
def setup_world():
    global camera_yaw_slider_id, camera_pitch_slider_id
    p.connect(p.GUI)
    # Hide all side menus/panels
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Optional: hide preview panes and other UI clutter
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    # Optional: cleaner look (toggle as you like)
    p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)  # or 0 to disable shadows

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=[0, 0, 0],
    )

    camera_yaw_slider_id = p.addUserDebugParameter(
        "Camera Yaw", -180.0, 180.0, camera_yaw
    )
    camera_pitch_slider_id = p.addUserDebugParameter(
        "Camera Pitch", -89.0, 45.0, camera_pitch
    )

    plane_id = p.loadURDF("plane.urdf")

    start_pos = [0, 0, 0.1]
    start_orn = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF("husky/husky.urdf", start_pos, start_orn, useFixedBase=False)

    # Obstacle
    box_pos = [2.0, 0.0, 0.5 * BOX_SCALE * 0.05]
    box_orn = p.getQuaternionFromEuler([0, 0, 0])
    box_id = p.loadURDF("cube_small.urdf", box_pos, box_orn, globalScaling=BOX_SCALE)
    p.changeDynamics(box_id, -1, mass=BOX_MASS, lateralFriction=0.9, rollingFriction=0.01, spinningFriction=0.01)

    # Map wheel joints by name
    wheel_joint_names = {
        "front_left_wheel": None, "rear_left_wheel": None,
        "front_right_wheel": None, "rear_right_wheel": None
    }
    for j in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, j)[1].decode()
        if name in wheel_joint_names:
            wheel_joint_names[name] = j

    wheels_left = [wheel_joint_names.get("front_left_wheel"), wheel_joint_names.get("rear_left_wheel")]
    wheels_right = [wheel_joint_names.get("front_right_wheel"), wheel_joint_names.get("rear_right_wheel")]

    # Fallback if names missing
    if any(j is None for j in wheels_left + wheels_right):
        wheels_left, wheels_right = [2, 4], [3, 5]

    # Disable default position motors
    for j in wheels_left + wheels_right:
        p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)

    return robot_id, wheels_left, wheels_right

def reset_robot(robot_id):
    p.resetBasePositionAndOrientation(robot_id, [0,0,0.1], p.getQuaternionFromEuler([0,0,0]))
    p.resetBaseVelocity(robot_id, [0,0,0], [0,0,0])

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def smooth_towards(curr, target, accel, dt):
    if target > curr:
        return min(target, curr + accel * dt)
    return max(target, curr - accel * dt)

def diff_to_wheels(vx, wz) -> Tuple[float, float]:
    v_l = vx - (wz * WHEEL_BASE / 2.0)
    v_r = vx + (wz * WHEEL_BASE / 2.0)
    wl = v_l / WHEEL_RADIUS  # rad/s
    wr = v_r / WHEEL_RADIUS
    return wl, wr

def get_pose(robot_id):
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    eul = p.getEulerFromQuaternion(orn)
    return (pos[0], pos[1], eul[2])  # x,y,yaw

# ---------------- Physics Thread ----------------
def physics_loop():
    global _last_pose, odometry_text_id
    global camera_yaw, camera_pitch, camera_distance, camera_yaw_slider_id, camera_pitch_slider_id

    robot_id, wheels_left, wheels_right = setup_world()

    vx = 0.0
    wz = 0.0

    while _running:
        now = time.time()
        # Handle command timeout
        with state_lock:
            desired_vx = cmd_vx
            desired_wz = cmd_wz
            if cmd_until is not None and now > cmd_until:
                desired_vx = 0.0
                desired_wz = 0.0

        # Clamp user command
        desired_vx = clamp(desired_vx, -MAX_VX, MAX_VX)
        desired_wz = clamp(desired_wz, -MAX_WZ, MAX_WZ)

        # Smooth accel
        vx = smooth_towards(vx, desired_vx, ACCEL_VX, DT)
        wz = smooth_towards(wz, desired_wz, ACCEL_WZ, DT)

        # Natural decay when near zero command
        if abs(desired_vx) < 1e-4: vx *= (1.0 - LINEAR_DAMP)
        if abs(desired_wz) < 1e-4: wz *= (1.0 - ANGULAR_DAMP)

        # Convert to wheel speeds and apply
        wl, wr = diff_to_wheels(vx, wz)
        for j in wheels_left:
            p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, targetVelocity=wl, force=MOTOR_TORQUE)
        for j in wheels_right:
            p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, targetVelocity=wr, force=MOTOR_TORQUE)

        # Camera follow with slider-based orbit control
        if camera_yaw_slider_id is not None:
            camera_yaw = p.readUserDebugParameter(camera_yaw_slider_id)
        if camera_pitch_slider_id is not None:
            camera_pitch = p.readUserDebugParameter(camera_pitch_slider_id)

        pos, _ = p.getBasePositionAndOrientation(robot_id)
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=pos,
        )

        # Step and record pose
        p.stepSimulation()
        _last_pose = get_pose(robot_id)

        # Update GUI odometry readout anchored above the robot.  Using
        # ``replaceItemUniqueId`` avoids visible flicker that occurred when the
        # previous text was removed every frame.
        odometry_text = "X: {:.2f}  Y: {:.2f}  Yaw: {:.2f} rad".format(*_last_pose)
        text_position = [0.0, 0.0, 0.8]
        debug_text_kwargs = dict(
            textColorRGB=[0.0, 0.0, 0.0],
            textSize=1.8,
            lifeTime=0,
            parentObjectUniqueId=robot_id,
            parentLinkIndex=-1,
        )
        if odometry_text_id is None:
            odometry_text_id = p.addUserDebugText(
                odometry_text,
                text_position,
                **debug_text_kwargs,
            )
        else:
            odometry_text_id = p.addUserDebugText(
                odometry_text,
                text_position,
                replaceItemUniqueId=odometry_text_id,
                **debug_text_kwargs,
            )
        time.sleep(DT)

    p.disconnect()

# ---------------- Flask API ----------------
app = Flask(__name__)

@app.get("/health")
def health():
    return {"ok": True}

@app.get("/pose")
def pose():
    x, y, yaw = _last_pose
    return {"x": x, "y": y, "yaw": yaw}

@app.post("/reset")
def api_reset():
    # Do a synchronous reset in sim thread by posting a flag via velocity + quick hack:
    # We can safely call PyBullet here for simplicity since GUI is single-process; quick lock avoids races.
    p.stepSimulation()  # nudge
    reset_robot(0)      # WRONG id if used—so do it properly:
    # Proper: use base reset directly on current bodyUniqueId (0 is plane). Find Husky via bodies list.
    # Minimal safe approach: search by name once.
    bodies = [p.getBodyUniqueId(i) for i in range(p.getNumBodies())]
    husky_id = None
    for b in bodies:
        if "husky" in p.getBodyInfo(b)[1].decode().lower():
            husky_id = b
            break
    if husky_id is not None:
        reset_robot(husky_id)
    with state_lock:
        global cmd_vx, cmd_wz, cmd_until
        cmd_vx = 0.0
        cmd_wz = 0.0
        cmd_until = None
    return {"reset": True}

@app.post("/stop")
def stop():
    with state_lock:
        global cmd_vx, cmd_wz, cmd_until
        cmd_vx = 0.0
        cmd_wz = 0.0
        cmd_until = None
    return {"stopped": True}

@app.post("/drive")
def drive():
    """
    JSON body:
    {
      "vx": 0.5,          # m/s (forward +)
      "wz": 0.8,          # rad/s (CCW +)
      "duration": 1.5     # seconds (optional; if set, auto-stop after)
    }
    """
    data = request.get_json(force=True, silent=True) or {}
    vx = float(data.get("vx", 0.0))
    wz = float(data.get("wz", 0.0))
    duration = data.get("duration", None)
    until = None
    if duration is not None:
        try:
            duration = float(duration)
            if duration > 0:
                until = time.time() + duration
        except Exception:
            duration = None

    with state_lock:
        global cmd_vx, cmd_wz, cmd_until
        cmd_vx = vx
        cmd_wz = wz
        cmd_until = until

    return {
        "accepted": True,
        "vx_cmd": clamp(vx, -MAX_VX, MAX_VX),
        "wz_cmd": clamp(wz, -MAX_WZ, MAX_WZ),
        "duration": duration
    }

# Convenience endpoints for simple moves
@app.post("/forward")
def forward():
    data = request.get_json(force=True, silent=True) or {}
    speed = float(data.get("speed", 0.6))
    dur = float(data.get("duration", 1.0))
    return drive_passthrough(speed, 0.0, dur)

@app.post("/backward")
def backward():
    data = request.get_json(force=True, silent=True) or {}
    speed = float(data.get("speed", 0.6))
    dur = float(data.get("duration", 1.0))
    return drive_passthrough(-speed, 0.0, dur)

@app.post("/turn_left")
def turn_left():
    data = request.get_json(force=True, silent=True) or {}
    # `rate` here is an angular velocity in rad/s so it can map directly to the
    # `/drive` endpoint's `wz` field without additional conversion.
    rate = float(data.get("rate", 1.0))
    dur = float(data.get("duration", 0.8))
    return drive_passthrough(0.0, rate, dur)

@app.post("/turn_right")
def turn_right():
    data = request.get_json(force=True, silent=True) or {}
    # Mirror of `turn_left`; `rate` is specified in rad/s.
    rate = float(data.get("rate", 1.0))
    dur = float(data.get("duration", 0.8))
    return drive_passthrough(0.0, -rate, dur)

def drive_passthrough(vx, wz, duration):
    with state_lock:
        global cmd_vx, cmd_wz, cmd_until
        cmd_vx = vx
        cmd_wz = wz
        cmd_until = time.time() + max(0.0, float(duration))
    return {"accepted": True, "vx_cmd": vx, "wz_cmd": wz, "duration": duration}

def start_threads():
    t = threading.Thread(target=physics_loop, daemon=True)
    t.start()
    return t

if __name__ == "__main__":
    start_threads()
    print("\nREST controls ready:")
    print("  POST http://127.0.0.1:5000/drive      {\"vx\":0.6,\"wz\":0.0,\"duration\":2}")
    print("  POST http://127.0.0.1:5000/turn_left  {\"rate\":1.2,\"duration\":1.0}")
    print("  POST http://127.0.0.1:5000/stop")
    print("  GET  http://127.0.0.1:5000/pose\n")
    app.run(host="127.0.0.1", port=5000, debug=False, threaded=True)

