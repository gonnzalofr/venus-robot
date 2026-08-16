"""
Live MQTT mission map for the rover.

Connects to the broker (same credentials as new_python.py), keeps the A<->B
relay alive, and draws a live matplotlib map:

  - robot PATH from 'pose' messages, with a heading arrow at the current spot
  - one colored dot per rock from item messages, e.g.
        {"type":"ROCK","x":"15","y":"-25","color":"Red","size":6,"temp":23.40}
    dot color = "color", dot size scaled by "size", label = temperature
  - a live info panel with the latest 'telemetry' and 'status'

Run:
    pip install paho-mqtt matplotlib
    python robot_map.py
"""

import json
import math
import threading

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.animation import FuncAnimation

import paho.mqtt.client as mqtt

# --- SETTINGS ---
BROKER = "mqtt.ics.ele.tue.nl"

# Robot A
USER_A = "robot_66_1"
PASS_A = "cSTY2NJS"
ID_A   = "66"

# Robot B
USER_B = "robot_6_1"
PASS_B = "f8rWApIb"
ID_B   = "6"

A_SEND = f"/pynqbridge/{ID_A}/send"
A_RECV = f"/pynqbridge/{ID_A}/recv"
B_SEND = f"/pynqbridge/{ID_B}/send"
B_RECV = f"/pynqbridge/{ID_B}/recv"

# If rocks land in the wrong place relative to the path, the rock x/y are
# probably in different units than pose x/y (e.g. cm vs m). Tweak this to
# rescale rock coordinates onto the pose frame. 1.0 = use as-is.
ROCK_SCALE = 1.0

# Message types we understand. Anything else with x/y is treated as an "item".
POSE_T = "pose"
TELEM_T = "telemetry"
STATUS_T = "status"

# --- SHARED STATE (written by MQTT thread, read by plot thread) ---
state_lock = threading.Lock()
path_x, path_y = [], []          # robot trail
pose = {"x": None, "y": None, "theta": 0.0}
rocks = {}                       # keyed by rounded (x, y) -> dict
latest_telem = {}
latest_status = {}


def _to_float(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def process_message(robot_id, payload):
    """Parse one MQTT payload and fold it into shared state."""
    try:
        raw = payload.decode("utf-8", errors="ignore")
        start = raw.find("{")
        if start == -1:
            return
        data = json.loads(raw[start:])
    except Exception:
        return  # malformed / partial message -> skip silently

    if not isinstance(data, dict):
        return

    mtype = str(data.get("type", "")).strip().lower()

    with state_lock:
        if mtype == POSE_T:
            x, y = _to_float(data.get("x")), _to_float(data.get("y"))
            th = _to_float(data.get("theta"))
            if x is not None and y is not None:
                pose["x"], pose["y"] = x, y
                if th is not None:
                    pose["theta"] = th
                # only append when the robot actually moved a bit
                if not path_x or math.hypot(x - path_x[-1], y - path_y[-1]) > 1e-4:
                    path_x.append(x)
                    path_y.append(y)

        elif mtype == TELEM_T:
            latest_telem.clear()
            latest_telem.update(data)

        elif mtype == STATUS_T:
            latest_status.clear()
            latest_status.update(data)

        else:
            # treat any other typed message carrying x/y as a placeable item
            x, y = _to_float(data.get("x")), _to_float(data.get("y"))
            if x is not None and y is not None:
                x *= ROCK_SCALE
                y *= ROCK_SCALE
                key = (round(x, 2), round(y, 2))
                rocks[key] = {
                    "x": x,
                    "y": y,
                    "type": mtype or "item",
                    "color": str(data.get("color", "gray")).lower(),
                    "size": _to_float(data.get("size")) or 1.0,
                    "temp": _to_float(data.get("temp")),
                }
                print(f"ITEM: {data}")


# --- MQTT CALLBACKS (parse + keep the relay alive) ---

def on_message_from_A(client, userdata, msg):
    process_message(ID_A, msg.payload)
    client_b.publish(B_RECV, msg.payload)   # relay A -> B


def on_message_from_B(client, userdata, msg):
    process_message(ID_B, msg.payload)
    client_a.publish(A_RECV, msg.payload)   # relay B -> A


# --- CLIENTS ---
client_a = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
client_a.username_pw_set(USER_A, PASS_A)
client_a.on_message = on_message_from_A

client_b = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
client_b.username_pw_set(USER_B, PASS_B)
client_b.on_message = on_message_from_B


def start_mqtt():
    client_a.connect(BROKER)
    client_b.connect(BROKER)
    client_a.subscribe(A_SEND)
    client_b.subscribe(B_SEND)
    client_a.loop_start()
    client_b.loop_start()
    print("MQTT bridge active. Listening + relaying...")


def safe_color(name):
    try:
        mcolors.to_rgba(name)
        return name
    except (ValueError, TypeError):
        return "gray"


# --- PLOT ---
fig, ax = plt.subplots(figsize=(9, 8))
fig.canvas.manager.set_window_title("Rover mission map")

info_text = fig.text(
    0.015, 0.985, "", va="top", ha="left", family="monospace", fontsize=9,
    bbox=dict(boxstyle="round", facecolor="#f3f3f3", edgecolor="#bbb"),
)


def update(_frame):
    with state_lock:
        px, py = list(path_x), list(path_y)
        rx, ry, theta = pose["x"], pose["y"], pose["theta"]
        rock_list = list(rocks.values())
        telem = dict(latest_telem)
        status = dict(latest_status)

    ax.clear()
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle=":", alpha=0.4)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Rover mission map  (live)")

    # robot trail
    if px:
        ax.plot(px, py, "-", color="#2a7", lw=1.5, alpha=0.7, label="path")

    # robot + heading arrow
    if rx is not None:
        ax.plot(rx, ry, "o", color="#0a0", ms=10, zorder=5)
        arrow = 0.4 * (ax.get_xlim()[1] - ax.get_xlim()[0]) * 0.1 or 0.3
        ax.annotate(
            "", xy=(rx + arrow * math.cos(theta), ry + arrow * math.sin(theta)),
            xytext=(rx, ry),
            arrowprops=dict(arrowstyle="->", color="#0a0", lw=2),
        )

    # rocks / items
    for r in rock_list:
        c = safe_color(r["color"])
        s = max(60, r["size"] * 40)
        ax.scatter(r["x"], r["y"], s=s, c=c, edgecolors="k",
                   linewidths=0.6, zorder=6)
        label = r["type"]
        if r["temp"] is not None:
            label += f"  {r['temp']:.1f}°C"
        ax.annotate(label, (r["x"], r["y"]),
                    textcoords="offset points", xytext=(8, 8), fontsize=8)

    # info panel
    lines = ["TELEMETRY"]
    if telem:
        for k in ("tof_bottom_mm", "tof_middle_mm", "tof_top_mm",
                  "ir_left", "ir_right", "temp_c", "moving"):
            if k in telem:
                lines.append(f"  {k:<14}: {telem[k]}")
    else:
        lines.append("  (waiting...)")
    lines.append("STATUS")
    if status:
        lines.append(f"  mode  : {status.get('mode')}")
        lines.append(f"  detail: {status.get('detail')}")
    else:
        lines.append("  (waiting...)")
    lines.append(f"ROCKS : {len(rock_list)}")
    info_text.set_text("\n".join(lines))

    if px or rock_list:
        ax.legend(loc="upper right", fontsize=8)


def main():
    start_mqtt()
    # keep a reference so the animation isn't garbage-collected
    _anim = FuncAnimation(fig, update, interval=300, cache_frame_data=False)
    try:
        plt.show()
    finally:
        print("\nShutting down bridge...")
        client_a.loop_stop()
        client_b.loop_stop()
        client_a.disconnect()
        client_b.disconnect()


if __name__ == "__main__":
    main()
