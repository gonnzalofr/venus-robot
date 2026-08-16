"""
Mission control for the dual-rover Venus mission -- ONE script that merges and
extends robot_map.py and new_python.py. It does three jobs at once:

  1. RELAY  - logs into the broker as both robots and forwards every message
              from each robot to the other's /recv topic, so a cube found by
              one robot is shared with the other.            (from new_python.py)
  2. LOG    - appends every received JSON message to a log file, with a
              timestamp and which robot sent it.             (from new_python.py)
  3. MAP    - draws a live matplotlib map: both robots' paths + heading, and
              every cube found.                              (from robot_map.py)

What's new vs the originals:
  - Tracks BOTH robots separately (path + heading per robot), not just one.
  - Cubes are drawn TO SCALE as coloured squares, so a 3x3x3 cm cube and a
    6x6x6 cm cube are visibly different on the map (the firmware sends
    size = 3 or size = 6). Each cube also carries an authoritative TEXT size
    cue ("6cm"/"3cm"), a thicker outline for the large cube, and a legend key,
    so the size class is readable even when the arena is zoomed out and the
    little squares shrink. Colour and the temperature measured around the cube
    are shown on the map and in the side panel.

Expected message formats (JSON, leading framing junk tolerated):
  cube:   {"type":"ROCK","x":"15","y":"-25","color":"Red","size":6,"temp":23.40}
  pose:   {"type":"pose","robot":"r1","x":0.12,"y":-0.03,"theta":1.57}
  status: {"type":"status","robot":"r1","mode":"explore","detail":"cube_located"}
NOTES:
  - The firmware publishes pose, status, ROCK (cubes), and mountain/black_tape
    objects over MQTT, so this map shows each robot's path + heading, the panel,
    and every object. (Pose/status/objects map ALL robots; only ROCK reports are
    relayed robot-to-robot, since cubes are the only thing they share for dedup.)
  - 'size' may legitimately arrive as 0 from the firmware's manual-scan path
    (a TODO stub); that is shown as UNKNOWN size ("?cm", dashed outline), NOT
    silently treated as a 3 cm cube. The autonomous mission path sends 3/6.

Run:
    pip install paho-mqtt matplotlib
    python mission_control.py
"""

import json
import math
import threading
import datetime

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation

import paho.mqtt.client as mqtt

# ============================== SETTINGS ==============================
BROKER = "mqtt.ics.ele.tue.nl"

# Robot A credentials + module id
USER_A, PASS_A, ID_A = "robot_66_1", "cSTY2NJS", "66"
# Robot B credentials + module id
USER_B, PASS_B, ID_B = "robot_6_1", "f8rWApIb", "6"

A_SEND, A_RECV = f"/pynqbridge/{ID_A}/send", f"/pynqbridge/{ID_A}/recv"
B_SEND, B_RECV = f"/pynqbridge/{ID_B}/send", f"/pynqbridge/{ID_B}/recv"

LOG_FILE = "pynq_mission_log.json"

# Units. The firmware sends cube (ROCK) x/y in CENTIMETRES and pose x/y in
# METRES. We draw everything in centimetres, so pose is scaled m -> cm. If a
# robot already sends pose in cm, set POSE_TO_CM = 1.0.
POSE_TO_CM = 100.0
ROCK_TO_CM = 1.0

# A find within this many cm of a known cube is the SAME cube, so the two
# robots' reports of one cube merge into a single marker.
CUBE_MERGE_CM = 12.0

# The firmware sends size = 6 (6x6x6 cm) or 3 (3x3x3 cm). Anything >= this is
# the large cube; 0 / missing is treated as UNKNOWN (not forced to 3).
LARGE_CUBE_MIN = 5.0
UNKNOWN_CUBE_SIDE_CM = 4.0   # nominal square drawn for unknown-size cubes

# Total cubes in the arena (for the "found N/TOTAL" readout).
TOTAL_CUBES = 6

# Message-type vocabulary. Mountains, craters and boundaries are now distinct
# environmental features (craters split out from the generic black-tape set).
POSE_T, TELEM_T, STATUS_T = "pose", "telemetry", "status"
CUBE_TYPES = {"rock", "scannable", "cube"}
MOUNTAIN_TYPES = {"mountain", "hill"}
CRATER_TYPES = {"crater", "cliff"}
BOUNDARY_TYPES = {"black_tape", "boundary", "tape"}
OBJECT_TYPES = MOUNTAIN_TYPES | CRATER_TYPES | BOUNDARY_TYPES

# Distinct visuals for the environmental features (used by both the map glyphs
# and the legend so they always agree).
MOUNTAIN_COLOR = "#8b5a2b"   # saddle brown -- a hill/mountain
BOUNDARY_COLOR = "#000000"   # black tape -- the arena limit
CRATER_COLOR   = "#d2691e"   # chocolate/rust ring -- a pit to avoid

# Per-robot drawing style (path/marker colour -- NOT the cube colour).
ROBOT_STYLE = {
    ID_A: {"c": "#e07b00", "name": f"Robot A ({ID_A})"},
    ID_B: {"c": "#c0379a", "name": f"Robot B ({ID_B})"},
}

# ===================== SHARED STATE (lock-guarded) =====================
state_lock = threading.Lock()
log_lock = threading.Lock()
robots = {
    rid: {"x": None, "y": None, "theta": 0.0, "path_x": [], "path_y": [],
          "mode": None, "detail": None}
    for rid in (ID_A, ID_B)
}
cubes = {}        # merge-key -> {x, y, color, face, size, large, temp, by}
objects = {}      # key -> {x, y, kind, color, face, temp}  (mountains/boundaries/items)
latest_telem = {}
last_logged = {ID_A: None, ID_B: None}


# ============================== HELPERS ==============================
def _to_float(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def safe_color(name):
    """Return a matplotlib-usable colour for drawing, falling back to grey."""
    try:
        mcolors.to_rgba(name)
        return name
    except (ValueError, TypeError):
        return "gray"


def log_message(robot_id, data):
    entry = {
        "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "sender_id": robot_id,
        "data": data,
    }
    try:
        with open(LOG_FILE, "a") as f:
            f.write(json.dumps(entry) + "\n")
    except Exception as e:
        print(f"LOG ERROR: {e}")


def _classify_size(data):
    """Return (size_cm_or_None, large_bool_or_None). 0/missing -> unknown."""
    size = _to_float(data.get("size"))
    if size is None or size == 0:
        return None, None                 # genuinely unknown -- do NOT assume 3
    return size, size >= LARGE_CUBE_MIN


def _add_cube(data, by):
    x, y = _to_float(data.get("x")), _to_float(data.get("y"))
    if x is None or y is None:
        return
    x *= ROCK_TO_CM
    y *= ROCK_TO_CM
    size, large = _classify_size(data)
    temp = _to_float(data.get("temp"))
    raw_color = str(data.get("color", "Unknown"))   # keep the firmware's word

    # Merge with an existing cube within CUBE_MERGE_CM (keep its key).
    key = None
    for k, c in cubes.items():
        if math.hypot(x - c["x"], y - c["y"]) <= CUBE_MERGE_CM:
            key = k
            break
    if key is None:
        key = (round(x, 1), round(y, 1))

    cubes[key] = {
        "x": x, "y": y, "color": raw_color, "face": safe_color(raw_color.lower()),
        "size": size, "large": large, "temp": temp, "by": by,
    }


def _add_object(data, kind):
    x, y = _to_float(data.get("x")), _to_float(data.get("y"))
    if x is None or y is None:
        return
    x *= ROCK_TO_CM
    y *= ROCK_TO_CM
    raw_color = str(data.get("color", "")) or None
    objects[(round(x, 1), round(y, 1), kind)] = {
        "x": x, "y": y, "kind": kind,
        "color": raw_color,
        "face": safe_color(raw_color.lower()) if raw_color else None,
        "temp": _to_float(data.get("temp")),
    }


def process_message(robot_id, payload):
    """Parse one MQTT payload, log it, and fold it into the shared map state."""
    try:
        raw = payload.decode("utf-8", errors="ignore")
        start = raw.find("{")
        if start == -1:
            return
        data = json.loads(raw[start:])
    except Exception:
        return False  # malformed / partial -> skip silently
    if not isinstance(data, dict):
        return False

    # Log every message (skip exact repeats from the same robot). Guarded so the
    # two callback threads can't interleave dedup state or file appends.
    with log_lock:
        if data != last_logged[robot_id]:
            last_logged[robot_id] = data
            log_message(robot_id, data)
            print(f"[{robot_id}] {data}")

    mtype = str(data.get("type", "")).strip().lower()
    is_cube = False

    with state_lock:
        if mtype == POSE_T:
            x, y = _to_float(data.get("x")), _to_float(data.get("y"))
            th = _to_float(data.get("theta"))
            if x is not None and y is not None:
                r = robots[robot_id]
                r["x"], r["y"] = x * POSE_TO_CM, y * POSE_TO_CM
                if th is not None:
                    r["theta"] = th
                if (not r["path_x"]
                        or math.hypot(r["x"] - r["path_x"][-1],
                                      r["y"] - r["path_y"][-1]) > 0.1):
                    r["path_x"].append(r["x"])
                    r["path_y"].append(r["y"])

        elif mtype == STATUS_T:
            r = robots[robot_id]
            r["mode"] = data.get("mode", r["mode"])
            r["detail"] = data.get("detail", r["detail"])

        elif mtype == TELEM_T:
            latest_telem.clear()
            latest_telem.update(data)

        elif mtype in CUBE_TYPES or ("color" in data and "size" in data):
            _add_cube(data, robot_id)
            is_cube = True

        elif mtype in OBJECT_TYPES:
            _add_object(data, mtype)

        else:
            # any other typed message carrying x/y -> generic map item
            if (_to_float(data.get("x")) is not None
                    and _to_float(data.get("y")) is not None):
                _add_object(data, mtype or "item")

    return is_cube   # tells the relay whether to forward this to the other robot


# ============================ MQTT (relay) ============================
client_a = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
client_a.username_pw_set(USER_A, PASS_A)

client_b = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
client_b.username_pw_set(USER_B, PASS_B)


def on_message_from_A(client, userdata, msg):
    # Map ALL of A's telemetry, but only relay its CUBE reports to B: the robots
    # only need each other's cubes (for cross-robot dedup), so not forwarding
    # pose/status/objects keeps each robot's UART link light.
    if process_message(ID_A, msg.payload):
        client_b.publish(B_RECV, msg.payload)   # relay A's cube -> B


def on_message_from_B(client, userdata, msg):
    if process_message(ID_B, msg.payload):
        client_a.publish(A_RECV, msg.payload)   # relay B's cube -> A


client_a.on_message = on_message_from_A
client_b.on_message = on_message_from_B


# Subscribe on every (re)connect so subscriptions survive an automatic reconnect.
def _make_on_connect(sub_topic, name):
    def _on_connect(client, userdata, flags, reason_code, properties):
        client.subscribe(sub_topic)
        print(f"[mqtt] {name}: connected (rc={reason_code}); subscribed {sub_topic}")
    return _on_connect


client_a.on_connect = _make_on_connect(A_SEND, "A")
client_b.on_connect = _make_on_connect(B_SEND, "B")


# --- Non-blocking connection: connect in the background, retry until it works ---
RECONNECT_DELAY_S = 5
_mqtt_stop = threading.Event()   # set on shutdown to break the retry loops


def _connection_worker(client, name):
    """Background worker: keep trying to connect this client until it succeeds,
    then hand off to paho's network loop (which auto-reconnects on later drops).
    Swallows TimeoutError / connection errors and retries every RECONNECT_DELAY_S
    seconds, so the GUI never blocks, freezes, or crashes on a missing broker."""
    while not _mqtt_stop.is_set():
        try:
            client.connect(BROKER)
            client.loop_start()        # on_connect (above) subscribes once CONNACK arrives
            return                     # connected -- paho keeps the link alive from here
        except (TimeoutError, OSError) as e:
            print(f"[mqtt] {name}: connect failed ({e}); retrying in {RECONNECT_DELAY_S}s")
        except Exception as e:         # anything else -> keep the app alive, retry
            print(f"[mqtt] {name}: connect error ({e!r}); retrying in {RECONNECT_DELAY_S}s")
        _mqtt_stop.wait(RECONNECT_DELAY_S)   # interruptible wait (wakes at once on shutdown)


def start_mqtt():
    """Launch each robot's MQTT connection in its OWN background thread so the GUI
    opens immediately and never blocks on connect(). Each thread retries until its
    broker connection succeeds."""
    threading.Thread(target=_connection_worker, args=(client_a, "A"), daemon=True).start()
    threading.Thread(target=_connection_worker, args=(client_b, "B"), daemon=True).start()
    print("MQTT connecting in the background; GUI is live "
          "(relay + logging begin once connected).")


# =============================== PLOT ===============================
fig, ax = plt.subplots(figsize=(9.5, 8))
fig.canvas.manager.set_window_title("Venus mission control")
info_text = fig.text(
    0.015, 0.985, "", va="top", ha="left", family="monospace", fontsize=9,
    bbox=dict(boxstyle="round", facecolor="#f3f3f3", edgecolor="#bbb"),
)


def _fit_limits(xs, ys, pad=15.0, min_span=45.0):
    """Frame the view around all content, with padding and a floor on the span
    so a single small cube isn't zoomed in absurdly."""
    if not xs:
        ax.set_xlim(-50, 50)
        ax.set_ylim(-50, 50)
        return
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    span = max(max(xs) - min(xs), max(ys) - min(ys), min_span) + 2 * pad
    ax.set_xlim(cx - span / 2, cx + span / 2)
    ax.set_ylim(cy - span / 2, cy + span / 2)


def _size_label(c):
    return f"{c['size']:.0f}cm" if c["size"] is not None else "?cm"


def update(_frame):
    with state_lock:
        rdata = {
            rid: {"x": r["x"], "y": r["y"], "theta": r["theta"],
                  "px": list(r["path_x"]), "py": list(r["path_y"]),
                  "mode": r["mode"], "detail": r["detail"]}
            for rid, r in robots.items()
        }
        cube_list = list(cubes.values())
        obj_list = list(objects.values())
        telem = dict(latest_telem)

    ax.clear()
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle=":", alpha=0.4)
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_title("Venus mission map  (live)")
    ax.axhline(0, color="#ccc", lw=0.8)
    ax.axvline(0, color="#ccc", lw=0.8)

    xs, ys = [], []
    robot_handles = []

    # robots: trail + position + heading arrow
    for rid, r in rdata.items():
        style = ROBOT_STYLE.get(rid, {"c": "#444", "name": rid})
        if r["px"]:
            ax.plot(r["px"], r["py"], "-", color=style["c"], lw=1.4, alpha=0.7)
            xs += r["px"]
            ys += r["py"]
        if r["x"] is not None:
            ax.plot(r["x"], r["y"], "o", color=style["c"], ms=11, zorder=5,
                    markeredgecolor="k")
            arrow = 12.0  # cm
            ax.annotate(
                "", xytext=(r["x"], r["y"]),
                xy=(r["x"] + arrow * math.cos(r["theta"]),
                    r["y"] + arrow * math.sin(r["theta"])),
                arrowprops=dict(arrowstyle="->", color=style["c"], lw=2))
            xs.append(r["x"])
            ys.append(r["y"])
            robot_handles.append(Line2D([0], [0], marker="o", color="w",
                                        markerfacecolor=style["c"],
                                        markeredgecolor="k", markersize=9,
                                        label=style["name"]))

    # --- environmental map features, each with a distinct glyph ---
    #   Mountain  : brown up-triangle (a hill)
    #   Boundary  : black square (the black-tape arena limit)
    #   Crater    : rust-coloured ring + centre dot (a pit to avoid)
    mountain_n = crater_n = boundary_n = 0
    for o in obj_list:
        k = o["kind"]
        if k in MOUNTAIN_TYPES:
            ax.plot(o["x"], o["y"], marker="^", color=MOUNTAIN_COLOR,
                    markeredgecolor="#5a3a1a", ms=14, zorder=4)
            mountain_n += 1
        elif k in CRATER_TYPES:
            ax.plot(o["x"], o["y"], marker="o", markerfacecolor="none",
                    markeredgecolor=CRATER_COLOR, markeredgewidth=2.5,
                    ms=15, zorder=4)
            ax.plot(o["x"], o["y"], marker="o", color=CRATER_COLOR, ms=4, zorder=5)
            crater_n += 1
        elif k in BOUNDARY_TYPES:
            ax.plot(o["x"], o["y"], marker="s", color=BOUNDARY_COLOR, ms=7, zorder=4)
            boundary_n += 1
        else:
            # any other typed item carrying a colour -> a coloured dot + label
            ax.scatter(o["x"], o["y"], s=80, c=o["face"] or "gray",
                       edgecolors="k", linewidths=0.6, zorder=4)
            lbl = k + (f"  {o['temp']:.1f}°C" if o["temp"] is not None else "")
            ax.annotate(lbl, (o["x"], o["y"]), textcoords="offset points",
                        xytext=(8, 8), fontsize=8)
        xs.append(o["x"])
        ys.append(o["y"])

    # cubes: to-scale square (colour fill) + thick edge for large + text size cue
    small_n = large_n = unknown_n = 0
    for c in cube_list:
        if c["large"] is True:
            side, lw, ls = c["size"], 2.6, "solid"
            large_n += 1
        elif c["large"] is False:
            side, lw, ls = c["size"], 1.3, "solid"
            small_n += 1
        else:                                   # unknown size
            side, lw, ls = UNKNOWN_CUBE_SIDE_CM, 1.3, "dashed"
            unknown_n += 1
        ax.add_patch(Rectangle(
            (c["x"] - side / 2, c["y"] - side / 2), side, side,
            facecolor=c["face"], edgecolor="k", linewidth=lw, linestyle=ls,
            zorder=6))
        temp_s = f"{c['temp']:.1f}°C" if c["temp"] is not None else "temp --"
        ax.annotate(f"{_size_label(c)} {c['color']}\n{temp_s}",
                    (c["x"], c["y"]), textcoords="offset points",
                    xytext=(side * 0.6 + 6, 6), fontsize=8, zorder=7)
        xs += [c["x"] - side, c["x"] + side]
        ys += [c["y"] - side, c["y"] + side]

    _fit_limits(xs, ys)

    # legend: environmental features + cube-size key + robots. Cube FILL shows the
    # sensor-detected colour (also named on each cube and in the panel). Only drawn
    # when non-empty (avoids the per-frame "no artists with labels" warning).
    feature_handles = [
        Line2D([0], [0], marker="^", color="w", markerfacecolor=MOUNTAIN_COLOR,
               markeredgecolor="#5a3a1a", markersize=12, label="Mountain"),
        Line2D([0], [0], marker="s", color="w", markerfacecolor=BOUNDARY_COLOR,
               markeredgecolor="k", markersize=10, label="Boundary (tape)"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="none",
               markeredgecolor=CRATER_COLOR, markeredgewidth=2.5, markersize=12,
               label="Crater"),
    ]
    size_handles = [
        Line2D([0], [0], marker="s", color="w", markerfacecolor="#cfc7b8",
               markeredgecolor="k", markersize=13, markeredgewidth=2.0,
               label="cube 6 cm (fill = colour)"),
        Line2D([0], [0], marker="s", color="w", markerfacecolor="#cfc7b8",
               markeredgecolor="k", markersize=8, label="cube 3 cm"),
    ]
    if unknown_n:
        size_handles.append(
            Line2D([0], [0], marker="s", color="w", markerfacecolor="#cfc7b8",
                   markeredgecolor="k", markersize=10, linestyle="--",
                   label="cube (size ?)"))
    handles = feature_handles + size_handles + robot_handles
    if handles:
        ax.legend(handles=handles, loc="upper right", fontsize=8)

    # info panel
    lines = []
    for rid, r in rdata.items():
        name = ROBOT_STYLE.get(rid, {"name": rid})["name"]
        pos = (f"({r['x']:.0f},{r['y']:.0f}) {math.degrees(r['theta']):.0f}°"
               if r["x"] is not None else "(no pose yet)")
        lines.append(name)
        lines.append(f"  pos : {pos}")
        lines.append(f"  mode: {r['mode']}  {r['detail'] or ''}")

    if telem:
        lines.append("")
        lines.append("TELEMETRY")
        for k in ("tof_bottom_mm", "tof_middle_mm", "tof_top_mm",
                  "ir_left", "ir_right", "temp_c", "moving"):
            if k in telem:
                lines.append(f"  {k:<14}: {telem[k]}")

    lines.append("")
    lines.append(f"CUBES {len(cube_list)}/{TOTAL_CUBES}   "
                 f"(6cm: {large_n}  3cm: {small_n}"
                 + (f"  ?: {unknown_n}" if unknown_n else "") + ")")
    for c in sorted(cube_list, key=lambda c: (c["large"] is not True, c["color"])):
        t = f"{c['temp']:.1f}°C" if c["temp"] is not None else "  --  "
        lines.append(f"  {_size_label(c):<4} {c['color']:<8} {t}"
                     f"  @({c['x']:.0f},{c['y']:.0f})")

    lines.append("")
    lines.append(f"FEATURES  mountains:{mountain_n}  "
                 f"craters:{crater_n}  boundary:{boundary_n}")
    info_text.set_text("\n".join(lines))


def main():
    start_mqtt()
    # keep a reference so the animation isn't garbage-collected
    _anim = FuncAnimation(fig, update, interval=300, cache_frame_data=False)
    try:
        plt.show()
    finally:
        print("\nShutting down bridge...")
        _mqtt_stop.set()             # stop any in-progress connect retries
        for c in (client_a, client_b):
            try:
                c.loop_stop()
                c.disconnect()
            except Exception:
                pass                 # never crash on shutdown (may not have connected)


if __name__ == "__main__":
    main()
