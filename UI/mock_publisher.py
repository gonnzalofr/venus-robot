#!/usr/bin/env python3
"""
mock_publisher.py  –  Standalone MQTT telemetry simulator
==========================================================
Publishes fake robot telemetry to the same broker that main.py listens on,
so you can develop and test the dashboard without physical hardware.

Run in a second terminal while main.py is running:

    python mock_publisher.py

Press Ctrl-C to stop.

──────────────────────────────────────────────────────────────────────────────
JSON payload structure sent on every tick
──────────────────────────────────────────────────────────────────────────────

    {
        "robot": {
            "x":       <float>,   // arena x-coordinate  (origin = arena centre)
            "y":       <float>,   // arena y-coordinate  (origin = arena centre)
            "heading": <float>    // degrees  0=East 90=North 270=South  CCW+
        },
        "objects": {              // omitted on some ticks (see OBJECT_EVERY)
            "cubes": [
                { "id": "C1", "x": <float>, "y": <float>, "color": "yellow" }
            ],
            "hills": [
                { "x": <float>, "y": <float>, "radius": <float> }
            ],
            "craters": [
                { "x": <float>, "y": <float>, "radius": <float> }
            ],
            "boundaries": [
                { "x1": <float>, "y1": <float>, "x2": <float>, "y2": <float> },
                ...
            ]
        }
    }

──────────────────────────────────────────────────────────────────────────────
Movement model
──────────────────────────────────────────────────────────────────────────────

    Robot A  –  clockwise circle (angle decreases over time)
                orbit radius RADIUS_A, centred on (0, 0)

    Robot B  –  counter-clockwise circle (angle increases over time)
                orbit radius RADIUS_B, centred on (0, 0)

    Heading convention (matches main.py parser):
        0°   = East  (+X)
        90°  = North (+Y, up on screen)
        270° = South (-Y, down on screen)
        CCW positive

    For a robot moving clockwise, the tangent direction is:
        heading = degrees( -(θ + π/2) )   where θ = -ω·t  (decreasing angle)

    For a robot moving counter-clockwise:
        heading = degrees( θ + π/2 )       where θ = +ω·t  (increasing angle)
"""

import json
import math
import sys
import time

import paho.mqtt.client as mqtt

# ── Broker credentials (match main.py) ────────────────────────────────────────
MQTT_BROKER   = "mqtt.ics.ele.tue.nl"
MQTT_PORT     = 1883
MQTT_USERNAME = "robot_66_1"
MQTT_PASSWORD = "cSTY2NJS"

TOPIC_A = "/PYNQBRIDGE/A/SEND"
TOPIC_B = "/PYNQBRIDGE/B/SEND"

# ── Simulation tuning ─────────────────────────────────────────────────────────
TICK_RATE    = 0.1     # seconds between publishes  (10 Hz)
OMEGA        = 0.05    # angular speed in radians per tick
RADIUS_A     = 100.0   # orbit radius for Robot A  (arena units)
RADIUS_B     =  70.0   # orbit radius for Robot B  (arena units)

# Objects are included in the payload once every N ticks.
# Set to 1 to always include them; higher values reduce noise in the console.
OBJECT_EVERY = 5

# ── Static map objects (centred-coordinate system, origin = arena centre) ─────
#
# These are sent periodically to let the dashboard map fill in progressively,
# just like real robots would do as they discover landmarks.
#
# One of each type is included so you can verify every visual rule in main.py:
#   • Cube    → coloured square
#   • Hill    → concentric green rings
#   • Crater  → BLACK TAPE circle  (same style as boundaries)
#   • Boundary → BLACK TAPE line segments forming the arena perimeter

STATIC_OBJECTS = {
    "cubes": [
        # A single yellow cube placed in the top-right quadrant
        {"id": "C1", "x": 120.0, "y": 150.0, "color": "yellow"},
    ],
    "hills": [
        # A medium hill in the top-left quadrant
        {"x": -130.0, "y": 110.0, "radius": 55.0},
    ],
    "craters": [
        # A crater in the bottom-right quadrant  (BLACK TAPE visual rule)
        {"x": 140.0, "y": -120.0, "radius": 35.0},
    ],
    "boundaries": [
        # Square perimeter ±220 units from origin (≈ 20 units inside ±240 edge)
        {"x1": -220.0, "y1": -220.0, "x2":  220.0, "y2": -220.0},
        {"x1":  220.0, "y1": -220.0, "x2":  220.0, "y2":  220.0},
        {"x1":  220.0, "y1":  220.0, "x2": -220.0, "y2":  220.0},
        {"x1": -220.0, "y1":  220.0, "x2": -220.0, "y2": -220.0},
    ],
}


# ── MQTT client setup ─────────────────────────────────────────────────────────

def _make_client() -> mqtt.Client:
    """Create a paho Client, handling the API-version change in paho ≥ 2.0."""
    try:
        return mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id="mock_publisher",
        )
    except AttributeError:
        return mqtt.Client(client_id="mock_publisher")


# ── Heading helpers ───────────────────────────────────────────────────────────

def cw_heading(theta: float) -> float:
    """
    Tangent direction (degrees) for a robot moving CLOCKWISE.

    Derived from the discrete velocity for theta decreasing by omega each tick:
        delta_x ≈ +omega * sin(theta)
        delta_y ≈ -omega * cos(theta)
        heading  = atan2(-cos(theta), sin(theta))
                 = theta - pi/2          (mod 2pi)

    Cardinal check (CW: East -> South -> West -> North):
        theta=0       (East)  -> heading 270° (South)  ✓
        theta=-pi/2   (South) -> heading 180° (West)   ✓
        theta=-pi     (West)  -> heading  90° (North)  ✓
        theta=-3pi/2  (North) -> heading   0° (East)   ✓
    """
    return math.degrees(theta - math.pi / 2) % 360


def ccw_heading(theta: float) -> float:
    """
    Tangent direction (degrees) for a robot moving COUNTER-CLOCKWISE.

    Derived from the discrete velocity for theta increasing by omega each tick:
        delta_x ≈ -omega * sin(theta)
        delta_y ≈ +omega * cos(theta)
        heading  = atan2(cos(theta), -sin(theta))
                 = theta + pi/2          (mod 2pi)

    Cardinal check (CCW: West -> South -> East -> North):
        theta=pi      (West)  -> heading 270° (South)  ✓
        theta=3pi/2   (South) -> heading   0° (East)   ✓
        theta=2pi     (East)  -> heading  90° (North)  ✓
        theta=5pi/2   (North) -> heading 180° (West)   ✓
    """
    return math.degrees(theta + math.pi / 2) % 360


# ── Main simulation loop ──────────────────────────────────────────────────────

def run() -> None:
    client = _make_client()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    print(f"[MOCK] Connecting to {MQTT_BROKER}:{MQTT_PORT} …")
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    except OSError as exc:
        print(f"[MOCK] Connection failed: {exc}")
        sys.exit(1)

    client.loop_start()
    print("[MOCK] Connected.  Publishing at 10 Hz — Ctrl-C to stop.\n")

    tick    = 0
    theta_a = 0.0         # Robot A starts on the +X axis (East)
    theta_b = math.pi     # Robot B starts on the -X axis (West), opposite side
    prev_xa = RADIUS_A    # track previous position for delta reporting
    prev_ya = 0.0
    prev_xb = RADIUS_B * math.cos(math.pi)
    prev_yb = RADIUS_B * math.sin(math.pi)

    try:
        while True:
            # ── Robot A: CLOCKWISE orbit ───────────────────────────────────────
            # θ decreases each tick  →  angle goes East→South→West→North (CW)
            theta_a -= OMEGA
            xa = RADIUS_A * math.cos(theta_a)
            ya = RADIUS_A * math.sin(theta_a)
            ha = cw_heading(theta_a)

            # ── Robot B: COUNTER-CLOCKWISE orbit ──────────────────────────────
            # θ increases each tick  →  angle goes West→South→East→North (CCW)
            theta_b += OMEGA
            xb = RADIUS_B * math.cos(theta_b)
            yb = RADIUS_B * math.sin(theta_b)
            hb = ccw_heading(theta_b)

            # ── Build payloads ─────────────────────────────────────────────────
            # Include map objects every OBJECT_EVERY ticks so the map fills in
            # progressively.  On other ticks send an empty objects dict so the
            # dashboard retains the last-known object state (last-writer-wins).
            objects_payload = STATIC_OBJECTS if (tick % OBJECT_EVERY == 0) else {}

            payload_a = json.dumps({
                "robot":   {"x": round(xa, 2), "y": round(ya, 2), "heading": round(ha, 1)},
                "objects": objects_payload,
            })

            payload_b = json.dumps({
                "robot":   {"x": round(xb, 2), "y": round(yb, 2), "heading": round(hb, 1)},
                "objects": {},   # B never reports objects in this simulation
            })

            client.publish(TOPIC_A, payload_a)
            client.publish(TOPIC_B, payload_b)

            # Print every tick so you can confirm coordinates are changing.
            # Δ shows displacement from the previous tick — should be ~5 units.
            da = math.hypot(xa - prev_xa, ya - prev_ya)
            db = math.hypot(xb - prev_xb, yb - prev_yb)
            obj_tag = " [objects]" if tick % OBJECT_EVERY == 0 else ""
            print(
                f"[tick {tick:05d}] "
                f"A({xa:7.2f}, {ya:7.2f})  Δ={da:.2f}  "
                f"B({xb:7.2f}, {yb:7.2f})  Δ={db:.2f}"
                f"{obj_tag}"
            )
            prev_xa, prev_ya = xa, ya
            prev_xb, prev_yb = xb, yb

            tick += 1
            time.sleep(TICK_RATE)

    except KeyboardInterrupt:
        print("\n[MOCK] Stopped by user.")
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    run()
