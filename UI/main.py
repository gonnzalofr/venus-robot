#!/usr/bin/env python3
"""
Real-time Robot Arena Dashboard
================================
Visualises two physical robots exploring a defined arena via pygame (2-D) and
paho-mqtt (networking).

Architecture
────────────
    Physical robots  ──[telemetry]──>  MQTT Broker  ──>  PC (this script)
    Physical robots  <──[commands]──   MQTT Broker  <──  PC (this script)

    Subscriptions (PC listens):
        /PYNQBRIDGE/A/SEND   – telemetry from Robot A
        /PYNQBRIDGE/B/SEND   – telemetry from Robot B

    Publications (PC sends):
        /PYNQBRIDGE/A/RECV   – commands to Robot A
        /PYNQBRIDGE/B/RECV   – commands to Robot B

Threading model
───────────────
    Main thread   – pygame render loop (30 FPS)
    MQTT thread   – paho background thread started with client.loop_start()

    Data hand-off uses a threading.Lock-protected dict.  The MQTT callback
    writes into it; the render loop takes a snapshot under the lock, then
    renders freely without holding the lock.

Phase 2 additions
─────────────────
    • Path History (Trails)       — deque per robot, drawn as fading dots behind each robot
    • Dynamic Auto-Scaling Camera — viewport auto-expands when robots approach / exceed edges
    • Live Telemetry HUD          — semi-transparent corner overlay (status, pose, heading)

Usage
─────
    Normal mode  :  python main.py
    Mock mode    :  python main.py --mock        (simulates robots locally)
"""

import collections   # PHASE 2: deque for trail history
import json
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import List

import pygame
import paho.mqtt.client as mqtt

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

MQTT_BROKER   = "mqtt.ics.ele.tue.nl"
MQTT_PORT     = 1883
MQTT_USERNAME = "robot_66_1"
MQTT_PASSWORD = "cSTY2NJS"

# Fallback credentials — swap the constants above if the primary pair fails:
# MQTT_USERNAME = "robot_6_1"
# MQTT_PASSWORD = "f8rWAplb"

TOPIC_A_SEND = "/PYNQBRIDGE/A/SEND"   # robot A → PC
TOPIC_B_SEND = "/PYNQBRIDGE/B/SEND"   # robot B → PC
TOPIC_A_RECV = "/PYNQBRIDGE/A/RECV"   # PC → robot A
TOPIC_B_RECV = "/PYNQBRIDGE/B/RECV"   # PC → robot B

# ── Physical arena dimensions (arbitrary units, e.g. centimetres) ─────────────
ARENA_W = 500.0
ARENA_H = 500.0

# ── Pygame window ──────────────────────────────────────────────────────────────
WIN_W = 940
WIN_H = 680

# Arena panel (left side of window)
PANEL_X = 20          # left edge of arena panel in screen coords
PANEL_Y = 50          # top  edge (leaves room for title bar)
PANEL_W = 620         # panel width  in pixels
PANEL_H = 620         # panel height in pixels  (same → square arena)

FPS = 30

# ── Colours ───────────────────────────────────────────────────────────────────
C_BG         = (28,  28,  34)
C_ARENA_FILL = (195, 183, 155)   # sandy floor
C_GRID       = (175, 164, 138)
C_ROBOT_A    = (255,  45,  45)   # vivid red
C_ROBOT_B    = ( 40, 160, 255)   # vivid blue
C_WHITE      = (255, 255, 255)
C_TEXT       = (215, 215, 215)
C_OK         = ( 55, 200,  80)
C_ERR        = (220,  55,  55)
C_CUBE_YEL   = (255, 200,   0)
C_CUBE_ORA   = (255, 135,   0)
C_HILL_OUTER = ( 90, 165,  65)
C_HILL_INNER = ( 55, 130,  40)

# ── THE BLACK-TAPE RULE ────────────────────────────────────────────────────────
# In the physical arena, both Craters and the outer Boundary are made of black
# tape on the floor.  The UI must make them visually *indistinguishable* — both
# are drawn with the exact same colour and stroke weight.
C_BLACK_TAPE       = ( 8,   8,   8)
BLACK_TAPE_WIDTH   = 8      # pixel thickness for crater rings AND boundary lines

# ── Render sizes ──────────────────────────────────────────────────────────────
ROBOT_RADIUS   = 12
HEADING_LEN    = 24    # length of heading indicator line in pixels
CUBE_HALF      = 7     # half-side of a cube square
HILL_RINGS     = 3     # how many concentric rings per hill
HILL_RING_W    = 5

# ── PHASE 2: Path History settings ────────────────────────────────────────────
# Each robot stores up to TRAIL_MAX_LEN (x, y) samples.  When the deque is
# full the oldest point is automatically dropped (collections.deque maxlen).
TRAIL_MAX_LEN = 300    # maximum stored position samples per robot
TRAIL_DOT_RAD = 2      # radius in pixels of each trail dot


# ═══════════════════════════════════════════════════════════════════════════════
# DATA STRUCTURES
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class RobotPose:
    x:         float = 0.0
    y:         float = 0.0
    heading:   float = 0.0   # degrees; 0 = east (+X), 90 = north (+Y), CCW positive
    last_seen: float = 0.0   # time.time() of latest update; 0 = never


@dataclass
class Cube:
    x:      float
    y:      float
    obj_id: str   = ""
    color:  str   = "yellow"


@dataclass
class Hill:
    x:      float
    y:      float
    radius: float = 40.0


@dataclass
class Crater:          # BLACK TAPE — same visual as Boundary
    x:      float
    y:      float
    radius: float = 30.0


@dataclass
class Boundary:        # BLACK TAPE — same visual as Crater
    x1: float
    y1: float
    x2: float
    y2: float


@dataclass
class WorldState:
    """The merged view of the arena, updated from both robots' telemetry."""
    robot_a:    RobotPose  = field(default_factory=RobotPose)
    robot_b:    RobotPose  = field(default_factory=RobotPose)
    cubes:      List[Cube]     = field(default_factory=list)
    hills:      List[Hill]     = field(default_factory=list)
    craters:    List[Crater]   = field(default_factory=list)
    boundaries: List[Boundary] = field(default_factory=list)


# ═══════════════════════════════════════════════════════════════════════════════
# PARSING  ←  MODIFY THIS FUNCTION WHEN THE REAL PYNQ PAYLOAD IS KNOWN
# ═══════════════════════════════════════════════════════════════════════════════

def parse_mqtt_message(topic: str, payload: str) -> dict:
    """
    Parse a raw UTF-8 MQTT payload into a normalised Python dict.

    THIS IS THE SINGLE FUNCTION TO CHANGE when the actual PYNQ JSON schema
    is decided.  Everything else in the pipeline consumes the dict this
    function returns.

    ── Expected JSON schema ────────────────────────────────────────────────
    {
        "robot": {
            "x":       <float>,   // arena x-coordinate
            "y":       <float>,   // arena y-coordinate
            "heading": <float>    // degrees, 0=east, 90=north, CCW positive
        },
        "objects": {
            "cubes": [
                { "id": <str>, "x": <float>, "y": <float>, "color": <str> }
            ],
            "hills": [
                { "x": <float>, "y": <float>, "radius": <float> }
            ],
            "craters": [
                { "x": <float>, "y": <float>, "radius": <float> }
            ],
            "boundaries": [
                { "x1": <float>, "y1": <float>,
                  "x2": <float>, "y2": <float> }
            ]
        }
    }
    ────────────────────────────────────────────────────────────────────────

    Returns a dict with keys:
        robot_x, robot_y, robot_heading  (floats)
        cubes, hills, craters, boundaries  (lists of dataclass objects)
    Returns an empty dict {} on any parse failure.
    """
    try:
        data = json.loads(payload)
    except json.JSONDecodeError as exc:
        print(f"[PARSE] Bad JSON on {topic}: {exc}")
        return {}

    result: dict = {}

    # ── Robot self-pose ───────────────────────────────────────────────────────
    robot = data.get("robot", {})
    result["robot_x"]       = float(robot.get("x",       0))
    result["robot_y"]       = float(robot.get("y",       0))
    result["robot_heading"] = float(robot.get("heading", 0))

    # ── Detected objects ──────────────────────────────────────────────────────
    objs = data.get("objects", {})

    result["cubes"] = [
        Cube(
            x      = float(c.get("x", 0)),
            y      = float(c.get("y", 0)),
            obj_id = str(c.get("id", "")),
            color  = str(c.get("color", "yellow")).lower(),
        )
        for c in objs.get("cubes", [])
    ]

    result["hills"] = [
        Hill(
            x      = float(h.get("x", 0)),
            y      = float(h.get("y", 0)),
            radius = float(h.get("radius", 40)),
        )
        for h in objs.get("hills", [])
    ]

    # Craters: BLACK TAPE — parsed identically to boundaries
    result["craters"] = [
        Crater(
            x      = float(cr.get("x", 0)),
            y      = float(cr.get("y", 0)),
            radius = float(cr.get("radius", 30)),
        )
        for cr in objs.get("craters", [])
    ]

    # Boundaries: BLACK TAPE — parsed identically to craters (same style)
    result["boundaries"] = [
        Boundary(
            x1 = float(b.get("x1", 0)),
            y1 = float(b.get("y1", 0)),
            x2 = float(b.get("x2", 0)),
            y2 = float(b.get("y2", 0)),
        )
        for b in objs.get("boundaries", [])
    ]

    return result


# ═══════════════════════════════════════════════════════════════════════════════
# PHASE 2 – DYNAMIC AUTO-SCALING CAMERA
# ═══════════════════════════════════════════════════════════════════════════════
#
# Problem: robots may report coordinates outside [0, ARENA_W/H] if the physical
# arena is larger than the initial constants, or if coordinate drift occurs.
#
# Solution: a Camera object tracks the bounding box of all seen coordinates.
# Whenever a new (x, y) is fed in, the viewport expands (never shrinks) to keep
# that point visible with a padding margin.  The world_to_screen() method
# replaces the old static a2s() logic; a2s() and a2r() now delegate here so
# all existing render functions continue to work with no signature changes.
#
# The viewport always maintains correct aspect ratio: the smaller scale factor
# (x vs. y axis) is used so nothing is clipped on either side.
# ─────────────────────────────────────────────────────────────────────────────

class Camera:
    """
    Maintains a monotonically-expanding axis-aligned viewport in arena coords.

    Initially covers [-ARENA_W/2, ARENA_W/2] × [-ARENA_H/2, ARENA_H/2] so that
    arena (0, 0) maps to the exact centre of the panel.
    Call feed(x, y) for every new data point; the viewport grows if needed.
    """
    _EDGE_PAD = 40.0   # arena units added around each new extreme point

    def __init__(self) -> None:
        # Centre the initial viewport on the arena origin so (0, 0) ↔ panel centre.
        self.x_min: float = -ARENA_W / 2.0
        self.x_max: float =  ARENA_W / 2.0
        self.y_min: float = -ARENA_H / 2.0
        self.y_max: float =  ARENA_H / 2.0

    def feed(self, x: float, y: float) -> None:
        """
        Expand the viewport so (x, y) is comfortably inside.
        The viewport never contracts — all past data always stays visible.
        """
        if x - self._EDGE_PAD < self.x_min:
            self.x_min = x - self._EDGE_PAD
        if x + self._EDGE_PAD > self.x_max:
            self.x_max = x + self._EDGE_PAD
        if y - self._EDGE_PAD < self.y_min:
            self.y_min = y - self._EDGE_PAD
        if y + self._EDGE_PAD > self.y_max:
            self.y_max = y + self._EDGE_PAD

    def _scale(self) -> float:
        """
        Pixels per arena unit.  Uses the more-constrained axis so the full
        viewport fits in PANEL_W × PANEL_H without clipping.
        """
        return min(
            PANEL_W / max(1.0, self.x_max - self.x_min),
            PANEL_H / max(1.0, self.y_max - self.y_min),
        )

    def world_to_screen(self, ax: float, ay: float) -> tuple:
        """
        Map arena (ax, ay) → screen (sx, sy).
        The scaled viewport is centred inside the fixed panel rectangle.
        Arena Y increases upward; screen Y increases downward (flipped here).
        """
        s  = self._scale()
        vw = (self.x_max - self.x_min) * s   # viewport width  in pixels at this zoom
        vh = (self.y_max - self.y_min) * s   # viewport height in pixels at this zoom

        # Centre the viewport within the panel by computing pixel offsets
        ox = (PANEL_W - vw) * 0.5
        oy = (PANEL_H - vh) * 0.5

        sx = int(PANEL_X + ox + (ax - self.x_min) * s)
        sy = int(PANEL_Y + oy + vh - (ay - self.y_min) * s)   # Y axis flip
        return sx, sy

    def scale_length(self, length: float) -> int:
        """Scale an arena-unit distance/radius to pixels."""
        return max(1, int(length * self._scale()))


# Module-level camera reference — set to a Camera() instance inside
# run_dashboard() before the render loop starts.  a2s() and a2r() read it.
# Using a module-level ref means all existing render functions call the Phase 2
# camera with zero signature changes.
_camera: Camera = None  # type: ignore


# ═══════════════════════════════════════════════════════════════════════════════
# COORDINATE HELPERS  (now backed by the Phase 2 Camera when active)
# ═══════════════════════════════════════════════════════════════════════════════

def a2s(ax: float, ay: float) -> tuple:
    """
    Arena-to-Screen coordinate transform.

    Delegates to _camera.world_to_screen() when the Phase 2 camera is active
    (i.e. run_dashboard has been entered).  Falls back to the original static
    formula so the function is still safe to call before initialisation.
    """
    if _camera is not None:
        return _camera.world_to_screen(ax, ay)
    # Static fallback: (0, 0) maps to panel centre, Y axis flipped.
    sx = int(PANEL_X + PANEL_W / 2.0 + (ax / ARENA_W) * PANEL_W)
    sy = int(PANEL_Y + PANEL_H / 2.0 - (ay / ARENA_H) * PANEL_H)
    return sx, sy


def a2r(length: float) -> int:
    """Scale an arena-unit length to a pixel radius/distance."""
    if _camera is not None:
        return _camera.scale_length(length)
    return max(1, int((length / ARENA_W) * PANEL_W))


# ═══════════════════════════════════════════════════════════════════════════════
# RENDERING
# ═══════════════════════════════════════════════════════════════════════════════

def render_arena_bg(surf: pygame.Surface) -> None:
    """
    Draw the arena floor, grid lines anchored to (0,0), and panel border.

    Grid lines radiate outward from the screen pixel that corresponds to arena
    (0, 0) — the logical origin.  This keeps the grid visually aligned with the
    coordinate system regardless of camera zoom or pan.  The centre axes are
    drawn one shade brighter so the origin is always identifiable at a glance.
    """
    panel = pygame.Rect(PANEL_X, PANEL_Y, PANEL_W, PANEL_H)
    pygame.draw.rect(surf, C_ARENA_FILL, panel)

    # Screen pixel of arena origin (0, 0) — the grid anchor point.
    # With the centred camera this lands exactly in the middle of the panel.
    ox, oy = a2s(0.0, 0.0)

    # Grid spacing: 50 arena units, scaled to pixels by the active camera.
    # Falls back to a fixed proportion if called before the camera is ready.
    GRID_STEP_ARENA = 50.0
    step_px = (
        _camera.scale_length(GRID_STEP_ARENA)
        if _camera is not None
        else max(4, int((GRID_STEP_ARENA / ARENA_W) * PANEL_W))
    )
    step_px = max(4, step_px)   # never narrower than 4 px

    # ── Vertical lines: radiate left from ox, then right from ox+step ─────────
    for start, direction in [(ox, -1), (ox + step_px, 1)]:
        x = start
        while PANEL_X <= x <= PANEL_X + PANEL_W:
            pygame.draw.line(surf, C_GRID, (x, PANEL_Y), (x, PANEL_Y + PANEL_H), 1)
            x += direction * step_px

    # ── Horizontal lines: radiate up from oy, then down from oy+step ──────────
    for start, direction in [(oy, -1), (oy + step_px, 1)]:
        y = start
        while PANEL_Y <= y <= PANEL_Y + PANEL_H:
            pygame.draw.line(surf, C_GRID, (PANEL_X, y), (PANEL_X + PANEL_W, y), 1)
            y += direction * step_px

    # ── Centre-axis crosshair: same grid colour but 1-shade brighter ──────────
    # Overdraws the regular grid lines at x=ox and y=oy so the origin stands out.
    C_AXIS = (155, 144, 118)   # slightly lighter than C_GRID
    if PANEL_X <= ox <= PANEL_X + PANEL_W:
        pygame.draw.line(surf, C_AXIS, (ox, PANEL_Y), (ox, PANEL_Y + PANEL_H), 1)
    if PANEL_Y <= oy <= PANEL_Y + PANEL_H:
        pygame.draw.line(surf, C_AXIS, (PANEL_X, oy), (PANEL_X + PANEL_W, oy), 1)

    pygame.draw.rect(surf, (70, 65, 55), panel, 2)   # thin border


def render_hills(surf: pygame.Surface, hills: List[Hill]) -> None:
    """Draw each hill as concentric green rings (topographic style)."""
    for h in hills:
        cx, cy = a2s(h.x, h.y)
        base_r = a2r(h.radius)
        for i in range(HILL_RINGS):
            factor = 1.0 - i * (0.28)
            r      = max(2, int(base_r * factor))
            color  = C_HILL_OUTER if i % 2 == 0 else C_HILL_INNER
            pygame.draw.circle(surf, color, (cx, cy), r, HILL_RING_W)


def render_boundaries(surf: pygame.Surface, boundaries: List[Boundary]) -> None:
    """
    Draw arena boundary segments.
    BLACK TAPE RULE: uses the exact same colour and stroke as craters.
    """
    for b in boundaries:
        pygame.draw.line(surf, C_BLACK_TAPE, a2s(b.x1, b.y1), a2s(b.x2, b.y2), BLACK_TAPE_WIDTH)


def render_craters(surf: pygame.Surface, craters: List[Crater]) -> None:
    """
    Draw craters as circular rings.
    BLACK TAPE RULE: uses the exact same colour and stroke as boundaries.
    """
    for c in craters:
        cx, cy = a2s(c.x, c.y)
        r = a2r(c.radius)
        pygame.draw.circle(surf, C_BLACK_TAPE, (cx, cy), r, BLACK_TAPE_WIDTH)


def render_cubes(surf: pygame.Surface, cubes: List[Cube], font_s: pygame.font.Font) -> None:
    """Draw cubes as coloured squares with an optional short label."""
    color_map = {
        "yellow": C_CUBE_YEL,
        "orange": C_CUBE_ORA,
        "red":    (210, 55, 55),
        "green":  (55, 190, 75),
        "blue":   (55, 120, 220),
        "purple": (160, 60, 200),
    }
    for cube in cubes:
        cx, cy = a2s(cube.x, cube.y)
        col    = color_map.get(cube.color, C_CUBE_YEL)
        rect   = pygame.Rect(cx - CUBE_HALF, cy - CUBE_HALF, CUBE_HALF * 2, CUBE_HALF * 2)
        pygame.draw.rect(surf, col,      rect)
        pygame.draw.rect(surf, (0, 0, 0), rect, 1)
        if cube.obj_id:
            lbl = font_s.render(cube.obj_id[:4], True, (0, 0, 0))
            surf.blit(lbl, (cx - CUBE_HALF, cy + CUBE_HALF + 1))


def render_robot(
    surf:    pygame.Surface,
    robot:   RobotPose,
    color:   tuple,
    label:   str,
    font:    pygame.font.Font,
) -> None:
    """
    Draw a robot as a filled circle with a white heading line and label.

    Robots with stale telemetry (> 5 s old) are drawn at 40 % opacity by
    blending toward the background colour.  A robot that has never reported
    (last_seen == 0) is treated as maximally stale.
    """
    rx, ry = a2s(robot.x, robot.y)
    age    = time.time() - robot.last_seen if robot.last_seen > 0 else 9999

    if age > 5.0:
        t     = min(1.0, (age - 5.0) / 10.0)   # 0→1 over 10 s after going stale
        blend = int(160 * (1.0 - t) + 40)       # alpha-like dim 160→40
        draw_color = tuple(int(c * blend / 255) for c in color)
    else:
        draw_color = color

    pygame.draw.circle(surf, draw_color, (rx, ry), ROBOT_RADIUS)
    pygame.draw.circle(surf, C_WHITE,    (rx, ry), ROBOT_RADIUS, 2)

    # Heading line: 0° = east (+x on screen), 90° = north (−y on screen)
    rad = math.radians(robot.heading)
    hx  = rx + int(HEADING_LEN * math.cos(rad))
    hy  = ry - int(HEADING_LEN * math.sin(rad))   # screen y is inverted
    pygame.draw.line(surf, C_WHITE, (rx, ry), (hx, hy), 3)

    lbl = font.render(label, True, C_WHITE)
    surf.blit(lbl, (rx - lbl.get_width() // 2, ry - ROBOT_RADIUS - 16))


# ── PHASE 2: Path History (Trails) ────────────────────────────────────────────
#
# Each robot accumulates a deque of (arena_x, arena_y) tuples (capped at
# TRAIL_MAX_LEN entries).  Here we draw those samples as small dots whose
# brightness ramps linearly from 20 % (oldest) to 70 % (newest), giving a
# natural visual fade without requiring per-pixel alpha compositing.
#
# The dots are drawn BEFORE the robot circles so the robot always appears on
# top of its own trail.
# ─────────────────────────────────────────────────────────────────────────────

def render_trails(
    surf:    pygame.Surface,
    trail_a: collections.deque,
    trail_b: collections.deque,
) -> None:
    """Draw fading exploration trails for Robot A (blue) and Robot B (red)."""

    def _draw(trail: collections.deque, base_color: tuple) -> None:
        n = len(trail)
        if n == 0:
            return
        for i, (ax, ay) in enumerate(trail):
            # Linear brightness: oldest sample at 20 %, newest at 70 %
            t      = i / max(1, n - 1)              # 0 = oldest, 1 = newest
            bright = 0.20 + 0.50 * t
            col    = tuple(int(c * bright) for c in base_color)
            sx, sy = a2s(ax, ay)
            # Clip rendering to the arena panel to avoid drawing outside it
            if PANEL_X <= sx <= PANEL_X + PANEL_W and PANEL_Y <= sy <= PANEL_Y + PANEL_H:
                pygame.draw.circle(surf, col, (sx, sy), TRAIL_DOT_RAD)

    _draw(trail_a, C_ROBOT_A)
    _draw(trail_b, C_ROBOT_B)


# ── PHASE 2: Live Telemetry HUD ───────────────────────────────────────────────
#
# A semi-transparent dark rectangle is composited over the bottom-left corner
# of the arena panel.  pygame.SRCALPHA allows the overlay surface to carry a
# per-pixel alpha channel so the arena floor shows through underneath.
#
# Three lines of monospace text show:
#   1. MQTT connection status  (green = connected, red = waiting)
#   2. Robot A: current (x, y) and heading
#   3. Robot B: current (x, y) and heading
# ─────────────────────────────────────────────────────────────────────────────

def render_hud(
    surf:      pygame.Surface,
    world:     WorldState,
    connected: bool,
    font_s:    pygame.font.Font,
) -> None:
    """Overlay a compact telemetry HUD on the arena panel."""

    # Build the three data lines as (text, colour) pairs
    lines = [
        (
            "MQTT: " + ("Connected" if connected else "Waiting for Data…"),
            C_OK if connected else C_ERR,
        ),
        (
            f"A  x={world.robot_a.x:7.1f}  y={world.robot_a.y:7.1f}  hdg={world.robot_a.heading:6.1f}°",
            C_ROBOT_A,
        ),
        (
            f"B  x={world.robot_b.x:7.1f}  y={world.robot_b.y:7.1f}  hdg={world.robot_b.heading:6.1f}°",
            C_ROBOT_B,
        ),
    ]

    pad   = 6     # internal padding (pixels) around the text block
    lh    = 16    # line height in pixels
    box_w = 330   # fixed width — wide enough for the longest formatted string
    box_h = pad * 2 + lh * len(lines)

    # Position: bottom-left corner of the arena panel, with a small inset
    box_x = PANEL_X + 6
    box_y = PANEL_Y + PANEL_H - box_h - 6

    # Create a surface with per-pixel alpha so the background is translucent.
    # Fill with a dark colour at ~70 % opacity (alpha 180 out of 255).
    overlay = pygame.Surface((box_w, box_h), pygame.SRCALPHA)
    overlay.fill((10, 10, 18, 180))
    surf.blit(overlay, (box_x, box_y))

    # Render each text line on top of the overlay
    for i, (text, color) in enumerate(lines):
        rendered = font_s.render(text, True, color)
        surf.blit(rendered, (box_x + pad, box_y + pad + i * lh))


def render_sidebar(
    surf:      pygame.Surface,
    world:     WorldState,
    connected: bool,
    font:      pygame.font.Font,
    font_s:    pygame.font.Font,
) -> None:
    """Right-hand information panel."""
    x  = PANEL_X + PANEL_W + 18
    y  = 55
    lh = 20   # line height

    def line(text: str, color: tuple = C_TEXT, bold: bool = False) -> None:
        nonlocal y
        f = font if bold else font_s
        surf.blit(f.render(text, True, color), (x, y))
        y += lh

    def gap(n: int = 1) -> None:
        nonlocal y
        y += lh * n

    # ── MQTT status ───────────────────────────────────────────────────────────
    line("MQTT",   bold=True)
    line(f"  {'Connected' if connected else 'Disconnected'}",
         C_OK if connected else C_ERR)
    line(f"  {MQTT_BROKER}")
    gap()

    # ── Robot A ───────────────────────────────────────────────────────────────
    line("Robot A", bold=True)
    a   = world.robot_a
    age = time.time() - a.last_seen if a.last_seen > 0 else None
    line(f"  x={a.x:7.1f}   y={a.y:7.1f}")
    line(f"  heading = {a.heading:6.1f}°")
    if age is None:
        line("  no data yet", C_ERR)
    else:
        stale = age > 5.0
        line(f"  last seen {age:.1f}s ago", C_ERR if stale else C_OK)
    gap()

    # ── Robot B ───────────────────────────────────────────────────────────────
    line("Robot B", bold=True)
    b   = world.robot_b
    age = time.time() - b.last_seen if b.last_seen > 0 else None
    line(f"  x={b.x:7.1f}   y={b.y:7.1f}")
    line(f"  heading = {b.heading:6.1f}°")
    if age is None:
        line("  no data yet", C_ERR)
    else:
        stale = age > 5.0
        line(f"  last seen {age:.1f}s ago", C_ERR if stale else C_OK)
    gap()

    # ── Object counts ─────────────────────────────────────────────────────────
    line("Objects",    bold=True)
    line(f"  Cubes      : {len(world.cubes)}")
    line(f"  Hills      : {len(world.hills)}")
    line(f"  Craters    : {len(world.craters)}")
    line(f"  Boundaries : {len(world.boundaries)}")
    gap()

    # ── Legend ────────────────────────────────────────────────────────────────
    line("Legend",         bold=True)
    line("  ● Red   = Robot A")
    line("  ● Blue  = Robot B")
    line("  □ Square= Cube")
    line("  ◎ Green = Hill")
    line("  ━━  Black tape:")
    line("     Crater = Boundary")
    gap()

    line("Keys:", bold=True)
    line("  ESC  quit")
    line("  R    reset robots")
    line("  C    clear trails")    # PHASE 2: new key hint


# ═══════════════════════════════════════════════════════════════════════════════
# MQTT BRIDGE
# ═══════════════════════════════════════════════════════════════════════════════

class MQTTBridge:
    """
    Thin wrapper around paho-mqtt.

    The paho client is started with client.loop_start(), which spawns a daemon
    thread that handles:
        • socket I/O (recv / send)
        • PING/PONG keepalive
        • Automatic reconnection after drops
        • Dispatching on_message callbacks

    The main (pygame) thread never calls loop() or loop_forever().  Data is
    exchanged via a threading.Lock-protected dict so neither thread blocks the
    other longer than a single dict copy.
    """

    def __init__(self, broker: str, port: int, username: str, password: str):
        self._broker    = broker
        self._port      = port
        self.connected  = False

        self._lock       = threading.Lock()
        self._inbox: dict = {}   # topic → {"parsed": dict, "timestamp": float}

        # paho ≥ 2.0 added a mandatory callback_api_version argument.
        # We try the new API first and fall back to the legacy constructor.
        try:
            self._client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id="arena_dashboard",
            )
        except AttributeError:
            # paho < 2.0
            self._client = mqtt.Client(client_id="arena_dashboard")

        self._client.username_pw_set(username, password)
        self._client.on_connect    = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message    = self._on_message

    # ── paho callbacks  (all called from the background thread) ───────────────

    def _on_connect(self, client, userdata, flags, reason_code, *args) -> None:
        rc = reason_code if isinstance(reason_code, int) else reason_code.value
        if rc == 0:
            self.connected = True
            print(f"[MQTT] Connected to {self._broker}:{self._port}")
            # Subscribe to both robot telemetry topics
            client.subscribe(TOPIC_A_SEND)
            client.subscribe(TOPIC_B_SEND)
            print(f"[MQTT] Subscribed to {TOPIC_A_SEND}, {TOPIC_B_SEND}")
        else:
            print(f"[MQTT] Connection refused (rc={rc})")

    def _on_disconnect(self, client, userdata, disconnect_flags, reason_code, *args) -> None:
        self.connected = False
        print(f"[MQTT] Disconnected (rc={reason_code}) — paho will auto-reconnect")

    def _on_message(self, client, userdata, msg) -> None:
        """Receive a message and store the parsed result thread-safely."""
        topic   = msg.topic
        payload = msg.payload.decode("utf-8", errors="replace")
        print(f"[MQTT] ← {topic}  {payload[:100]}")

        parsed = parse_mqtt_message(topic, payload)
        if not parsed:
            return

        # Hold the lock only for the minimal dict-update operation
        with self._lock:
            self._inbox[topic] = {"parsed": parsed, "timestamp": time.time()}

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Connect to the broker and start the background network loop."""
        print(f"[MQTT] Connecting to {self._broker}:{self._port} …")
        try:
            self._client.connect(self._broker, self._port, keepalive=60)
        except OSError as exc:
            print(f"[MQTT] connect() failed: {exc}")
            return

        # loop_start() is the key call: it launches a thread and returns
        # immediately, leaving the caller (pygame main loop) unblocked.
        self._client.loop_start()

    def stop(self) -> None:
        """Graceful shutdown: stop the background thread, then disconnect."""
        self._client.loop_stop()
        self._client.disconnect()

    def publish(self, topic: str, payload: dict) -> None:
        """Send a JSON command to a robot (safe to call from any thread)."""
        self._client.publish(topic, json.dumps(payload))

    def snapshot(self) -> dict:
        """
        Return a copy of the latest parsed inbox (thread-safe).
        Call this from the render loop to read the latest telemetry without
        holding the lock during rendering.
        """
        with self._lock:
            return dict(self._inbox)


# ═══════════════════════════════════════════════════════════════════════════════
# WORLD-STATE MERGER
# ═══════════════════════════════════════════════════════════════════════════════

def merge_into_world(inbox: dict, world: WorldState) -> None:
    """
    Apply the latest MQTT snapshots to the shared WorldState.

    Each robot reports its own pose.  Object lists are "last writer wins" —
    the most recently received non-empty list from either robot overwrites
    the stored list.  Replace this function with a smarter merge strategy
    (e.g. SLAM fusion) when needed.
    """
    if TOPIC_A_SEND in inbox:
        entry = inbox[TOPIC_A_SEND]
        d     = entry["parsed"]
        world.robot_a.x         = d.get("robot_x",       world.robot_a.x)
        world.robot_a.y         = d.get("robot_y",       world.robot_a.y)
        world.robot_a.heading   = d.get("robot_heading", world.robot_a.heading)
        world.robot_a.last_seen = entry["timestamp"]
        _apply_objects(world, d)

    if TOPIC_B_SEND in inbox:
        entry = inbox[TOPIC_B_SEND]
        d     = entry["parsed"]
        world.robot_b.x         = d.get("robot_x",       world.robot_b.x)
        world.robot_b.y         = d.get("robot_y",       world.robot_b.y)
        world.robot_b.heading   = d.get("robot_heading", world.robot_b.heading)
        world.robot_b.last_seen = entry["timestamp"]
        _apply_objects(world, d)


def _apply_objects(world: WorldState, parsed: dict) -> None:
    if parsed.get("cubes"):
        world.cubes = parsed["cubes"]
    if parsed.get("hills"):
        world.hills = parsed["hills"]
    if parsed.get("craters"):
        world.craters = parsed["craters"]
    if parsed.get("boundaries"):
        world.boundaries = parsed["boundaries"]


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════════════

def run_dashboard() -> None:
    # ── PHASE 2: Expose the camera globally so a2s()/a2r() can use it ─────────
    # run_dashboard() is the only entry point that owns the Camera; using a
    # global variable lets all render helpers pick it up without changing their
    # function signatures.
    global _camera

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Robot Arena Dashboard")
    clock = pygame.time.Clock()

    font_title = pygame.font.SysFont("monospace", 17, bold=True)
    font_label = pygame.font.SysFont("monospace", 13, bold=True)
    font_small = pygame.font.SysFont("monospace", 11)

    world  = WorldState()

    # Set non-overlapping initial poses so the robots are distinguishable before
    # the first MQTT packet arrives.  These values are overwritten the moment
    # merge_into_world() receives real telemetry (last_seen stays 0 until then,
    # so the robots render in the "stale" dimmed style as a visual hint).
    #
    # Heading convention: 0 = East, 90 = North, 270 = South (CCW positive).
    # The render_robot() formula is:  hy = ry - HEADING_LEN * sin(heading_rad)
    # so heading=90  → hy decreases → line points UP   (North on screen) ✓
    #    heading=270 → hy increases → line points DOWN  (South on screen) ✓
    world.robot_a.x       =   0.0
    world.robot_a.y       = -30.0   # slightly below origin
    world.robot_a.heading =  90.0   # facing North (up)

    world.robot_b.x       =   0.0
    world.robot_b.y       =  30.0   # slightly above origin
    world.robot_b.heading = 270.0   # facing South (down)

    bridge = MQTTBridge(MQTT_BROKER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD)
    bridge.start()

    # ── PHASE 2: Initialise camera and trail state ─────────────────────────────
    _camera = Camera()   # activate the dynamic viewport (a2s/a2r now use this)

    # One deque per robot; maxlen enforces the ring-buffer cap automatically.
    trail_a: collections.deque = collections.deque(maxlen=TRAIL_MAX_LEN)
    trail_b: collections.deque = collections.deque(maxlen=TRAIL_MAX_LEN)

    # Track last-seen timestamps so we only append a new trail point when the
    # MQTT layer delivers genuinely fresh telemetry (not on every render frame).
    last_seen_a: float = 0.0
    last_seen_b: float = 0.0
    # ──────────────────────────────────────────────────────────────────────────

    running = True
    while running:
        # ── Event handling ────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    bridge.publish(TOPIC_A_RECV, {"cmd": "reset"})
                    bridge.publish(TOPIC_B_RECV, {"cmd": "reset"})
                    print("[CMD] reset sent to both robots")
                # PHASE 2: C key clears both trails (useful after a robot is
                # repositioned manually so old paths don't confuse the map)
                elif event.key == pygame.K_c:
                    trail_a.clear()
                    trail_b.clear()
                    print("[UI] trails cleared")

        # ── Pull MQTT data (one lock acquisition, then release) ───────────────
        merge_into_world(bridge.snapshot(), world)

        # ── PHASE 2: Update trail history and camera viewport ─────────────────
        # We only record a new sample when last_seen changed, which means a new
        # MQTT packet arrived since the previous frame.  This prevents filling
        # the trail with identical duplicate points while a robot is stationary.

        if world.robot_a.last_seen > last_seen_a and world.robot_a.last_seen > 0:
            # Append the new position to Robot A's history ring-buffer
            trail_a.append((world.robot_a.x, world.robot_a.y))
            # Feed the camera: if this point is near or outside the edge the
            # viewport will expand so the robot stays visible
            _camera.feed(world.robot_a.x, world.robot_a.y)
            last_seen_a = world.robot_a.last_seen

        if world.robot_b.last_seen > last_seen_b and world.robot_b.last_seen > 0:
            trail_b.append((world.robot_b.x, world.robot_b.y))
            _camera.feed(world.robot_b.x, world.robot_b.y)
            last_seen_b = world.robot_b.last_seen

        # Also feed detected object positions into the camera so cubes/craters
        # near the arena edge keep the viewport correctly sized
        for cube in world.cubes:
            _camera.feed(cube.x, cube.y)
        for hill in world.hills:
            _camera.feed(hill.x, hill.y)
        for crater in world.craters:
            _camera.feed(crater.x, crater.y)
        # ──────────────────────────────────────────────────────────────────────

        # ── Render ────────────────────────────────────────────────────────────
        screen.fill(C_BG)

        # Title bar
        title = font_title.render(
            "Robot Arena Dashboard   [ESC = quit  |  R = reset  |  C = clear trails]",
            True, C_TEXT,
        )
        screen.blit(title, (10, 14))

        # Arena (draw order: bg → hills → boundaries → craters → trails → cubes → robots → HUD)
        render_arena_bg(screen)
        render_hills(screen, world.hills)
        render_boundaries(screen, world.boundaries)
        render_craters(screen, world.craters)

        # PHASE 2: draw trails before robots so robots render on top
        render_trails(screen, trail_a, trail_b)

        render_cubes(screen, world.cubes, font_small)
        render_robot(screen, world.robot_a, C_ROBOT_A, "A", font_label)
        render_robot(screen, world.robot_b, C_ROBOT_B, "B", font_label)

        # PHASE 2: HUD overlaid last so it always sits above the arena content
        render_hud(screen, world, bridge.connected, font_small)

        # Info sidebar (unchanged from Phase 1)
        render_sidebar(screen, world, bridge.connected, font_label, font_small)

        pygame.display.flip()
        clock.tick(FPS)

    bridge.stop()
    pygame.quit()


# ═══════════════════════════════════════════════════════════════════════════════
# MOCK PUBLISHER  (python main.py --mock)
# ═══════════════════════════════════════════════════════════════════════════════
# Run this in a second terminal to simulate robots without real hardware:
#
#     python main.py --mock
#
# It publishes a moving Robot A (circular orbit) and a drifting Robot B, plus a
# static set of cubes, hills, a crater, and arena boundaries.
# ─────────────────────────────────────────────────────────────────────────────

def run_mock_publisher() -> None:
    print("[MOCK] Starting mock publisher — Ctrl-C to stop")
    try:
        pub = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id="mock_pub",
        )
    except AttributeError:
        pub = mqtt.Client(client_id="mock_pub")

    pub.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    try:
        pub.connect(MQTT_BROKER, MQTT_PORT)
    except OSError as exc:
        print(f"[MOCK] Cannot connect: {exc}")
        return

    pub.loop_start()

    # All coordinates are now origin-centred: (0, 0) is the arena centre.
    # Old values (bottom-left origin) had centre at (250, 250); all positions
    # here are the old values minus 250 on each axis.
    static_objects = {
        "cubes": [
            {"id": "C1", "x": -160, "y": -155, "color": "yellow"},
            {"id": "C2", "x":  140, "y":  160, "color": "orange"},
            {"id": "C3", "x":  -30, "y":  190, "color": "green"},
        ],
        "hills": [
            {"x":  60, "y":  -90, "radius": 65},
            {"x": -120, "y":  80, "radius": 45},
        ],
        "craters": [
            {"x": 170, "y": 30, "radius": 38},
        ],
        # Boundary square: ±235 units from origin (≈ 15 units inside ±250 edge)
        "boundaries": [
            {"x1": -235, "y1": -235, "x2":  235, "y2": -235},
            {"x1":  235, "y1": -235, "x2":  235, "y2":  235},
            {"x1":  235, "y1":  235, "x2": -235, "y2":  235},
            {"x1": -235, "y1":  235, "x2": -235, "y2": -235},
        ],
    }

    t = 0.0
    try:
        while True:
            t += 0.05

            # Robot A: circular orbit centred on the origin (0, 0)
            payload_a = json.dumps({
                "robot": {
                    "x":       90 * math.cos(t),
                    "y":       90 * math.sin(t),
                    "heading": math.degrees(t + math.pi / 2) % 360,
                },
                "objects": static_objects,
            })

            # Robot B: slow Lissajous figure centred slightly above origin
            payload_b = json.dumps({
                "robot": {
                    "x":       60 * math.sin(t * 0.6),
                    "y":       90 + 45 * math.cos(t * 0.9),
                    "heading": (math.degrees(t * 0.4) + 200) % 360,
                },
                "objects": {},   # B reports no objects in this mock
            })

            pub.publish(TOPIC_A_SEND, payload_a)
            pub.publish(TOPIC_B_SEND, payload_b)
            time.sleep(0.15)

    except KeyboardInterrupt:
        print("[MOCK] Stopped.")
    finally:
        pub.loop_stop()
        pub.disconnect()


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    if "--mock" in sys.argv:
        run_mock_publisher()
    else:
        run_dashboard()
