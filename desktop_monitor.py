#!/usr/bin/env python3
"""
Venus desktop monitor -- ADAPTED FOR OUR ROVER (frontier explorer).

This is the team's tkinter monitor reworked to speak OUR robot's protocol.
Instead of the TU/e MQTT broker it connects to the SAME stream the web UI uses:
ws_bridge.py's WebSocket (ws://<pynq>:8765), which forwards the firmware's
newline-delimited JSON. Pure standard library -- no paho / websocket-client.

  Robot -> monitor :  {"type":"pose","x":..,"y":..,"theta":..}
                      {"type":"plan","x":..,"y":..}                (frontier goal)
                      {"type":"observation","kind":"mountain"|"black_tape","x":..,"y":..}
                      {"type":"observation","kind":"scannable","color":..,"x":..,"y":..,"size":1|2}
                      {"type":"status","mode":..,"detail":..}
  Monitor -> robot :  {"type":"mode","mode":"explore"}   (EXPLORE button)
                      {"type":"mode","mode":"stop"}       (STOP button)

Usage:   python3 desktop_monitor.py [ws://host:port]
         (default ws://10.43.0.1:8765 -- the Pynq AP address)
"""
import base64
import json
import math
import os
import queue
import socket
import struct
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from typing import Optional

# ── Connection ───────────────────────────────────────────────────────────────
WS_URL = sys.argv[1] if len(sys.argv) > 1 else "ws://10.43.0.1:8765"

# ── Map / display parameters ─────────────────────────────────────────────────
MAP_M   = 1.4          # initial map extent in metres (~our 1.2 m arena + margin)
STALE_S = 6.0          # seconds before "last message" turns red
MAP_ROTATION_DEG = 0.0 # our firmware uses standard math coords (x right, y up); no rotation

WALL_GRID_M    = 0.03  # mountain points snapped to a 3 cm grid (dedup) = our cell size
WALL_MAX_PTS   = 6000
ROCK_DEDUP_M   = 0.06  # merge rock detections closer than this
BSEG_JOIN_M    = 0.12  # join consecutive tape points into one polyline if closer than this
BSEG_MIN_GAP_M = 0.02  # ignore tape points closer than this to the last one

ROCK_COLORS = {
    "red": "#cc3333", "green": "#33aa33", "blue": "#3366cc",
    "yellow": "#d6b400", "white": "#dddddd", "black": "#222222",
    "unknown": "#999999", "none": "#999999",
}
EVENT_COLORS = {"boundary": "#ff8800", "obstacle": "#884400", "cliff": "#cc3333"}
EVENT_DEDUP_M  = 0.05    # merge events closer than this
CORNER_MIN_SIDE_M = 0.20  # only show a derived rectangle once the tape spans this much


# ════════════════════════ STATE ════════════════════════
@dataclass
class RobotState:
    status:     str             = "waiting"
    ready:      bool            = False
    x:          float           = 0.0
    y:          float           = 0.0
    theta:      float           = 0.0
    moving:     bool            = False
    last_seen:  Optional[float] = None
    last_color: str             = "None"
    # live sensors (from telemetry)
    tof_bottom_mm: Optional[int] = None
    tof_middle_mm: Optional[int] = None
    tof_top_mm:    Optional[int] = None
    ir:         list            = field(default_factory=lambda: [0, 0])  # [left, right]
    temp_c:     Optional[float] = None
    # map layers
    path:       list            = field(default_factory=list)   # [(x,y), …]
    rocks:      list            = field(default_factory=list)   # [{x,y,color,size_mm,temp_c,robot}]
    events:     list            = field(default_factory=list)   # [{x,y,event}]
    bsegs:      list            = field(default_factory=list)   # tape polylines [[(x,y),…]]
    walls:      set             = field(default_factory=set)    # mountain grid cells
    goal:       Optional[tuple] = None                          # current frontier target
    # boundary derived from the tape cloud
    corners:    list            = field(default_factory=list)   # [{x,y,n}]
    rect_dims:  Optional[tuple] = None                          # (w_m, h_m, area_m2)
    tape_bbox:  Optional[list]  = None                          # [minx,miny,maxx,maxy]


# ════════════════════════ GEOMETRY ════════════════════════
_rot_rad = math.radians(MAP_ROTATION_DEG)
_rot_cos = math.cos(_rot_rad)
_rot_sin = math.sin(_rot_rad)


def rotate_point(x: float, y: float) -> tuple:
    return (x * _rot_cos - y * _rot_sin, x * _rot_sin + y * _rot_cos)


def rotate_angle(theta: float) -> float:
    return theta + _rot_rad


# ════════════════════════ PROTOCOL DECODE (testable, no GUI) ════════════════════════
def _f(d, key, fallback=None):
    """float(d[key]) or fallback, tolerating missing / bad values."""
    v = d.get(key)
    if v is None:
        return fallback
    try:
        return float(v)
    except (ValueError, TypeError):
        return fallback


def apply_message(s: RobotState, d: dict, log=lambda *_: None) -> None:
    """Fold ONE decoded JSON message from our firmware into the state.
    Pure logic (no tkinter) so it can be unit-tested headless."""
    if not isinstance(d, dict):
        return
    t = d.get("type")
    if t not in ("pose", "plan", "status", "observation", "telemetry"):
        return                                  # ignore unknown types (don't refresh freshness)
    s.last_seen = time.time()

    if t == "pose":
        x = _f(d, "x"); y = _f(d, "y")
        if x is None or y is None:
            return
        s.x, s.y = x, y
        th = _f(d, "theta")
        if th is not None:
            s.theta = th
        pt = (round(s.x, 4), round(s.y, 4))
        if not s.path or s.path[-1] != pt:
            s.path.append(pt)

    elif t == "plan":
        x = _f(d, "x"); y = _f(d, "y")
        if x is not None and y is not None:
            s.goal = (x, y)

    elif t == "status":
        mode = str(d.get("mode", ""))
        detail = str(d.get("detail", ""))
        s.ready = True
        # the firmware sends "alive" every ~2 s -- a no-op for the label so it
        # never clobbers the last meaningful detail (rock_detected,
        # exploration_complete, target_reached, …).
        if detail == "alive":
            pass
        elif detail:
            s.status = detail
            log(f"[STATE] {mode}/{detail}")
        elif mode:
            s.status = mode
        # motion still tracks the live mode (any active driving mode)
        s.moving = mode in ("explore", "target", "manual")

    elif t == "telemetry":
        _apply_telemetry(s, d)

    elif t == "observation":
        kind = str(d.get("kind", ""))
        x = _f(d, "x"); y = _f(d, "y")
        if x is None or y is None:
            return
        if kind == "scannable":
            _apply_rock(s, x, y, d, log)
        elif kind == "mountain":
            if len(s.walls) < WALL_MAX_PTS:
                s.walls.add((round(x / WALL_GRID_M), round(y / WALL_GRID_M)))
            _add_event(s, x, y, "obstacle")
        elif kind == "black_tape":
            _apply_tape(s, x, y)
            _add_event(s, x, y, "boundary")
            _update_boundary(s, x, y)           # derive arena corners + dimensions


def _apply_rock(s: RobotState, x, y, d, log) -> None:
    color = str(d.get("color", "unknown")).lower()
    robot = str(d.get("robot", "self")).lower()  # "self" (us) vs "partner"
    has_size = "size" in d                        # partner channel omits size
    size = int(_f(d, "size", 1) or 1)
    size_mm = 60 if size >= 2 else 30             # 6x6 cm large vs 3x3 cm small
    if color in ("", "none", "unknown") or color not in ROCK_COLORS:
        return
    for r in s.rocks:                             # dedup / merge -- only within the SAME robot
        if r.get("robot", "self") == robot and math.hypot(r["x"] - x, r["y"] - y) < ROCK_DEDUP_M:
            r["x"] = (r["x"] + x) / 2.0
            r["y"] = (r["y"] + y) / 2.0
            r["color"] = color
            if has_size:                          # don't overwrite a known size with a guess
                r["size_mm"] = size_mm
                r["size_known"] = True
            s.last_color = color.upper()          # re-detections update the label too
            return
    temp_c = _f(d, "temp_c")
    s.rocks.append({"x": x, "y": y, "color": color, "size_mm": size_mm,
                    "robot": robot, "size_known": has_size, "temp_c": temp_c})
    s.last_color = color.upper()
    tag = "" if robot == "self" else " (partner)"
    sz = ("large" if size_mm >= 60 else "small") if has_size else "size?"
    tt = f"  {temp_c:.0f}°C" if temp_c is not None else ""
    log(f"[ROCK]  {color:6s} {sz}{tag}{tt} @ ({x:.2f}, {y:.2f}) m")


def _apply_tape(s: RobotState, x, y) -> None:
    if s.bsegs:
        lx, ly = s.bsegs[-1][-1]
        dd = math.hypot(x - lx, y - ly)
        if dd < BSEG_MIN_GAP_M:
            return
        if dd < BSEG_JOIN_M:
            s.bsegs[-1].append((x, y))
            return
    s.bsegs.append([(x, y)])


def _apply_telemetry(s: RobotState, d: dict) -> None:
    b = _f(d, "tof_bottom_mm"); m = _f(d, "tof_middle_mm"); t = _f(d, "tof_top_mm")
    s.tof_bottom_mm = int(b) if b else None
    s.tof_middle_mm = int(m) if m else None
    s.tof_top_mm    = int(t) if t else None
    s.ir = [1 if d.get("ir_left") else 0, 1 if d.get("ir_right") else 0]
    tc = _f(d, "temp_c")
    if tc is not None:
        s.temp_c = tc
    if isinstance(d.get("moving"), bool):
        s.moving = d["moving"]


def _add_event(s: RobotState, x, y, etype: str) -> None:
    for e in s.events:
        if e["event"] == etype and math.hypot(e["x"] - x, e["y"] - y) < EVENT_DEDUP_M:
            return
    s.events.append({"x": x, "y": y, "event": etype})


def _update_boundary(s: RobotState, x, y) -> None:
    """Grow the tape bounding box and derive the arena rectangle + 4 corners."""
    if s.tape_bbox is None:
        s.tape_bbox = [x, y, x, y]
    else:
        bb = s.tape_bbox
        bb[0] = min(bb[0], x); bb[1] = min(bb[1], y)
        bb[2] = max(bb[2], x); bb[3] = max(bb[3], y)
    minx, miny, maxx, maxy = s.tape_bbox
    w, h = maxx - minx, maxy - miny
    if w >= CORNER_MIN_SIDE_M and h >= CORNER_MIN_SIDE_M:
        s.corners = [{"x": minx, "y": miny, "n": 1}, {"x": maxx, "y": miny, "n": 2},
                     {"x": maxx, "y": maxy, "n": 3}, {"x": minx, "y": maxy, "n": 4}]
        s.rect_dims = (w, h, w * h)


# ════════════════════════ WEBSOCKET CLIENT (pure stdlib) ════════════════════════
class WSClient:
    """Minimal RFC-6455 text client. Auto-reconnects. on_line(str) is called
    from a background thread for every JSON line received."""

    def __init__(self, url: str, on_line, on_status):
        self.host, self.port, self.path = self._parse(url)
        self.on_line = on_line
        self.on_status = on_status
        self.sock: Optional[socket.socket] = None
        self._rbuf = b""                       # leftover bytes after the handshake / between frames
        self._send_lock = threading.Lock()
        self._running = True
        self.connected = False
        threading.Thread(target=self._run, daemon=True).start()

    @staticmethod
    def _parse(url: str):
        u = url.strip()
        if u.startswith("ws://"):
            u = u[5:]
        elif u.startswith("wss://"):
            u = u[6:]
        path = "/"
        if "/" in u:
            u, path = u.split("/", 1)
            path = "/" + path
        host, _, port = u.partition(":")
        return host, int(port or 8765), path

    def _recv_exact(self, sock, n):
        """Read exactly n bytes, draining the leftover buffer first (so frames
        that arrived glued to the HTTP handshake aren't lost / desynced)."""
        while len(self._rbuf) < n:
            chunk = sock.recv(4096)
            if not chunk:
                return None
            self._rbuf += chunk
        out, self._rbuf = self._rbuf[:n], self._rbuf[n:]
        return out

    def _handshake(self, sock):
        key = base64.b64encode(os.urandom(16)).decode()
        req = ("GET {p} HTTP/1.1\r\n"
               "Host: {h}:{port}\r\n"
               "Upgrade: websocket\r\n"
               "Connection: Upgrade\r\n"
               "Sec-WebSocket-Key: {k}\r\n"
               "Sec-WebSocket-Version: 13\r\n\r\n"
               ).format(p=self.path, h=self.host, port=self.port, k=key)
        sock.sendall(req.encode())
        resp = b""
        while b"\r\n\r\n" not in resp:
            chunk = sock.recv(1024)
            if not chunk:
                raise OSError("handshake closed")
            resp += chunk
            if len(resp) > 8192:
                raise OSError("handshake too long")
        head, _, tail = resp.partition(b"\r\n\r\n")
        if b" 101 " not in head.split(b"\r\n", 1)[0] + b" ":
            raise OSError("no 101 upgrade: " + head.split(b"\r\n", 1)[0].decode("latin1"))
        self._rbuf = tail            # frames that arrived glued to the response

    MAX_FRAME = 8 * 1024 * 1024              # reject absurd frame lengths

    def _read_frame(self, sock):
        """Returns (fin, opcode, payload bytes) or None on close/error."""
        hdr = self._recv_exact(sock, 2)
        if hdr is None:
            return None
        fin = bool(hdr[0] & 0x80)
        opcode = hdr[0] & 0x0F
        masked = hdr[1] & 0x80
        length = hdr[1] & 0x7F
        if length == 126:
            ext = self._recv_exact(sock, 2)
            if ext is None:
                return None
            length = struct.unpack(">H", ext)[0]
        elif length == 127:
            ext = self._recv_exact(sock, 8)
            if ext is None:
                return None
            length = struct.unpack(">Q", ext)[0]
        if length > self.MAX_FRAME:
            raise OSError("frame too large: %d" % length)
        mask = self._recv_exact(sock, 4) if masked else None
        payload = self._recv_exact(sock, length) if length else b""
        if payload is None or (masked and mask is None):
            return None
        if masked:
            payload = bytes(payload[i] ^ mask[i % 4] for i in range(length))
        return fin, opcode, payload

    def _read_loop(self, sock):
        msg = bytearray()                        # reassembly buffer for fragmented text
        while self._running:
            frame = self._read_frame(sock)
            if frame is None:
                break
            fin, opcode, payload = frame
            if opcode == 0x8:                    # close
                break
            if opcode == 0x9:                    # ping -> pong (control payload <= 125 B)
                self._send_frame(sock, 0xA, payload[:125])
                continue
            if opcode == 0xA:                    # pong -> ignore
                continue
            if opcode in (0x1, 0x0):             # text start / continuation
                msg += payload
                if fin:                          # whole message assembled -> decode ONCE
                    text = msg.decode("utf-8", "replace")
                    msg = bytearray()
                    for line in text.split("\n"):
                        line = line.strip()
                        if line:
                            self.on_line(line)

    def _send_frame(self, sock, opcode, payload: bytes):
        n = len(payload)
        header = bytearray([0x80 | opcode])      # FIN + opcode
        if n < 126:
            header.append(0x80 | n)
        elif n < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", n)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", n)
        mask = os.urandom(4)
        header += mask
        masked = bytes(payload[i] ^ mask[i % 4] for i in range(n))
        with self._send_lock:
            sock.sendall(bytes(header) + masked)

    def send_json(self, obj: dict) -> bool:
        sock = self.sock
        if sock is None or not self.connected:
            return False
        try:
            self._send_frame(sock, 0x1, json.dumps(obj).encode("utf-8"))
            return True
        except OSError:
            return False

    def _run(self):
        backoff = 1.0
        while self._running:
            try:
                self._rbuf = b""                  # fresh buffer per connection
                sock = socket.create_connection((self.host, self.port), timeout=6)
                sock.settimeout(None)
                self._handshake(sock)
                self.sock = sock
                self.connected = True
                self.on_status(True, "connected to {}:{}".format(self.host, self.port))
                backoff = 1.0
                self._read_loop(sock)
            except Exception as exc:                      # noqa: BLE001 (report any failure)
                self.on_status(False, "error: {}".format(exc))
            self.connected = False
            self.sock = None
            if not self._running:
                break
            self.on_status(False, "reconnecting in {:.0f}s…".format(backoff))
            time.sleep(backoff)
            backoff = min(backoff * 2, 10.0)

    def close(self):
        self._running = False
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass


# ════════════════════════ APP (GUI) ════════════════════════
class App:
    def __init__(self) -> None:
        self.state = RobotState()
        self._q: queue.Queue = queue.Queue()
        self._ws: Optional[WSClient] = None
        self._connected = False

        self._zoom = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._drag_start = None
        self._drag_pan_start = None

        self.root = tk.Tk()
        self.root.title("Venus Monitor — our rover")
        self.root.resizable(True, True)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self._build_ui()

    # ── UI ──
    def _build_ui(self) -> None:
        info = tk.Frame(self.root, padx=10, pady=8)
        info.pack(fill=tk.X)

        self.lbl_conn   = tk.Label(info, text="Link: connecting…", anchor="w", fg="orange")
        self.lbl_robot  = tk.Label(info, text="Robot: waiting", anchor="w")
        self.lbl_coords = tk.Label(info, text="Position: x=0.000 m  y=0.000 m  θ=0.000 rad", anchor="w")
        self.lbl_dist   = tk.Label(info, text="ToF  bottom: —  middle: —  top: —", anchor="w")
        self.lbl_temp   = tk.Label(info, text="Temp: —", anchor="w")
        self.lbl_ir     = tk.Label(info, text="IR tape:  ○ ○   (left  right)",
                                   anchor="w", font=("TkFixedFont", 9))
        self.lbl_goal   = tk.Label(info, text="Frontier goal: —", anchor="w", fg="#bb6600")
        self.lbl_rocks  = tk.Label(info, text="Rocks: 0", anchor="w")
        self.lbl_last_color = tk.Label(info, text="Latest rock: None", anchor="w",
                                       fg="#336699", font=("TkDefaultFont", 9, "bold"))
        self.lbl_rect   = tk.Label(info, text="Boundary: not traced yet", anchor="w",
                                   fg="#555555", font=("TkDefaultFont", 9, "bold"))
        self.lbl_map    = tk.Label(info, text="Mountains: 0   Tape pts: 0", anchor="w", fg="#555555")
        self.lbl_motion = tk.Label(info, text="Motion: —", anchor="w")
        self.lbl_age    = tk.Label(info, text="Last message: never", anchor="w", fg="gray")

        ctrl = tk.Frame(info)
        self.btn_start = tk.Button(ctrl, text="EXPLORE", command=self._send_explore,
                                   width=14, state=tk.DISABLED,
                                   bg="#33cc33", fg="white", activebackground="#229922")
        self.btn_stop  = tk.Button(ctrl, text="STOP", command=self._send_stop,
                                   width=14, state=tk.DISABLED,
                                   bg="#cc3333", fg="white", activebackground="#991111")
        self.btn_reconn = tk.Button(ctrl, text="⟳ Reconnect", command=self._reconnect,
                                    width=14, bg="#885522", fg="white", activebackground="#663311")
        self.btn_save  = tk.Button(ctrl, text="💾 Save map", command=self._save_map,
                                   width=14, bg="#446688", fg="white", activebackground="#335577")
        for b in (self.btn_start, self.btn_stop, self.btn_reconn, self.btn_save):
            b.pack(fill=tk.X, pady=1)

        labels = [self.lbl_conn, self.lbl_robot, self.lbl_coords, self.lbl_dist, self.lbl_temp,
                  self.lbl_ir, self.lbl_goal, self.lbl_rocks, self.lbl_last_color,
                  self.lbl_rect, self.lbl_map, self.lbl_motion, self.lbl_age]
        for row, w in enumerate(labels):
            w.grid(row=row, column=0, sticky="w")
        ctrl.grid(row=0, column=1, rowspan=len(labels), padx=(20, 0), sticky="ne")
        info.columnconfigure(0, weight=1)

        self.cw, self.ch = 760, 460
        self._margin = 40
        self._scale = (min(self.cw, self.ch) - 2 * self._margin) / MAP_M

        zoom_bar = tk.Frame(self.root, padx=10, pady=2)
        zoom_bar.pack(fill=tk.X)
        tk.Label(zoom_bar, text="Map:", fg="#555").pack(side=tk.LEFT)
        tk.Button(zoom_bar, text=" + ", command=lambda: self._zoom_step(1.35),
                  width=3, relief=tk.FLAT, bg="#dddddd").pack(side=tk.LEFT, padx=1)
        tk.Button(zoom_bar, text=" − ", command=lambda: self._zoom_step(1 / 1.35),
                  width=3, relief=tk.FLAT, bg="#dddddd").pack(side=tk.LEFT, padx=1)
        tk.Button(zoom_bar, text="Fit data", command=self._fit_view,
                  relief=tk.FLAT, bg="#dddddd").pack(side=tk.LEFT, padx=(6, 0))
        tk.Button(zoom_bar, text="Reset view", command=self._reset_view,
                  relief=tk.FLAT, bg="#dddddd").pack(side=tk.LEFT, padx=(4, 0))
        self.show_walls = tk.BooleanVar(value=True)
        tk.Checkbutton(zoom_bar, text="Mountains", variable=self.show_walls,
                       command=self._redraw).pack(side=tk.LEFT, padx=(8, 0))
        self.show_tape = tk.BooleanVar(value=True)
        tk.Checkbutton(zoom_bar, text="Tape", variable=self.show_tape,
                       command=self._redraw).pack(side=tk.LEFT, padx=(4, 0))
        self.show_path = tk.BooleanVar(value=True)
        tk.Checkbutton(zoom_bar, text="Path", variable=self.show_path,
                       command=self._redraw).pack(side=tk.LEFT, padx=(4, 0))
        self.lbl_zoom = tk.Label(zoom_bar, text="100%", fg="#777", font=("TkDefaultFont", 8))
        self.lbl_zoom.pack(side=tk.LEFT, padx=(8, 0))
        tk.Label(zoom_bar, text="  scroll=zoom · drag=pan", fg="#aaa",
                 font=("TkDefaultFont", 8)).pack(side=tk.LEFT, padx=(8, 0))

        self.canvas = tk.Canvas(self.root, width=self.cw, height=self.ch, bg="#f8f8f8",
                                highlightthickness=1, highlightbackground="#bbbbbb", cursor="fleur")
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 4))
        self.canvas.bind("<Configure>",       self._on_resize)
        self.canvas.bind("<MouseWheel>",       self._on_scroll)
        self.canvas.bind("<Button-4>",         self._on_scroll)
        self.canvas.bind("<Button-5>",         self._on_scroll)
        self.canvas.bind("<ButtonPress-1>",    self._on_drag_start)
        self.canvas.bind("<B1-Motion>",        self._on_drag)
        self.canvas.bind("<ButtonRelease-1>",  self._on_drag_end)

        log_frame = tk.Frame(self.root, padx=10, pady=4)
        log_frame.pack(fill=tk.X)
        tk.Label(log_frame, text="Log:", anchor="w").pack(anchor="w")
        log_inner = tk.Frame(log_frame)
        log_inner.pack(fill=tk.X)
        self._logbox = tk.Text(log_inner, height=6, state=tk.DISABLED,
                               font=("TkFixedFont", 9), bg="#f0f0f0")
        scrollbar = tk.Scrollbar(log_inner, command=self._logbox.yview)
        self._logbox.config(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self._logbox.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self._redraw()

    def _on_resize(self, event):
        self.cw, self.ch = event.width, event.height
        self._scale = (min(self.cw, self.ch) - 2 * self._margin) / MAP_M
        self._redraw()

    # ── connection ──
    def run(self) -> None:
        self._append_log(f"[WS]    connecting to {WS_URL}")
        self._ws = WSClient(WS_URL, on_line=self._on_line, on_status=self._on_ws_status)
        self.root.after(120, self._poll_queue)
        self.root.after(1000, self._tick)
        self.root.mainloop()

    def _on_line(self, line: str):
        try:
            data = json.loads(line)
        except Exception:
            return
        if isinstance(data, dict):
            self._q.put(("data", data))

    def _on_ws_status(self, ok: bool, text: str):
        self._q.put(("conn", ok, text))

    def _close(self):
        if self._ws:
            self._ws.close()
        self.root.destroy()

    def _poll_queue(self):
        dirty = False
        try:
            while True:
                item = self._q.get_nowait()
                if item[0] == "conn":
                    _, ok, text = item
                    self._connected = ok
                    self.lbl_conn.config(text=f"Link: {text}", fg="green" if ok else "orange")
                    self._append_log(f"[WS]    {text}")
                    self._update_buttons()
                    dirty = True
                elif item[0] == "data":
                    apply_message(self.state, item[1], self._append_log)
                    dirty = True
        except queue.Empty:
            pass
        if dirty:
            self._refresh_labels()
            self._redraw()
            self._update_buttons()
        self.root.after(120, self._poll_queue)

    def _tick(self):
        s = self.state
        if s.last_seen is not None:
            age = time.time() - s.last_seen
            stale = age > STALE_S
            self.lbl_age.config(text=f"Last message: {age:.1f} s ago",
                                fg="red" if stale else "gray")
        self._update_buttons()
        self.root.after(1000, self._tick)

    def _update_buttons(self):
        on = self._connected
        self.btn_start.config(state=tk.NORMAL if on else tk.DISABLED)
        self.btn_stop.config(state=tk.NORMAL if on else tk.DISABLED)

    def _send_explore(self):
        if self._ws and self._ws.send_json({"type": "mode", "mode": "explore"}):
            self._append_log("[CMD]   explore sent")

    def _send_stop(self):
        if self._ws and self._ws.send_json({"type": "mode", "mode": "stop"}):
            self._append_log("[CMD]   stop sent")

    def _reconnect(self):
        self._append_log("[WS]    manual reconnect (keeping map data)")
        if self._ws:
            self._ws.close()
        self._ws = WSClient(WS_URL, on_line=self._on_line, on_status=self._on_ws_status)

    def _save_map(self):
        s = self.state
        out = {
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "path": s.path,
            "rocks": s.rocks,
            "events": s.events,
            "tape_segments": s.bsegs,
            "mountains": sorted(list(s.walls)),
            "corners": s.corners,
            "rect_dims": s.rect_dims,
            "goal": s.goal,
        }
        name = f"venus_map_{time.strftime('%Y%m%d_%H%M%S')}.json"
        try:
            with open(name, "w", encoding="utf-8") as f:
                json.dump(out, f, indent=1)
            self._append_log(f"[MAP]   saved to {name}")
        except OSError as exc:
            self._append_log(f"[MAP]   save failed: {exc}")

    # ── labels ──
    def _refresh_labels(self):
        s = self.state
        self.lbl_robot.config(text=f"Robot: {s.status}")
        self.lbl_coords.config(
            text=f"Position:  x={s.x:.3f} m   y={s.y:.3f} m   θ={s.theta:.3f} rad")
        b = f"{s.tof_bottom_mm} mm" if s.tof_bottom_mm else "—"
        m = f"{s.tof_middle_mm} mm" if s.tof_middle_mm else "—"
        t = f"{s.tof_top_mm} mm" if s.tof_top_mm else "—"
        self.lbl_dist.config(text=f"ToF  bottom: {b}  middle: {m}  top: {t}")
        self.lbl_temp.config(text=f"Temp: {s.temp_c:.1f} °C" if s.temp_c is not None else "Temp: —")
        dots = "  ".join("●" if v else "○" for v in s.ir)
        self.lbl_ir.config(text=f"IR tape:  {dots}   (left  right)",
                           fg="#cc3333" if any(s.ir) else "#333333")
        if s.goal is not None:
            self.lbl_goal.config(text=f"Frontier goal: ({s.goal[0]:.2f}, {s.goal[1]:.2f}) m")
        else:
            self.lbl_goal.config(text="Frontier goal: —")
        self.lbl_rocks.config(text=f"Rocks: {len(s.rocks)}")
        self.lbl_last_color.config(text=f"Latest rock: {s.last_color}",
                                   fg=ROCK_COLORS.get(s.last_color.lower(), "#336699"))
        if s.rect_dims:
            w, h, area = s.rect_dims
            self.lbl_rect.config(text=f"Boundary: {w:.2f} m × {h:.2f} m  =  {area:.3f} m²",
                                 fg="#227722")
        else:
            self.lbl_rect.config(text=f"Boundary: {len(s.corners)}/4 corners "
                                      f"({sum(len(b) for b in s.bsegs)} tape pts)", fg="#555555")
        self.lbl_map.config(text=f"Mountains: {len(s.walls)}   Events: {len(s.events)}")
        self.lbl_motion.config(text=f"Motion: {'moving' if s.moving else 'stopped'}")

    def _append_log(self, text: str):
        ts = time.strftime("%H:%M:%S")
        self._logbox.config(state=tk.NORMAL)
        self._logbox.insert(tk.END, f"{ts}  {text}\n")
        self._logbox.see(tk.END)
        self._logbox.config(state=tk.DISABLED)

    # ── transforms / zoom / pan ──
    def _w2px(self, x: float, y: float):
        rx, ry = rotate_point(x, y)
        sz = self._scale * self._zoom
        return (self.cw / 2 + (rx - self._pan_x) * sz,
                self.ch / 2 - (ry - self._pan_y) * sz)

    def _zoom_step(self, factor, cx=None, cy=None):
        if cx is None: cx = self.cw / 2
        if cy is None: cy = self.ch / 2
        sz_old = self._scale * self._zoom
        self._zoom = max(0.2, min(40.0, self._zoom * factor))
        sz_new = self._scale * self._zoom
        self._pan_x += (cx - self.cw / 2) * (1 / sz_old - 1 / sz_new)
        self._pan_y -= (cy - self.ch / 2) * (1 / sz_old - 1 / sz_new)
        self.lbl_zoom.config(text=f"{int(self._zoom * 100)}%")
        self._redraw()

    def _on_scroll(self, event):
        factor = 1.2
        if getattr(event, "delta", 0):
            factor = 1.2 if event.delta > 0 else 1 / 1.2
        elif getattr(event, "num", 0) == 5:
            factor = 1 / 1.2
        self._zoom_step(factor, event.x, event.y)

    def _on_drag_start(self, event):
        self._drag_start = (event.x, event.y)
        self._drag_pan_start = (self._pan_x, self._pan_y)

    def _on_drag(self, event):
        if self._drag_start is None:
            return
        sz = self._scale * self._zoom
        self._pan_x = self._drag_pan_start[0] - (event.x - self._drag_start[0]) / sz
        self._pan_y = self._drag_pan_start[1] + (event.y - self._drag_start[1]) / sz
        self._redraw()

    def _on_drag_end(self, _event):
        self._drag_start = None
        self._drag_pan_start = None

    def _reset_view(self):
        self._zoom, self._pan_x, self._pan_y = 1.0, 0.0, 0.0
        self.lbl_zoom.config(text="100%")
        self._redraw()

    def _fit_view(self):
        s = self.state
        xs, ys = [], []

        def add(wx, wy):
            rx, ry = rotate_point(wx, wy)
            xs.append(rx); ys.append(ry)

        add(0.0, 0.0)
        for (px, py) in s.path:        add(px, py)
        for r in s.rocks:              add(r["x"], r["y"])
        for e in s.events:             add(e["x"], e["y"])
        for cn in s.corners:           add(cn["x"], cn["y"])
        for seg in s.bsegs:
            for (bx, by) in seg:       add(bx, by)
        for (gx, gy) in s.walls:       add(gx * WALL_GRID_M, gy * WALL_GRID_M)
        if s.goal:                     add(s.goal[0], s.goal[1])
        if len(xs) < 2:
            return
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        self._pan_x = (min_x + max_x) / 2
        self._pan_y = (min_y + max_y) / 2
        span = max(max_x - min_x, max_y - min_y, 0.4) * 1.3
        self._zoom = max(0.2, min(40.0,
            (min(self.cw, self.ch) - 2 * self._margin) / (span * self._scale)))
        self.lbl_zoom.config(text=f"{int(self._zoom * 100)}%")
        self._redraw()

    # ── drawing ──
    @staticmethod
    def _nice_step(target: float) -> float:
        if target <= 0:
            return 0.1
        mag = 10 ** math.floor(math.log10(target))
        for f in (0.1, 0.2, 0.25, 0.5, 1.0, 2.0, 2.5, 5.0, 10.0):
            step = mag * f
            if step >= target * 0.9:
                return step
        return mag * 10

    def _redraw(self):
        c = self.canvas
        c.delete("all")
        s = self.state
        sz = self._scale * self._zoom

        # adaptive grid
        minor = self._nice_step((self.cw / sz) / 8)
        major = minor * 5
        xl = self._pan_x - self.cw / (2 * sz) - minor
        xr = self._pan_x + self.cw / (2 * sz) + minor
        yb = self._pan_y - self.ch / (2 * sz) - minor
        yt = self._pan_y + self.ch / (2 * sz) + minor

        def snap(v, step):
            return math.floor(v / step) * step

        gx = snap(xl, minor)
        while gx <= xr:
            is_axis = abs(gx) < minor * 0.01
            is_major = not is_axis and abs(round(gx / major) * major - gx) < minor * 0.01
            px = self.cw / 2 + (gx - self._pan_x) * sz
            col = "#888888" if is_axis else ("#cccccc" if is_major else "#ececec")
            c.create_line(px, 0, px, self.ch, fill=col, width=2 if is_axis else 1)
            if is_major or is_axis:
                c.create_text(px + 2, self.ch / 2 + 2, text=f"{gx:.3g}m",
                              fill="#aaaaaa", font=("TkFixedFont", 7), anchor="nw")
            gx = round(gx + minor, 10)
        gy = snap(yb, minor)
        while gy <= yt:
            is_axis = abs(gy) < minor * 0.01
            is_major = not is_axis and abs(round(gy / major) * major - gy) < minor * 0.01
            py = self.ch / 2 - (gy - self._pan_y) * sz
            col = "#888888" if is_axis else ("#cccccc" if is_major else "#ececec")
            c.create_line(0, py, self.cw, py, fill=col, width=2 if is_axis else 1)
            if is_major or is_axis:
                c.create_text(self.cw / 2 - 2, py - 2, text=f"{gy:.3g}m",
                              fill="#aaaaaa", font=("TkFixedFont", 7), anchor="se")
            gy = round(gy + minor, 10)

        # mountains (point cloud)
        if self.show_walls.get():
            for (gxc, gyc) in s.walls:
                px, py = self._w2px(gxc * WALL_GRID_M, gyc * WALL_GRID_M)
                c.create_rectangle(px - 3, py - 3, px + 3, py + 3, fill="#8a6d3b", outline="")

        # path
        if self.show_path.get() and len(s.path) >= 2:
            pts = []
            for wx, wy in s.path:
                px, py = self._w2px(wx, wy)
                pts += [px, py]
            c.create_line(*pts, fill="#9ec8e8", width=2)

        # tape (boundary) polylines
        if self.show_tape.get():
            for seg in s.bsegs:
                if len(seg) >= 2:
                    pts = []
                    for wx, wy in seg:
                        px, py = self._w2px(wx, wy)
                        pts += [px, py]
                    c.create_line(*pts, fill="#ff8800", width=3,
                                  capstyle=tk.ROUND, joinstyle=tk.ROUND)
                elif seg:
                    px, py = self._w2px(seg[0][0], seg[0][1])
                    c.create_oval(px - 3, py - 3, px + 3, py + 3, fill="#ff8800", outline="")

        # derived arena rectangle: corner skeleton + numbered corner dots
        if len(s.corners) >= 2:
            pts = []
            for cn in s.corners:
                px, py = self._w2px(cn["x"], cn["y"])
                pts += [px, py]
            if len(s.corners) >= 4:                       # close the loop
                px, py = self._w2px(s.corners[0]["x"], s.corners[0]["y"])
                pts += [px, py]
            c.create_line(*pts, fill="#111111", width=3, capstyle=tk.ROUND, joinstyle=tk.ROUND)
            for cn in s.corners:
                px, py = self._w2px(cn["x"], cn["y"])
                c.create_oval(px - 5, py - 5, px + 5, py + 5,
                              fill="#ffcc00", outline="#111111", width=2)
                c.create_text(px, py - 12, text=f"C{cn['n']}",
                              fill="#996600", font=("TkFixedFont", 8, "bold"))

        # events (boundary / obstacle / cliff) as X marks
        for ev in s.events:
            ex, ey = self._w2px(ev["x"], ev["y"])
            col = EVENT_COLORS.get(ev["event"], "#888888")
            rr = 6
            c.create_line(ex - rr, ey - rr, ex + rr, ey + rr, fill=col, width=2)
            c.create_line(ex + rr, ey - rr, ex - rr, ey + rr, fill=col, width=2)

        # rocks
        for rock in s.rocks:
            rx, ry = self._w2px(rock["x"], rock["y"])
            color = rock["color"]
            fill = ROCK_COLORS.get(color, "#999999")
            outline = "#ffffff" if color == "black" else "#222222"   # contrast on dark fills
            hs = max(5, (rock["size_mm"] / 1000.0) / 2.0 * sz)
            partner = rock.get("robot", "self") != "self"
            opts = dict(fill=fill, outline=outline, width=2)
            if partner:
                opts["dash"] = (3, 2)                                # partner = dashed
            c.create_rectangle(rx - hs, ry - hs, rx + hs, ry + hs, **opts)
            label = color[:1].upper() + ("·P" if partner else "")
            # light fills (white/yellow) need a dark label to be readable
            lcol = "#111111" if color in ("white", "yellow") else "#ffffff"
            c.create_text(rx, ry, text=label, fill=lcol, font=("TkFixedFont", 7, "bold"))
            if not rock.get("size_known", True):
                c.create_text(rx, ry - hs - 6, text="?", fill="#888888",
                              font=("TkFixedFont", 8, "bold"))
            tc = rock.get("temp_c")
            if tc is not None:
                c.create_text(rx, ry + hs + 7, text=f"{tc:.0f}°", fill="#333333",
                              font=("TkFixedFont", 7))

        # frontier goal marker
        if s.goal is not None:
            gxp, gyp = self._w2px(s.goal[0], s.goal[1])
            rbx, rby = self._w2px(s.x, s.y)
            c.create_line(rbx, rby, gxp, gyp, fill="#e58b4d", width=1, dash=(4, 3))
            c.create_oval(gxp - 9, gyp - 9, gxp + 9, gyp + 9, outline="#e58b4d", width=2)
            c.create_line(gxp - 13, gyp, gxp - 5, gyp, fill="#e58b4d", width=2)
            c.create_line(gxp + 5, gyp, gxp + 13, gyp, fill="#e58b4d", width=2)
            c.create_line(gxp, gyp - 13, gxp, gyp - 5, fill="#e58b4d", width=2)
            c.create_line(gxp, gyp + 5, gxp, gyp + 13, fill="#e58b4d", width=2)

        # robot + heading
        rbx, rby = self._w2px(s.x, s.y)
        r = 10
        c.create_oval(rbx - r, rby - r, rbx + r, rby + r,
                      fill="#cc3333" if s.moving else "#336699", outline="#222222", width=2)
        rot = rotate_angle(s.theta)
        c.create_line(rbx, rby, rbx + 22 * math.cos(rot), rby - 22 * math.sin(rot),
                      fill="#222222", width=2, arrow=tk.LAST)

        # status dot
        alive = s.last_seen is not None and (time.time() - s.last_seen) < STALE_S
        c.create_oval(self.cw - 22, 8, self.cw - 10, 20,
                      fill="#33aa33" if alive else "#cc3333", outline="")
        bg, fg, txt = ("#e0f5e0", "#227722", "LINK ●") if self._connected else ("#f5e0e0", "#aa3333", "LINK ○")
        c.create_rectangle(4, 4, 60, 22, fill=bg, outline=fg, width=1)
        c.create_text(32, 13, text=txt, fill=fg, font=("TkDefaultFont", 7, "bold"))

        self._draw_legend()

    def _draw_legend(self):
        lx, ly = 8, self.ch - 10
        items = [
            ("■ rock sample", "#444444"),
            ("● corner",      "#996600"),
            ("— arena rect",  "#111111"),
            ("▪ mountain",    "#8a6d3b"),
            ("— tape boundary", "#ff8800"),
            ("× obstacle",    "#884400"),
            ("⊕ frontier goal", "#e58b4d"),
            ("— path",        "#9ec8e8"),
        ]
        for text, color in reversed(items):
            self.canvas.create_text(lx, ly, text=text, anchor="sw",
                                     fill=color, font=("TkFixedFont", 8))
            ly -= 14


if __name__ == "__main__":
    print("Venus desktop monitor (our rover)")
    print(f"  Connecting to {WS_URL}")
    print("  (start ws_bridge.py --exec \"./main\" on the Pynq first)")
    App().run()
