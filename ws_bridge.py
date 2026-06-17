#!/usr/bin/env python3
"""
Rover UI WebSocket bridge -- runs ON THE PYNQ, serves telemetry over the
network so the browser UI connects over WiFi (no USB serial needed).

Pure standard library: no pip installs. Minimal WebSocket server
(handshake + masked/unmasked frames) with two modes:

  SIMULATE (default):
      python3 ws_bridge.py
    Streams a fake wandering rover. Good for testing the link.

  REAL ROBOT:
      python3 ws_bridge.py --exec "sudo ./robot_main"
    Spawns the firmware, forwards its stdout (JSON telemetry) to every
    connected browser, and forwards browser commands (target/mode) back
    to the firmware's stdin. Firmware diagnostics on its stderr stay in
    your terminal.

Find the Pynq IP with `hostname -I` (or use the SSH address). Then in the
browser UI connect to  ws://<pynq-ip>:8765
"""

import argparse
import base64
import hashlib
import json
import math
import socket
import struct
import subprocess
import sys
import threading
import time

WS_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
HOST = "0.0.0.0"
PORT = 8765

clients = set()
clients_lock = threading.Lock()


# --------------------------- WebSocket framing --------------------------

def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def ws_handshake(conn):
    req = conn.recv(4096).decode("utf-8", "replace")
    key = None
    for line in req.split("\r\n"):
        if line.lower().startswith("sec-websocket-key:"):
            key = line.split(":", 1)[1].strip()
    if not key:
        return False
    accept = base64.b64encode(
        hashlib.sha1((key + WS_MAGIC).encode()).digest()).decode()
    conn.send((
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " + accept + "\r\n\r\n").encode())
    return True


def ws_send(conn, text):
    payload = text.encode("utf-8")
    n = len(payload)
    header = bytearray([0x81])           # FIN + text
    if n < 126:
        header.append(n)
    elif n < 65536:
        header.append(126)
        header += struct.pack(">H", n)
    else:
        header.append(127)
        header += struct.pack(">Q", n)
    conn.sendall(bytes(header) + payload)


def ws_recv(conn):
    """Read one client->server frame. Returns text, '' (control), or None (close)."""
    hdr = recv_exact(conn, 2)
    if not hdr:
        return None
    opcode = hdr[0] & 0x0F
    masked = hdr[1] & 0x80
    ln = hdr[1] & 0x7F
    if ln == 126:
        ext = recv_exact(conn, 2)
        if ext is None:
            return None
        ln = struct.unpack(">H", ext)[0]
    elif ln == 127:
        ext = recv_exact(conn, 8)
        if ext is None:
            return None
        ln = struct.unpack(">Q", ext)[0]
    mask = recv_exact(conn, 4) if masked else b"\x00\x00\x00\x00"
    if mask is None:
        return None
    payload = recv_exact(conn, ln) if ln else b""
    if payload is None:
        return None
    if masked:
        payload = bytes(payload[i] ^ mask[i % 4] for i in range(ln))
    if opcode == 0x8:          # close
        return None
    if opcode in (0x9, 0xA):   # ping / pong -> ignore
        return ""
    return payload.decode("utf-8", "replace")


def broadcast(text):
    with clients_lock:
        dead = []
        for c in list(clients):
            try:
                ws_send(c, text)
            except Exception:
                dead.append(c)
        for c in dead:
            clients.discard(c)


# ------------------------------- modes ----------------------------------

def make_exec_handler(cmd):
    """Spawn the firmware; pipe its stdout -> browsers, browser msgs -> its stdin."""
    proc = subprocess.Popen(
        cmd, shell=True,
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        bufsize=1, universal_newlines=True)

    def reader():
        for line in proc.stdout:
            line = line.strip()
            if line:
                broadcast(line)
        print("[ws] firmware process ended", file=sys.stderr)
    threading.Thread(target=reader, daemon=True).start()

    def on_message(msg):
        try:
            proc.stdin.write(msg if msg.endswith("\n") else msg + "\n")
            proc.stdin.flush()
        except Exception:
            pass
    return on_message


OBJECTS = [
    (0.60, 0.40, "scannable", "red"), (-0.50, 0.85, "scannable", "green"),
    (0.95, -0.60, "scannable", "blue"), (-0.80, -0.70, "scannable", "yellow"),
    (1.20, 0.20, "mountain", None), (0.00, 1.30, "mountain", None),
    (-1.30, 0.10, "mountain", None), (0.30, -1.20, "black_tape", None),
]


def start_sim():
    """Background thread broadcasting a fake wandering rover to all clients."""
    def sim():
        x = y = th = 0.0
        phase = 0.0
        dt = 0.05
        reported = set()
        last_pose = last_status = 0.0
        while True:
            if math.hypot(x, y) > 1.4:
                desired = math.atan2(-y, -x)
            else:
                phase += 0.6 * dt
                desired = th + 0.5 * math.sin(phase)
            err = (desired - th + math.pi) % (2 * math.pi) - math.pi
            mx = 1.2 * dt
            err = max(-mx, min(mx, err))
            th = (th + err + math.pi) % (2 * math.pi) - math.pi
            x += 0.18 * dt * math.cos(th)
            y += 0.18 * dt * math.sin(th)
            for i, (ox, oy, kind, color) in enumerate(OBJECTS):
                if i in reported:
                    continue
                if math.hypot(ox - x, oy - y) <= 0.22:
                    reported.add(i)
                    if kind == "scannable":
                        broadcast(json.dumps({"type": "observation", "robot": "r1",
                                              "kind": "scannable", "color": color,
                                              "x": ox, "y": oy, "confidence": 0.92}))
                    else:
                        broadcast(json.dumps({"type": "observation", "robot": "r1",
                                              "kind": kind, "x": ox, "y": oy,
                                              "confidence": 0.80}))
            now = time.time()
            if now - last_pose >= 0.3:
                broadcast(json.dumps({"type": "pose", "robot": "r1",
                                      "x": round(x, 4), "y": round(y, 4),
                                      "theta": round(th, 4)}))
                last_pose = now
            if now - last_status >= 2.0:
                broadcast(json.dumps({"type": "status", "robot": "r1",
                                      "mode": "explore", "detail": "alive"}))
                last_status = now
            time.sleep(dt)
    threading.Thread(target=sim, daemon=True).start()


# ------------------------------- server ---------------------------------

def handle_client(conn, addr, on_message):
    try:
        if not ws_handshake(conn):
            conn.close()
            return
        with clients_lock:
            clients.add(conn)
        print("[ws] client connected:", addr, file=sys.stderr)
        while True:
            msg = ws_recv(conn)
            if msg is None:
                break
            if msg and on_message:
                on_message(msg)
    except Exception:
        pass
    finally:
        with clients_lock:
            clients.discard(conn)
        conn.close()
        print("[ws] client gone:", addr, file=sys.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exec", dest="exec_cmd", default=None,
                    help='Run the firmware and bridge it, e.g. --exec "sudo ./robot_main"')
    ap.add_argument("--port", type=int, default=PORT)
    args = ap.parse_args()

    if args.exec_cmd:
        on_message = make_exec_handler(args.exec_cmd)
        print("[ws] REAL ROBOT mode: %s" % args.exec_cmd, file=sys.stderr)
    else:
        start_sim()
        on_message = None
        print("[ws] SIMULATE mode (use --exec \"sudo ./robot_main\" for the real robot)", file=sys.stderr)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, args.port))
    srv.listen(5)
    print("[ws] listening on ws://%s:%d  (connect to ws://<pynq-ip>:%d)"
          % (HOST, args.port, args.port), file=sys.stderr)
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=handle_client,
                         args=(conn, addr, on_message), daemon=True).start()


if __name__ == "__main__":
    main()
