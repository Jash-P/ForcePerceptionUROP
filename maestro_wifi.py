# maestro_wifi.py
import socket
import threading
import time
from typing import Callable, Dict, Optional

class MaestroWiFiClient:
    """
    Minimal client for the ESP32C3 Maestro Wi-Fi bridge.
    - Maintains a TCP connection
    - Background receiver parses lines
    - Streams POS telemetry into .latest (dict)
    - Synchronous request/response (id=) helpers
    """
    def __init__(self, host: str, port: int = 9000, timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        # request id & waiter map
        self._next_id = 1
        self._waiters: Dict[str, Dict] = {}

        # latest telemetry: {ch: {"raw": int, "us": float, "t": int}}
        self.latest: Dict[int, Dict] = {}
        # optional callback for telemetry: cb(ch, raw, us, t) -> None
        self.on_pos: Optional[Callable[[int, int, float, int], None]] = None

        # metadata after HELLO
        self.hello: Dict[str, str] = {}

    # ---- connection management ----
    def connect(self):
        if self.sock:
            raise RuntimeError("already connected")
        self.sock = socket.create_connection((self.host, self.port), self.timeout)
        self.sock.settimeout(0.5)
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # wait briefly for HELLO
        t0 = time.time()
        while not self.hello and (time.time() - t0) < 2.0:
            time.sleep(0.05)

    def close(self):
        self._stop.set()
        if self.sock:
            try: self.sock.shutdown(socket.SHUT_RDWR)
            except Exception: pass
            try: self.sock.close()
            except Exception: pass
        self.sock = None

    def __enter__(self): self.connect(); return self
    def __exit__(self, exc_type, exc, tb): self.close()

    # ---- low-level send ----
    def _send_line(self, line: str):
        if not self.sock: raise RuntimeError("not connected")
        data = (line.strip() + "\n").encode("ascii")
        with self._lock:
            self.sock.sendall(data)

    def _next_req_id(self) -> str:
        with self._lock:
            rid = str(self._next_id)
            self._next_id += 1
            return rid

    def _wait_for_id(self, rid: str, timeout: float):
        ev = threading.Event()
        slot = {"event": ev, "line": None}
        self._waiters[rid] = slot
        ok = ev.wait(timeout)
        self._waiters.pop(rid, None)
        if not ok:
            raise TimeoutError(f"timeout waiting for id={rid}")
        return slot["line"]

    # ---- receiver ----
    def _rx_loop(self):
        buf = b""
        try:
            while not self._stop.is_set():
                try:
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        raise ConnectionError("socket closed")
                    buf += chunk
                except socket.timeout:
                    chunk = b""
                # process complete lines
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        s = line.decode("ascii", errors="ignore").strip()
                        if s:
                            self._handle_line(s)
                    except Exception:
                        pass
        except Exception:
            # connection failure – just exit thread
            pass

    # ---- line parser ----
    def _handle_line(self, s: str):
        # print("[DBG]", s)  # uncomment for debugging
        if s.startswith("POS "):
            # POS ch=<n> raw=<q> us=<v> t=<ms>   OR "POS ch=<n> timeout ..."
            toks = dict(part.split("=",1) for part in s.split()[1:] if "=" in part)
            if "ch" in toks:
                ch = int(toks["ch"])
                if "raw" in toks and "us" in toks and "t" in toks:
                    raw = int(toks["raw"]); us = float(toks["us"]); t = int(float(toks["t"]))
                    self.latest[ch] = {"raw": raw, "us": us, "t": t}
                    if self.on_pos: 
                        try: self.on_pos(ch, raw, us, t)
                        except Exception: pass
                # ignore timeout lines for latest
            return

        if s.startswith("HELLO "):
            toks = dict(part.split("=",1) for part in s.split()[1:] if "=" in part)
            self.hello = toks
            return

        # request/response mapping via "id="
        # look for token "id=xxx"
        rid = None
        parts = s.split()
        for p in parts:
            if p.startswith("id="):
                rid = p[3:]
                break
        if rid and rid in self._waiters:
            slot = self._waiters[rid]
            slot["line"] = s
            slot["event"].set()
            return
        # otherwise just ignore or log

    # ---- public API ----
    def set_target(self, ch: int, us: int, timeout: float = 1.0) -> str:
        rid = self._next_req_id()
        self._send_line(f"SET {ch} {us} id={rid}")
        return self._wait_for_id(rid, timeout)

    def get_position(self, ch: int, timeout: float = 1.0):
        rid = self._next_req_id()
        self._send_line(f"GET {ch} id={rid}")
        line = self._wait_for_id(rid, timeout)
        # line example: "GET id=7 ch=0 raw=6000 us=1500.00"
        toks = dict(part.split("=",1) for part in line.split() if "=" in part)
        return int(toks.get("ch","-1")), int(toks.get("raw","-1")), float(toks.get("us","nan"))

    def get_moving(self, timeout: float = 1.0) -> int:
        rid = self._next_req_id()
        self._send_line(f"MOVING id={rid}")
        line = self._wait_for_id(rid, timeout)
        toks = dict(part.split("=",1) for part in line.split() if "=" in part)
        return int(toks.get("state","-1"))

    def get_errors(self, timeout: float = 1.0) -> str:
        rid = self._next_req_id()
        self._send_line(f"ERRORS id={rid}")
        line = self._wait_for_id(rid, timeout)
        # returns line; caller can parse bits=0x....
        return line

    def set_autopoll(self, enable: bool, timeout: float = 1.0) -> str:
        rid = self._next_req_id()
        self._send_line(f"AUTOPOLL {1 if enable else 0} id={rid}")
        return self._wait_for_id(rid, timeout)

    def set_poll(self, channels: int, interval_ms: Optional[int] = None, timeout: float = 1.0) -> str:
        rid = self._next_req_id()
        if interval_ms is None:
            self._send_line(f"POLL {channels} id={rid}")
        else:
            self._send_line(f"POLL {channels} {interval_ms} id={rid}")
        return self._wait_for_id(rid, timeout)

    def ping(self, timeout: float = 1.0) -> str:
        rid = self._next_req_id()
        self._send_line(f"PING id={rid}")
        return self._wait_for_id(rid, timeout)

    # --- new convenience ops that hit the new firmware commands ---
    def set_speed(self, ch: int, value: int, timeout: float = 1.0) -> str:
        rid = self._next_req_id()
        self._send_line(f"SPEED {ch} {value} id={rid}")
        return self._wait_for_id(rid, timeout)

    def set_accel(self, ch: int, value: int, timeout: float = 1.0) -> str:
        rid = self._next_req_id()
        self._send_line(f"ACCEL {ch} {value} id={rid}")
        return self._wait_for_id(rid, timeout)

    def run_script(self, sub: int, param: int | None = None, timeout: float = 1.0) -> str:
        rid = self._next_req_id()
        if param is None:
            self._send_line(f"RUN {sub} id={rid}")
        else:
            self._send_line(f"RUN {sub} {param} id={rid}")
        return self._wait_for_id(rid, timeout)

    def stop_script(self, timeout: float = 1.0) -> str:
        rid = self._next_req_id()
        self._send_line(f"STOPSCRIPT id={rid}")
        return self._wait_for_id(rid, timeout)

