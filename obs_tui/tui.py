#!/usr/bin/env python3
import argparse, asyncio, time, shutil, re
from pathlib import Path
from collections import deque
import yaml, zmq, zmq.asyncio
from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, DataTable, Static

# ---------------- Base class ----------------
class HealthMonitor:
    def __init__(self, name: str):
        self.name = name
        self.status = "UNKNOWN"  # OK | WARN | CRIT | UNKNOWN
        self.msg = ""
        self.ts = 0.0

    async def run(self):
        raise NotImplementedError

# ---------------- ZMQ Monitor ----------------
class ZMQMonitor(HealthMonitor):
    """Monitors a ZMQ SUB address with idle detection and simple reconnects.
    Exposes: rate (Hz), bytes/s, idle seconds, reconnects, preview.
    """
    def __init__(self, name, address, topic=b"", *,
                 rate_window_sec: float = 5.0,
                 warn_idle_sec: float = 2.0,
                 crit_idle_sec: float = 5.0,
                 reset_after_sec: float = 0.0):
        super().__init__(name)
        self.address, self.topic = address, topic
        self.window = float(rate_window_sec)
        self.warn_idle = float(warn_idle_sec)
        self.crit_idle = float(crit_idle_sec)
        self.reset_after = float(reset_after_sec)

        self.ctx = zmq.asyncio.Context.instance()
        self.samples = deque()              # (ts, nbytes)
        self.last_msg_ts = 0.0
        self.start_ts = time.time()
        self.ever_received = False
        self.reconnects = 0

        self.rate = 0.0
        self.bps = 0.0
        self.preview = ""

    def _make_socket(self):
        sock = self.ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, self.topic)
        sock.connect(self.address)
        return sock

    async def run(self):
        sock = self._make_socket()
        self.start_ts = time.time()
        while True:
            try:
                # Wait up to 1000 ms for a message
                events = await sock.poll(1000)
                now = time.time()
                if events:
                    msg = await sock.recv_multipart()
                    nbytes = sum(len(m) for m in msg)
                    self.samples.append((now, nbytes))
                    # Drop old samples outside the window
                    cutoff = now - self.window
                    while self.samples and self.samples[0][0] < cutoff:
                        self.samples.popleft()
                    # Compute smoothed stats
                    self.rate = len(self.samples) / max(self.window, 1e-6)
                    self.bps = sum(sz for _, sz in self.samples) / max(self.window, 1e-6)
                    self.last_msg_ts = now
                    self.ever_received = True
                    # Short preview from first frame
                    first = msg[0] if msg else b""
                    try:
                        self.preview = first[:60].decode("utf-8", errors="ignore")
                    except Exception:
                        self.preview = first.hex()[:60]

                # Compute idleness and set status/message
                idle = (now - self.last_msg_ts) if self.ever_received else (now - self.start_ts)
                if self.ever_received:
                    if idle >= self.crit_idle:
                        self.status = "CRIT"
                    elif idle >= self.warn_idle:
                        self.status = "WARN"
                    else:
                        self.status = "OK"
                else:
                    if idle >= self.crit_idle:
                        self.status = "CRIT"
                    elif idle >= self.warn_idle:
                        self.status = "WARN"
                    else:
                        self.status = "UNKNOWN"

                # Keep a concise message; detailed columns go in the table
                if self.status == "OK":
                    self.msg = f"flow {self.rate:.1f} Hz {self.bps:.1f} B/s"
                else:
                    self.msg = f"idle {idle:.1f}s"

                # Optional reconnect after long idle
                if self.reset_after > 0 and idle >= self.reset_after:
                    try:
                        sock.close(linger=0)
                    finally:
                        self.reconnects += 1
                        sock = self._make_socket()
                        await asyncio.sleep(0.05)

                self.ts = now

            except asyncio.CancelledError:
                sock.close(linger=0)
                raise
            except Exception as e:
                # Treat unexpected errors as critical, try to recreate socket
                self.status = "CRIT"
                self.msg = f"recv err: {e} (reconnecting)"
                self.ts = time.time()
                try:
                    sock.close(linger=0)
                finally:
                    self.reconnects += 1
                    sock = self._make_socket()
                    await asyncio.sleep(0.1)

# ---------------- Time Monitor (Chrony) ----------------
class TimeMonitor(HealthMonitor):
    def __init__(self):
        super().__init__("time")
        self.source = ""
        self.offset_ms = None
        self.up_int_s = None

    async def run(self):
        sys_time_re = re.compile(r"^System time\s*:\s*([-+]?\d*\.?\d+)\s+seconds\s+(fast|slow)", re.I)
        last_off_re = re.compile(r"^Last offset\s*:\s*([-+]?\d*\.?\d+)\s+seconds", re.I)
        update_int_re = re.compile(r"^Update interval\s*:\s*(\d*\.?\d*)\s+seconds", re.I)

        # selected source line has '*' in the first columns
        src_re = re.compile(r"^[\^~=#]?[\*\+\-]\s*([^\s]+)")
        while True:
            try:
                # Offset via 'chronyc tracking'
                proc = await asyncio.create_subprocess_exec(
                    "chronyc", "tracking",
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                )
                out, err = await proc.communicate()
                if proc.returncode != 0:
                    raise RuntimeError(err.decode().strip() or f"chronyc rc={proc.returncode}")
                text = out.decode()
                offset_s = None
                interval_s = None
                for line in text.splitlines():
                    m = sys_time_re.search(line)
                    if m:
                        val = float(m.group(1))
                        sign = -1.0 if (m.group(2).lower() == "slow") else 1.0
                        offset_s = sign * val
                        # break
                    m2 = last_off_re.search(line)
                    if m2:
                        offset_s = float(m2.group(1))
                    up_int = update_int_re.search(line)
                    if up_int:
                        interval_s = float(up_int.group(1))
                        break
                        
                if offset_s is None:
                    raise RuntimeError("could not parse chronyc tracking output")
                if interval_s is None:
                    raise RuntimeError("could not parse chronyc tracking output")
                self.offset_ms = offset_s * 1000.0
                self.up_int_s = interval_s

                # Selected source via 'chronyc sources -n'
                proc2 = await asyncio.create_subprocess_exec(
                    "chronyc", "sources", "-n",
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                )
                out2, _ = await proc2.communicate()
                selected = ""
                for line in out2.decode().splitlines():
                    if line[:3].find('*') != -1:  # current selected source
                        m = src_re.match(line.strip())
                        if m:
                            selected = m.group(1)
                            break
                self.source = selected or self.source or "(unknown)"

                if abs(self.offset_ms) < 1 and interval_s < 64:
                    self.status = "OK"
                elif abs(self.offset_ms) < 10 and interval_s < 1025:
                    self.status = "WARN"
                else:
                    self.status = "CRIT"
                self.msg = ""
            except Exception as e:
                self.status, self.msg = "CRIT", f"chrony err {e}"
            self.ts = time.time()
            await asyncio.sleep(5)

# ---------------- Disk Monitor ----------------
class DiskMonitor(HealthMonitor):
    def __init__(self, path: str, warning: float = 100, critical: float = 20):
        super().__init__(f"{path}")
        self.path = path
        self.wanning = warning
        self.critical = critical

    async def run(self):
        while True:
            try:
                usage = await asyncio.to_thread(shutil.disk_usage, self.path)
                free_gb = usage.free / (1024**3)
                if free_gb > self.wanning:
                    self.status = "OK"
                elif free_gb > self.critical:
                    self.status = "WARN"
                else:
                    self.status = "CRIT"
                self.msg = f"{free_gb:.1f} GB free"
            except Exception as e:
                self.status, self.msg = "CRIT", str(e)
            self.ts = time.time()
            await asyncio.sleep(10)

# ---------------- App ----------------
class HealthApp(App):
    BINDINGS = [("q", "quit", "Quit")]

    def __init__(self, config: Path):
        super().__init__()
        self.config = yaml.safe_load(config.read_text())
        self.monitors: list[HealthMonitor] = []

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)

        # ZMQ table with dedicated columns
        yield Static("[b]ZMQ[/b]", expand=False)
        self.table_zmq = DataTable(zebra_stripes=True)
        self.table_zmq.add_columns("Name", "Address", "Status", "Idle(s)", "Rate(Hz)", "B/s", "Reconn", "Preview")
        yield self.table_zmq
        yield Static("")  # spacing

        # Time table with source column
        yield Static("[b]Time[/b]", expand=False)
        self.table_time = DataTable(zebra_stripes=True)
        self.table_time.add_columns("Source", "Status", "Offset(ms)", "Update Int(s)", "Msg")
        yield self.table_time
        yield Static("")

        # Disk table unchanged (simple)
        yield Static("[b]Disk space[/b]", expand=False)
        self.table_disk = DataTable(zebra_stripes=True)
        self.table_disk.add_columns("Location", "Status", "Msg")
        yield self.table_disk

        yield Footer()

    async def on_mount(self):
        # Build monitors from config
        rw = float(self.config.get("rate_window_sec", 5))
        warn_idle = float(self.config.get("warn_idle_sec", 2))
        crit_idle = float(self.config.get("crit_idle_sec", 5))
        reset_after = float(self.config.get("reset_after_sec", 0))

        for c in self.config.get("channels", []):
            topic = c.get("topic", "").encode()
            m = ZMQMonitor(c["name"], c["address"], topic,
                           rate_window_sec=rw,
                           warn_idle_sec=warn_idle,
                           crit_idle_sec=crit_idle,
                           reset_after_sec=reset_after)
            self.monitors.append(m)
            self.run_worker(m.run(), exclusive=False)
        if self.config.get("check_time"):
            m = TimeMonitor()
            self.monitors.append(m)
            self.run_worker(m.run(), exclusive=False)
        for d in self.config.get("check_disks", []):
            m = DiskMonitor(d["path"], d.get("warning", 100.), d.get("critical", 20.))
            self.monitors.append(m)
            self.run_worker(m.run(), exclusive=False)

        self.set_interval(1, self.refresh_tables)

    def _color_status(self, status: str) -> str:
        if status == "OK":
            return "[green]OK[/]"
        if status == "WARN":
            return "[yellow]WARN[/]"
        if status == "CRIT":
            return "[red]CRIT[/]"
        return f"[dim]{status}[/]"

    def refresh_tables(self):
        now = time.time()
        self.table_zmq.clear()
        self.table_time.clear()
        self.table_disk.clear()
        for m in self.monitors:
            status_colored = self._color_status(m.status)
            if isinstance(m, ZMQMonitor):
                idle = (now - m.last_msg_ts) if m.ever_received else (now - m.start_ts)
                self.table_zmq.add_row(
                    m.name, m.address, status_colored,
                    f"{idle:.1f}", f"{m.rate:.2f}", f"{m.bps:.0f}", f"{m.reconnects}", m.preview
                )
            elif isinstance(m, TimeMonitor):
                off = "" if m.offset_ms is None else f"{m.offset_ms:.1f}"
                up_int = "" if m.up_int_s is None else f"{m.up_int_s:.1f}"
                self.table_time.add_row(m.source or "(unknown)", status_colored,  off, up_int, m.msg)
            elif isinstance(m, DiskMonitor):
                self.table_disk.add_row(m.name, status_colored, m.msg)

# -------------- entrypoint --------------
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", type=Path, required=True)
    args = p.parse_args()
    HealthApp(args.config).run()

if __name__ == "__main__":
    main()

