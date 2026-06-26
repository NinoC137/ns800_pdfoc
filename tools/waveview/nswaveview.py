#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NS800 serial waveform viewer.

MCU output format:
    1,2,3,4\\r
    1,2,3,4\\r\\n
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import random
import socket
import sys
import threading
import time
import webbrowser
from collections import deque
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Iterable
from urllib.parse import parse_qs, urlparse


DEFAULT_BAUD = 115200
DEFAULT_CHANNELS = 4
DEFAULT_WINDOW = 1000
DEFAULT_HOST = "127.0.0.1"
DEFAULT_HTTP_PORT = 8765
SSE_KEEPALIVE_S = 1.0


HTML_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>NS800 WaveView</title>
<style>
:root {
  color-scheme: dark;
  --bg: #101114;
  --panel: #181a20;
  --text: #f2f4f8;
  --muted: #9aa4b2;
  --grid: #2b3038;
  --accent: #43d9ad;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  background: var(--bg);
  color: var(--text);
  font: 14px/1.4 -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}
header {
  height: 54px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 16px;
  border-bottom: 1px solid #242832;
  background: #14161b;
}
h1 { margin: 0; font-size: 17px; font-weight: 650; }
main {
  height: calc(100vh - 54px);
  display: grid;
  grid-template-rows: auto 1fr auto;
}
.toolbar {
  display: flex;
  flex-wrap: wrap;
  gap: 10px 14px;
  align-items: center;
  padding: 10px 16px;
  background: var(--panel);
  border-bottom: 1px solid #242832;
}
.field { display: flex; align-items: center; gap: 6px; color: var(--muted); }
input {
  width: 92px;
  padding: 5px 7px;
  color: var(--text);
  background: #0f1116;
  border: 1px solid #323846;
  border-radius: 4px;
}
button {
  padding: 6px 10px;
  color: var(--text);
  background: #242a35;
  border: 1px solid #384253;
  border-radius: 4px;
  cursor: pointer;
}
button:hover { border-color: var(--accent); }
.status {
  display: flex;
  flex-wrap: wrap;
  gap: 16px;
  color: var(--muted);
}
.status strong { color: var(--text); font-weight: 600; }
.plot-wrap {
  min-height: 0;
  position: relative;
  padding: 10px 12px 0;
}
canvas {
  width: 100%;
  height: 100%;
  display: block;
  background: #0c0d11;
  border: 1px solid #252a35;
}
.legend {
  min-height: 42px;
  display: flex;
  flex-wrap: wrap;
  gap: 10px 18px;
  align-items: center;
  padding: 9px 16px 12px;
  color: var(--muted);
}
.swatch {
  display: inline-block;
  width: 12px;
  height: 12px;
  margin-right: 6px;
  vertical-align: -1px;
  border-radius: 2px;
}
</style>
</head>
<body>
<header>
  <h1>NS800 WaveView</h1>
  <div id="source"></div>
</header>
<main>
  <section class="toolbar">
    <button id="pauseBtn">暂停</button>
    <button id="clearBtn">清空</button>
    <button id="xZoomInBtn">放大X</button>
    <button id="xZoomOutBtn">缩小X</button>
    <label class="field">X samples <input id="xWindow" type="number" min="20" max="20000" step="10" value="1000"></label>
    <label class="field"><input id="autoScale" type="checkbox" checked>自动缩放</label>
    <label class="field">Y min <input id="yMin" type="number" value="-1000"></label>
    <label class="field">Y max <input id="yMax" type="number" value="1000"></label>
    <div class="status">
      <span>samples <strong id="samples">0</strong></span>
      <span>bad lines <strong id="bad">0</strong></span>
      <span>rate <strong id="rate">0</strong> Hz</span>
      <span>latest <strong id="latest">-</strong></span>
    </div>
  </section>
  <section class="plot-wrap"><canvas id="plot"></canvas></section>
  <section class="legend" id="legend"></section>
</main>
<script>
const colors = [
  "#4cc9f0", "#f72585", "#80ed99", "#ffd166",
  "#b5179e", "#90dbf4", "#f8961e", "#caffbf"
];
const canvas = document.getElementById("plot");
const ctx = canvas.getContext("2d");
const pauseBtn = document.getElementById("pauseBtn");
const clearBtn = document.getElementById("clearBtn");
const xZoomInBtn = document.getElementById("xZoomInBtn");
const xZoomOutBtn = document.getElementById("xZoomOutBtn");
const xWindowInput = document.getElementById("xWindow");
const autoScale = document.getElementById("autoScale");
const yMinInput = document.getElementById("yMin");
const yMaxInput = document.getElementById("yMax");
const samplesEl = document.getElementById("samples");
const badEl = document.getElementById("bad");
const rateEl = document.getElementById("rate");
const latestEl = document.getElementById("latest");
const sourceEl = document.getElementById("source");
const legendEl = document.getElementById("legend");

let paused = false;
let channelCount = 0;
let windowSize = 1000;
let rows = [];
let sampleCount = 0;
let badLines = 0;
let lastCount = 0;
let lastRateTime = performance.now();

function clampWindow(value) {
  if (!Number.isFinite(value)) return 1000;
  return Math.max(20, Math.min(20000, Math.round(value)));
}

function setWindowSize(value) {
  windowSize = clampWindow(value);
  xWindowInput.value = String(windowSize);
  if (rows.length > windowSize) rows.splice(0, rows.length - windowSize);
}

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function setLegend(n) {
  if (n === channelCount) return;
  channelCount = n;
  legendEl.innerHTML = "";
  for (let i = 0; i < n; i++) {
    const item = document.createElement("span");
    item.innerHTML = `<span class="swatch" style="background:${colors[i % colors.length]}"></span>CH${i}`;
    legendEl.appendChild(item);
  }
}

function ingest(msg) {
  if (msg.window && !xWindowInput.dataset.touched) setWindowSize(msg.window);
  if (msg.source) sourceEl.textContent = msg.source;
  if (typeof msg.bad_lines === "number") badLines = msg.bad_lines;
  if (!msg.values || paused) {
    badEl.textContent = badLines;
    return;
  }
  setLegend(msg.values.length);
  rows.push(msg.values);
  if (rows.length > windowSize) rows.splice(0, rows.length - windowSize);
  sampleCount = msg.seq;
  samplesEl.textContent = sampleCount;
  badEl.textContent = badLines;
  latestEl.textContent = msg.values.join(", ");
}

function computeRange() {
  if (!autoScale.checked) {
    let ymin = Number(yMinInput.value);
    let ymax = Number(yMaxInput.value);
    if (!Number.isFinite(ymin)) ymin = -1;
    if (!Number.isFinite(ymax)) ymax = 1;
    if (ymin === ymax) ymax = ymin + 1;
    return [Math.min(ymin, ymax), Math.max(ymin, ymax)];
  }
  let ymin = Infinity;
  let ymax = -Infinity;
  for (const row of rows) {
    for (const v of row) {
      if (v < ymin) ymin = v;
      if (v > ymax) ymax = v;
    }
  }
  if (!Number.isFinite(ymin) || !Number.isFinite(ymax)) return [-1, 1];
  if (ymin === ymax) {
    ymin -= 1;
    ymax += 1;
  }
  const pad = (ymax - ymin) * 0.08;
  return [ymin - pad, ymax + pad];
}

function drawGrid(w, h, ymin, ymax) {
  ctx.strokeStyle = "#2b3038";
  ctx.lineWidth = 1;
  ctx.fillStyle = "#8b95a5";
  ctx.font = "12px -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif";
  for (let i = 0; i <= 8; i++) {
    const x = (w * i) / 8;
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let i = 0; i <= 6; i++) {
    const y = (h * i) / 6;
    const value = ymax - ((ymax - ymin) * i) / 6;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
    ctx.fillText(value.toFixed(1), 8, Math.max(12, y - 3));
  }
}

function draw() {
  const rect = canvas.getBoundingClientRect();
  const w = rect.width;
  const h = rect.height;
  ctx.clearRect(0, 0, w, h);
  const [ymin, ymax] = computeRange();
  drawGrid(w, h, ymin, ymax);

  const n = rows.length;
  const xStep = n > 1 ? w / (windowSize - 1) : w;
  const yScale = h / (ymax - ymin);
  for (let ch = 0; ch < channelCount; ch++) {
    ctx.strokeStyle = colors[ch % colors.length];
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < n; i++) {
      const v = rows[i][ch];
      if (typeof v !== "number") continue;
      const x = w - (n - 1 - i) * xStep;
      const y = h - (v - ymin) * yScale;
      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();
  }

  const now = performance.now();
  if (now - lastRateTime >= 1000) {
    rateEl.textContent = String(Math.round((sampleCount - lastCount) * 1000 / (now - lastRateTime)));
    lastCount = sampleCount;
    lastRateTime = now;
  }
  requestAnimationFrame(draw);
}

pauseBtn.addEventListener("click", () => {
  paused = !paused;
  pauseBtn.textContent = paused ? "继续" : "暂停";
});
clearBtn.addEventListener("click", () => {
  rows = [];
});
xZoomInBtn.addEventListener("click", () => {
  xWindowInput.dataset.touched = "1";
  setWindowSize(windowSize / 2);
});
xZoomOutBtn.addEventListener("click", () => {
  xWindowInput.dataset.touched = "1";
  setWindowSize(windowSize * 2);
});
xWindowInput.addEventListener("change", () => {
  xWindowInput.dataset.touched = "1";
  setWindowSize(Number(xWindowInput.value));
});
window.addEventListener("resize", resizeCanvas);
resizeCanvas();
setWindowSize(windowSize);
draw();

const stream = new EventSource("/stream");
stream.onmessage = (event) => ingest(JSON.parse(event.data));
stream.onerror = () => { sourceEl.textContent = "stream disconnected"; };
</script>
</body>
</html>
"""


@dataclass
class WaveState:
    channels: int
    window: int
    source: str
    seq: int = 0
    bad_lines: int = 0
    samples: deque[list[int]] = field(default_factory=deque)
    subscribers: list[queue.Queue[dict]] = field(default_factory=list)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def push_sample(self, values: list[int]) -> None:
        msg: dict
        with self.lock:
            self.seq += 1
            self.samples.append(values)
            while len(self.samples) > self.window:
                self.samples.popleft()
            msg = {
                "seq": self.seq,
                "values": values,
                "bad_lines": self.bad_lines,
                "window": self.window,
                "source": self.source,
            }
            subscribers = list(self.subscribers)
        for sub in subscribers:
            try:
                sub.put_nowait(msg)
            except queue.Full:
                pass

    def push_bad_line(self) -> None:
        with self.lock:
            self.bad_lines += 1
            msg = {
                "seq": self.seq,
                "bad_lines": self.bad_lines,
                "window": self.window,
                "source": self.source,
            }
            subscribers = list(self.subscribers)
        for sub in subscribers:
            try:
                sub.put_nowait(msg)
            except queue.Full:
                pass

    def subscribe(self) -> queue.Queue[dict]:
        sub: queue.Queue[dict] = queue.Queue(maxsize=2048)
        with self.lock:
            self.subscribers.append(sub)
            for index, values in enumerate(self.samples, start=max(1, self.seq - len(self.samples) + 1)):
                sub.put_nowait({
                    "seq": index,
                    "values": values,
                    "bad_lines": self.bad_lines,
                    "window": self.window,
                    "source": self.source,
                })
        return sub

    def unsubscribe(self, sub: queue.Queue[dict]) -> None:
        with self.lock:
            if sub in self.subscribers:
                self.subscribers.remove(sub)


def parse_line(line: bytes, channels: int) -> list[int] | None:
    text = line.decode("ascii", errors="ignore").strip()
    if not text:
        return None
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != channels:
        return None
    try:
        return [int(part, 10) for part in parts]
    except ValueError:
        return None


def serial_reader(state: WaveState, port: str, baud: int, stop: threading.Event) -> None:
    try:
        import serial
    except ModuleNotFoundError as exc:
        raise SystemExit("pyserial is missing. Install tools/bringup/requirements-bringup.txt") from exc

    pending = bytearray()
    with serial.Serial(port, baudrate=baud, timeout=0.1) as ser:
        while not stop.is_set():
            chunk = ser.read(256)
            if not chunk:
                continue
            for byte in chunk:
                if byte in (10, 13):
                    if pending:
                        values = parse_line(bytes(pending), state.channels)
                        if values is None:
                            state.push_bad_line()
                        else:
                            state.push_sample(values)
                        pending.clear()
                else:
                    pending.append(byte)
                    if len(pending) > 256:
                        pending.clear()
                        state.push_bad_line()


def demo_reader(state: WaveState, stop: threading.Event, rate_hz: float) -> None:
    t0 = time.monotonic()
    period = 1.0 / rate_hz
    next_tick = time.monotonic()
    while not stop.is_set():
        t = time.monotonic() - t0
        values = [
            int(1000 * math.sin(2.0 * math.pi * 1.0 * t)),
            int(800 * math.sin(2.0 * math.pi * 1.0 * t - 2.0 * math.pi / 3.0)),
            int(800 * math.sin(2.0 * math.pi * 1.0 * t + 2.0 * math.pi / 3.0)),
            int(((t * 300.0) % 1200.0) - 600.0),
        ]
        while len(values) < state.channels:
            values.append(random.randint(-500, 500))
        state.push_sample(values[:state.channels])
        next_tick += period
        time.sleep(max(0.0, next_tick - time.monotonic()))


def make_handler(state: WaveState):
    class WaveHandler(BaseHTTPRequestHandler):
        server_version = "NSWaveView/1.0"

        def log_message(self, fmt: str, *args) -> None:
            sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), fmt % args))

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(HTML_PAGE.encode("utf-8"))
                return
            if parsed.path == "/stream":
                self.handle_stream()
                return
            if parsed.path == "/api/parse":
                self.handle_parse_test(parsed.query)
                return
            self.send_error(404)

        def handle_stream(self) -> None:
            sub = state.subscribe()
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            last_ping = time.monotonic()
            try:
                while True:
                    try:
                        msg = sub.get(timeout=0.1)
                        data = json.dumps(msg, separators=(",", ":"))
                        self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                        self.wfile.flush()
                    except queue.Empty:
                        if time.monotonic() - last_ping >= SSE_KEEPALIVE_S:
                            self.wfile.write(b": keepalive\n\n")
                            self.wfile.flush()
                            last_ping = time.monotonic()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                state.unsubscribe(sub)

        def handle_parse_test(self, query: str) -> None:
            params = parse_qs(query)
            raw = params.get("line", [""])[0].encode("ascii", errors="ignore")
            parsed = parse_line(raw, state.channels)
            payload = {"ok": parsed is not None, "values": parsed}
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(payload).encode("utf-8"))

    return WaveHandler


class WaveHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request, client_address) -> None:
        exc_type = sys.exc_info()[0]
        if exc_type in (BrokenPipeError, ConnectionResetError, TimeoutError):
            return
        super().handle_error(request, client_address)


def find_free_port(host: str, preferred: int) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.bind((host, preferred))
            return preferred
        except OSError:
            sock.bind((host, 0))
            return int(sock.getsockname()[1])


def positive_int(value: str) -> int:
    result = int(value, 10)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Realtime serial CSV waveform viewer")
    parser.add_argument("--port", help="serial port, e.g. /dev/cu.usbmodemXXXX")
    parser.add_argument("--baud", type=positive_int, default=DEFAULT_BAUD)
    parser.add_argument("--channels", type=positive_int, default=DEFAULT_CHANNELS)
    parser.add_argument("--window", type=positive_int, default=DEFAULT_WINDOW)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--http-port", type=positive_int, default=DEFAULT_HTTP_PORT)
    parser.add_argument("--demo", action="store_true", help="generate demo waveforms without MCU")
    parser.add_argument("--demo-rate", type=float, default=200.0)
    parser.add_argument("--no-browser", action="store_true", help="do not open browser automatically")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.demo and not args.port:
        raise SystemExit("--port is required unless --demo is used")

    source = "demo" if args.demo else f"{args.port} @ {args.baud}"
    state = WaveState(channels=args.channels, window=args.window, source=source)
    stop = threading.Event()
    reader_args = (state, stop, args.demo_rate) if args.demo else (state, args.port, args.baud, stop)
    reader_target = demo_reader if args.demo else serial_reader
    reader = threading.Thread(target=reader_target, args=reader_args, daemon=True)
    reader.start()

    http_port = find_free_port(args.host, args.http_port)
    server = WaveHTTPServer((args.host, http_port), make_handler(state))
    url = f"http://{args.host}:{http_port}"
    print(f"NS800 WaveView: {url}", flush=True)
    print(f"source={source} channels={args.channels} window={args.window}", flush=True)
    if not args.no_browser:
        webbrowser.open(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping...", flush=True)
    finally:
        stop.set()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
