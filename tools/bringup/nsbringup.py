#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NS800RT7P65 RT-Thread ADC BSP 联调自动化 CLI。

本工具把常见上板动作串成可重复流程：编译、J-Link 烧录、串口日志采集、
FinSH 命令测试和 GDB 快照。脚本只收集证据，不判断 ADC 电气结果是否正确。
"""

from __future__ import annotations

import argparse
import datetime as _dt
import glob
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


PROJECT = "ns800rt7p65_rtthread_adc"
DEFAULT_ELF = Path("build") / f"{PROJECT}.elf"
DEFAULT_BAUD = 115200
DEFAULT_PROMPT = "msh >"
DEFAULT_GDB_PORT = 2331
DEFAULT_JLINK_DEVICE = "NS800RT7P65"
DEFAULT_GDB_DEVICE = "Cortex-M7"
DEFAULT_IFACE = "SWD"
DEFAULT_SPEED = "4000"
DEFAULT_SMOKE_COMMANDS = [
    "version",
    "list_device",
    "adc_sample",
    "adc_regs",
    "adc_unit_read a 0 0",
    "adc_unit_scan a 0 0 4",
]


class BringupError(RuntimeError):
    """联调流程中的可预期错误。"""


@dataclass
class StepResult:
    name: str
    ok: bool
    detail: str = ""
    log: str | None = None


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def now_stamp() -> str:
    return _dt.datetime.now().strftime("%Y%m%d-%H%M%S")


def make_run_dir(root: Path | None = None) -> Path:
    base = root or repo_root() / "build" / "bringup"
    run_dir = base / now_stamp()
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def rel(path: Path) -> str:
    try:
        return str(path.relative_to(repo_root()))
    except ValueError:
        return str(path)


def print_step(name: str, message: str) -> None:
    print(f"[{name}] {message}", flush=True)


def command_exists(name: str) -> bool:
    return shutil.which(name) is not None


def check_pyserial() -> None:
    try:
        import serial  # noqa: F401
    except ModuleNotFoundError as exc:
        raise BringupError(
            "缺少 pyserial。请先执行：.venv/bin/pip install -r requirements-bringup.txt"
        ) from exc


def list_serial_ports() -> list[str]:
    patterns = ["/dev/cu.*", "/dev/tty.*"]
    ports: list[str] = []
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    return sorted(set(ports))


def open_text_log(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    return path.open("w", encoding="utf-8", errors="replace")


def run_process(
    cmd: Sequence[str],
    *,
    cwd: Path,
    log_path: Path,
    timeout: float | None = None,
    env: dict[str, str] | None = None,
) -> int:
    with open_text_log(log_path) as log:
        log.write("$ " + " ".join(cmd) + "\n\n")
        log.flush()
        proc = subprocess.Popen(
            list(cmd),
            cwd=str(cwd),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=env,
        )
        start = time.monotonic()
        assert proc.stdout is not None
        try:
            for line in proc.stdout:
                print(line, end="")
                log.write(line)
                log.flush()
                if timeout is not None and time.monotonic() - start > timeout:
                    proc.terminate()
                    raise subprocess.TimeoutExpired(cmd, timeout)
            return proc.wait()
        except subprocess.TimeoutExpired:
            proc.kill()
            log.write(f"\nTIMEOUT after {timeout}s\n")
            log.flush()
            return 124


def run_build(args: argparse.Namespace, run_dir: Path) -> StepResult:
    cmd = ["make"]
    if args.clean:
        cmd.append("clean")
        log = run_dir / "build-clean.log"
        rc = run_process(cmd, cwd=repo_root(), log_path=log)
        if rc != 0:
            return StepResult("build-clean", False, f"make clean failed: {rc}", rel(log))

    cmd = ["make", f"-j{args.jobs}"]
    log = run_dir / "build.log"
    rc = run_process(cmd, cwd=repo_root(), log_path=log)
    return StepResult("build", rc == 0, f"exit={rc}", rel(log))


def run_flash(args: argparse.Namespace, run_dir: Path) -> StepResult:
    log = run_dir / "flash.log"
    env = os.environ.copy()
    env.update(
        {
            "JLINK_EXE": args.jlink_exe,
            "JLINK_DEVICE": args.jlink_device,
            "JLINK_IF": args.interface,
            "JLINK_SPEED": args.speed,
        }
    )
    rc = run_process(["make", "flash"], cwd=repo_root(), log_path=log, env=env)
    return StepResult("flash", rc == 0, f"exit={rc}", rel(log))


def read_serial_until(
    *,
    port: str,
    baud: int,
    timeout: float,
    expect: str | None,
    log_path: Path,
) -> tuple[bool, str]:
    check_pyserial()
    import serial

    deadline = time.monotonic() + timeout
    chunks: list[str] = []
    with serial.Serial(port, baudrate=baud, timeout=0.1) as ser, open_text_log(log_path) as log:
        while time.monotonic() < deadline:
            data = ser.read(256)
            if not data:
                continue
            text = data.decode("utf-8", errors="replace")
            chunks.append(text)
            log.write(text)
            log.flush()
            print(text, end="", flush=True)
            if expect and expect in "".join(chunks):
                return True, "".join(chunks)
    output = "".join(chunks)
    return (expect is None and bool(output)) or (expect is not None and expect in output), output


def run_serial_log(args: argparse.Namespace, run_dir: Path) -> StepResult:
    log = run_dir / "serial.log"
    ok, output = read_serial_until(
        port=args.port,
        baud=args.baud,
        timeout=args.timeout,
        expect=args.expect,
        log_path=log,
    )
    detail = f"expect={args.expect!r}, bytes={len(output.encode('utf-8', errors='replace'))}"
    return StepResult("serial-log", ok, detail, rel(log))


def run_msh_commands(args: argparse.Namespace, run_dir: Path) -> StepResult:
    check_pyserial()
    import serial

    log_path = run_dir / "msh.log"
    json_path = run_dir / "msh.json"
    commands = args.cmd or DEFAULT_SMOKE_COMMANDS
    results: list[dict[str, object]] = []

    with serial.Serial(args.port, baudrate=args.baud, timeout=0.1) as ser, open_text_log(log_path) as log:
        # 唤醒 shell，清掉可能残留的半行输入。
        serial_write_line(ser, "", args.char_delay)
        time.sleep(0.2)
        ser.read(ser.in_waiting or 1)

        for cmd in commands:
            serial_write_line(ser, cmd, args.char_delay)
            deadline = time.monotonic() + args.timeout
            chunks: list[str] = []
            while time.monotonic() < deadline:
                data = ser.read(256)
                if not data:
                    continue
                text = data.decode("utf-8", errors="replace")
                chunks.append(text)
                log.write(text)
                log.flush()
                print(text, end="", flush=True)
                if args.prompt in "".join(chunks):
                    break
            output = "".join(chunks)
            ok = args.prompt in output
            results.append({"cmd": cmd, "ok": ok, "output": output})

    json_path.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
    ok = all(bool(item["ok"]) for item in results)
    return StepResult("msh", ok, f"{sum(bool(item['ok']) for item in results)}/{len(results)} commands ok", rel(json_path))


def serial_write_line(ser, line: str, char_delay: float) -> None:
    """逐字符发送一行命令，降低 USB 虚拟串口/FinSH 轮询输入丢字节概率。"""
    payload = line + "\r\n"
    if char_delay <= 0:
        ser.write(payload.encode("utf-8"))
        ser.flush()
        return

    for ch in payload:
        ser.write(ch.encode("utf-8"))
        ser.flush()
        time.sleep(char_delay)


def port_open(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.2)
        return sock.connect_ex((host, port)) == 0


def start_gdb_server(args: argparse.Namespace, run_dir: Path) -> tuple[subprocess.Popen[str] | None, Path]:
    log_path = run_dir / "jlink-gdbserver.log"
    if args.use_existing_gdbserver:
        if not port_open("127.0.0.1", args.gdb_port):
            raise BringupError(f"GDB Server 端口 {args.gdb_port} 未打开")
        return None, log_path

    if port_open("127.0.0.1", args.gdb_port):
        raise BringupError(
            f"GDB Server 端口 {args.gdb_port} 已被占用；请关闭旧进程或使用 --use-existing-gdbserver"
        )

    log = open_text_log(log_path)
    cmd = [
        args.jlink_gdbserver,
        "-device",
        args.gdb_device,
        "-if",
        args.interface,
        "-speed",
        args.speed,
        "-port",
        str(args.gdb_port),
    ]
    log.write("$ " + " ".join(cmd) + "\n\n")
    log.flush()
    proc = subprocess.Popen(
        cmd,
        cwd=str(repo_root()),
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    deadline = time.monotonic() + args.server_timeout
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            log.close()
            raise BringupError(f"JLinkGDBServer exited early: {proc.returncode}")
        if port_open("127.0.0.1", args.gdb_port):
            return proc, log_path
        time.sleep(0.1)
    proc.terminate()
    log.close()
    raise BringupError(f"等待 GDB Server 端口 {args.gdb_port} 超时")


def stop_process(proc: subprocess.Popen[str] | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        proc.kill()


def parse_mem_args(items: Iterable[str]) -> list[tuple[str, str]]:
    parsed: list[tuple[str, str]] = []
    for item in items:
        if ":" not in item:
            raise BringupError(f"--mem 格式应为 address:word_count，例如 0x40030000:16：{item}")
        addr, count = item.split(":", 1)
        int(addr, 0)
        int(count, 0)
        parsed.append((addr, count))
    return parsed


def write_gdb_script(args: argparse.Namespace, path: Path) -> None:
    mems = parse_mem_args(args.mem or [])
    lines = [
        "set pagination off",
        "set confirm off",
        f"target remote localhost:{args.gdb_port}",
        "monitor halt",
        "info registers",
        "bt",
        "x/i $pc",
        "x/16wx $sp",
    ]
    for addr, count in mems:
        lines.append(f"x/{count}wx {addr}")
    lines.append("detach")
    lines.append("quit")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_gdb_snapshot(args: argparse.Namespace, run_dir: Path) -> StepResult:
    elf = Path(args.elf)
    if not elf.is_absolute():
        elf = repo_root() / elf
    if not elf.exists():
        return StepResult("gdb-snapshot", False, f"ELF not found: {rel(elf)}")

    server: subprocess.Popen[str] | None = None
    try:
        server, server_log = start_gdb_server(args, run_dir)
        with tempfile.NamedTemporaryFile("w", suffix=".gdb", delete=False, encoding="utf-8") as tmp:
            script_path = Path(tmp.name)
        write_gdb_script(args, script_path)
        log = run_dir / "gdb.txt"
        rc = run_process([args.gdb, str(elf), "-x", str(script_path)], cwd=repo_root(), log_path=log, timeout=args.timeout)
        try:
            script_path.unlink()
        except OSError:
            pass
        detail = f"exit={rc}, server_log={rel(server_log)}"
        return StepResult("gdb-snapshot", rc == 0, detail, rel(log))
    except BringupError as exc:
        return StepResult("gdb-snapshot", False, str(exc))
    finally:
        if not args.keep_gdbserver:
            stop_process(server)


def write_summary(run_dir: Path, results: Sequence[StepResult]) -> Path:
    lines = ["# Bring-up Summary", ""]
    for result in results:
        mark = "PASS" if result.ok else "FAIL"
        line = f"- **{result.name}**: {mark}"
        if result.detail:
            line += f" - {result.detail}"
        if result.log:
            line += f" (`{result.log}`)"
        lines.append(line)
    lines.append("")
    summary = run_dir / "summary.md"
    summary.write_text("\n".join(lines), encoding="utf-8")
    return summary


def cmd_list_serial(_args: argparse.Namespace) -> int:
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found.")
        return 1
    for port in ports:
        print(port)
    return 0


def cmd_build(args: argparse.Namespace) -> int:
    run_dir = make_run_dir(args.out)
    result = run_build(args, run_dir)
    summary = write_summary(run_dir, [result])
    print(f"summary: {rel(summary)}")
    return 0 if result.ok else 1


def cmd_flash(args: argparse.Namespace) -> int:
    run_dir = make_run_dir(args.out)
    result = run_flash(args, run_dir)
    summary = write_summary(run_dir, [result])
    print(f"summary: {rel(summary)}")
    return 0 if result.ok else 1


def cmd_serial_log(args: argparse.Namespace) -> int:
    run_dir = make_run_dir(args.out)
    result = run_serial_log(args, run_dir)
    summary = write_summary(run_dir, [result])
    print(f"summary: {rel(summary)}")
    return 0 if result.ok else 1


def cmd_msh(args: argparse.Namespace) -> int:
    run_dir = make_run_dir(args.out)
    result = run_msh_commands(args, run_dir)
    summary = write_summary(run_dir, [result])
    print(f"summary: {rel(summary)}")
    return 0 if result.ok else 1


def cmd_gdb_snapshot(args: argparse.Namespace) -> int:
    run_dir = make_run_dir(args.out)
    result = run_gdb_snapshot(args, run_dir)
    summary = write_summary(run_dir, [result])
    print(f"summary: {rel(summary)}")
    return 0 if result.ok else 1


def cmd_smoke(args: argparse.Namespace) -> int:
    run_dir = make_run_dir(args.out)
    results: list[StepResult] = []

    if not args.skip_build:
        print_step("smoke", "build")
        results.append(run_build(args, run_dir))
        if not results[-1].ok:
            summary = write_summary(run_dir, results)
            print(f"summary: {rel(summary)}")
            return 1

    if not args.skip_flash:
        print_step("smoke", "flash")
        results.append(run_flash(args, run_dir))
        if not results[-1].ok:
            summary = write_summary(run_dir, results)
            print(f"summary: {rel(summary)}")
            return 1

    if args.port:
        print_step("smoke", "serial-log")
        results.append(run_serial_log(args, run_dir))
        print_step("smoke", "msh")
        results.append(run_msh_commands(args, run_dir))
    else:
        results.append(StepResult("serial", False, "serial port not provided; skip serial checks"))

    if not args.no_gdb:
        print_step("smoke", "gdb-snapshot")
        results.append(run_gdb_snapshot(args, run_dir))

    summary = write_summary(run_dir, results)
    print(f"summary: {rel(summary)}")
    return 0 if all(result.ok for result in results) else 1


def add_common_output(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--out", type=Path, default=None, help="输出目录，默认 build/bringup/<timestamp>")


def add_build_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--clean", action="store_true", help="构建前执行 make clean")
    parser.add_argument("--jobs", type=int, default=4, help="make 并行任务数，默认 4")


def add_jlink_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--jlink-exe", default="JLinkExe", help="J-Link Commander 命令")
    parser.add_argument("--jlink-device", default=DEFAULT_JLINK_DEVICE, help="J-Link 烧录 device 名")
    parser.add_argument("--interface", default=DEFAULT_IFACE, help="调试接口，默认 SWD")
    parser.add_argument("--speed", default=DEFAULT_SPEED, help="J-Link 速度，默认 4000")


def add_serial_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", required=True, help="串口设备，例如 /dev/cu.usbmodemxxx")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="串口波特率，默认 115200")
    parser.add_argument("--prompt", default=DEFAULT_PROMPT, help="FinSH 提示符，默认 'msh >'")
    parser.add_argument("--char-delay", type=float, default=0.002, help="发送命令时的字符间隔秒数，默认 0.002")


def add_gdb_args(
    parser: argparse.ArgumentParser,
    *,
    include_transport: bool = True,
    include_timeout: bool = True,
) -> None:
    parser.add_argument("--elf", default=str(DEFAULT_ELF), help="ELF 路径")
    parser.add_argument("--gdb", default="arm-none-eabi-gdb", help="GDB 命令")
    parser.add_argument("--jlink-gdbserver", default="JLinkGDBServer", help="J-Link GDB Server 命令")
    parser.add_argument("--gdb-device", default=DEFAULT_GDB_DEVICE, help="GDB Server device 名，默认 Cortex-M7")
    parser.add_argument("--gdb-port", type=int, default=DEFAULT_GDB_PORT, help="GDB Server 端口，默认 2331")
    if include_transport:
        parser.add_argument("--interface", default=DEFAULT_IFACE, help="调试接口，默认 SWD")
        parser.add_argument("--speed", default=DEFAULT_SPEED, help="J-Link 速度，默认 4000")
    parser.add_argument("--mem", action="append", default=[], help="读取内存 address:word_count，可重复")
    if include_timeout:
        parser.add_argument("--timeout", type=float, default=15.0, help="GDB 命令超时秒数")
    parser.add_argument("--server-timeout", type=float, default=5.0, help="等待 GDB Server 启动超时秒数")
    parser.add_argument("--use-existing-gdbserver", action="store_true", help="连接已有 JLinkGDBServer")
    parser.add_argument("--keep-gdbserver", action="store_true", help="执行后保留脚本启动的 GDB Server")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="NS800RT7P65 BSP bring-up automation CLI")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("list-serial", help="列出本机串口设备")
    p.set_defaults(func=cmd_list_serial)

    p = sub.add_parser("build", help="执行 make 构建")
    add_common_output(p)
    add_build_args(p)
    p.set_defaults(func=cmd_build)

    p = sub.add_parser("flash", help="执行 make flash")
    add_common_output(p)
    add_jlink_args(p)
    p.set_defaults(func=cmd_flash)

    p = sub.add_parser("serial-log", help="采集串口日志并等待关键字")
    add_common_output(p)
    add_serial_args(p)
    p.add_argument("--timeout", type=float, default=10.0, help="采集超时秒数")
    p.add_argument("--expect", default=DEFAULT_PROMPT, help="等待关键字，默认 msh >")
    p.set_defaults(func=cmd_serial_log)

    p = sub.add_parser("msh", help="发送 FinSH 命令并收集输出")
    add_common_output(p)
    add_serial_args(p)
    p.add_argument("--timeout", type=float, default=3.0, help="每条命令等待超时秒数")
    p.add_argument("--cmd", action="append", default=[], help="FinSH 命令，可重复")
    p.set_defaults(func=cmd_msh)

    p = sub.add_parser("gdb-snapshot", help="采集 GDB 寄存器/栈/backtrace 快照")
    add_common_output(p)
    add_gdb_args(p)
    p.set_defaults(func=cmd_gdb_snapshot)

    p = sub.add_parser("smoke", help="执行构建/烧录/串口/GDB 冒烟测试")
    add_common_output(p)
    add_build_args(p)
    add_jlink_args(p)
    add_gdb_args(p, include_transport=False, include_timeout=False)
    p.add_argument("--port", default="", help="串口设备；不提供则跳过串口检查")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="串口波特率，默认 115200")
    p.add_argument("--prompt", default=DEFAULT_PROMPT, help="FinSH 提示符，默认 'msh >'")
    p.add_argument("--char-delay", type=float, default=0.002, help="发送命令时的字符间隔秒数，默认 0.002")
    p.add_argument("--expect", default=DEFAULT_PROMPT, help="启动日志等待关键字")
    p.add_argument("--timeout", type=float, default=10.0, help="串口/GDB 默认超时秒数")
    p.add_argument("--skip-build", action="store_true", help="跳过构建")
    p.add_argument("--skip-flash", action="store_true", help="跳过烧录")
    p.add_argument("--no-gdb", action="store_true", help="跳过 GDB 快照")
    p.add_argument("--cmd", action="append", default=[], help="覆盖默认 FinSH 命令，可重复")
    p.set_defaults(func=cmd_smoke)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except BringupError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
