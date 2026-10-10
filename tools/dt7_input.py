#!/usr/bin/env python3
"""DT7 keyboard/mouse forwarding for bare-board diagnostics.

Protocol source and hardware proof: docs/workflow/tickets/MC02-0049*.md.
Default: list devices. Transmission requires an explicit verified DJI port.
"""
from __future__ import annotations
import argparse
import json
import math
import os
import struct
import sys
import time
from pathlib import Path

PERIOD = 0.033
MAX_DURATION = 180.0
KEYS = {key: 1 << bit for bit, key in enumerate(
    ("W", "S", "A", "D", "SHIFT", "CTRL", "Q", "E", "R", "F", "G", "Z", "X", "C", "V", "B"))}

def _integer(value, low, high, name):
    if not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in {low}..{high}")
    return value

def _crc(data: bytes, seed: int, poly: int) -> int:
    result = seed
    for byte in data:
        result ^= byte
        for _ in range(8):
            result = (result >> 1) ^ poly if result & 1 else result >> 1
    return result

def encode(*, keys=0, x=0, y=0, left=0, right=0) -> bytes:
    """Official 5.0.2.37 frame; little endian, sequence/reserved/mouse Z zero."""
    keys = _integer(keys, 0, 65535, "keys")
    x = _integer(x, -32768, 32767, "x")
    y = _integer(y, -32768, 32767, "y")
    left = _integer(left, 0, 1, "left")
    right = _integer(right, 0, 1, "right")
    header = bytes((0x5a, 16, 1))
    body = header + bytes((_crc(header, 0x77, 0x8c),))
    body += struct.pack("<HHhhhBBHH", 0, 0x2000, x, y, 0, left, right, keys, 0)
    return body + struct.pack("<H", _crc(body, 0x1862, 0x8408))

class InputState:
    def __init__(self):
        self.release()

    def key(self, name: str, down: bool):
        bit = KEYS[name.upper()]
        self.keys = self.keys | bit if down else self.keys & ~bit

    def button(self, name: str, down: bool):
        if name not in ("left", "right"):
            raise ValueError("button must be left/right")
        setattr(self, name, int(down))

    def mouse(self, x: int, y: int):
        new_x = _integer(self.x + x, -32768, 32767, "accumulated mouse x")
        new_y = _integer(self.y + y, -32768, 32767, "accumulated mouse y")
        self.x, self.y = new_x, new_y

    def packet(self) -> bytes:
        return encode(keys=self.keys, x=self.x, y=self.y, left=self.left, right=self.right)

    def sent(self):
        self.x = self.y = 0

    def release(self):
        self.keys = self.x = self.y = self.left = self.right = 0

class SendError(RuntimeError):
    pass

class Sender:
    """One transport, bounded writes, no reopen or resend after an error."""
    def __init__(self, transport, record):
        self.transport, self.record = transport, record
        self.failed = False
        self.closed = False

    def send(self, state: InputState):
        if self.failed or self.closed:
            raise SendError("sender stopped")
        payload = state.packet()
        try:
            count = self.transport.write(payload)
            if count != len(payload):
                raise SendError(f"short write: {count}/{len(payload)}")
        except Exception as error:
            self.failed = True
            self.record({"event":"write-failed", "hex":payload.hex(), "error":str(error)})
            raise SendError(str(error)) from error
        self.record({"event":"write", "hex":payload.hex(), "bytes":count})
        state.sent()

    def release(self, state: InputState) -> bool:
        state.release()
        if self.failed or self.closed:
            self.record({"event":"release-unconfirmed", "reason":"sender stopped; no firmware acknowledgement"})
            return False
        try:
            self.send(state)
        except SendError:
            self.record({"event":"release-unconfirmed", "reason":"release write failed"})
            return False
        self.record({"event":"release-write-completed", "note":"not a firmware acknowledgement"})
        return True

    def close(self, state: InputState):
        if not self.closed:
            try:
                self.release(state)
            finally:
                self.closed = True
                self.transport.close()

class LiveInput:
    """Window events and replay share InputState/Sender; pause is explicit."""
    def __init__(self, sender: Sender):
        self.sender = sender
        self.state = InputState()
        self.active = False

    def start(self):
        if not self.sender.release(self.state):
            raise SendError("cannot start a stopped sender")
        self.active = True

    def pause(self):
        self.active = False
        self.sender.release(self.state)

    def key(self, name, down):
        if self.active:
            self.state.key(name, down)

    def button(self, name, down):
        if self.active:
            self.state.button(name, down)

    def mouse(self, x, y):
        if self.active:
            self.state.mouse(x, y)

    def tick(self):
        if self.active:
            try:
                self.sender.send(self.state)
            except SendError:
                self.active = False
                self.state.release()
                raise

    def close(self):
        self.active = False
        self.sender.close(self.state)

def validate_replay(data):
    if not isinstance(data, dict) or set(data) != {"duration_s", "actions"}:
        raise ValueError("replay needs duration_s and actions only")
    duration = data["duration_s"]
    if isinstance(duration, bool) or not isinstance(duration, (int, float)) or not math.isfinite(duration) or not 0 < duration <= MAX_DURATION:
        raise ValueError("duration_s must be finite, >0 and <=180")
    actions = data["actions"]
    if not isinstance(actions, list):
        raise ValueError("actions must be a list")
    previous = 0
    for action in actions:
        if not isinstance(action, dict):
            raise ValueError("action must be an object")
        at = action.get("at")
        if isinstance(at, bool) or not isinstance(at, (int,float)) or not math.isfinite(at) or not previous <= at <= duration:
            raise ValueError("action times must be finite, ordered and inside duration_s")
        previous = at
        kind = action.get("type")
        if kind == "key" and set(action) == {"at","type","key","down"}:
            if not isinstance(action["key"], str) or action["key"].upper() not in KEYS or type(action["down"]) is not bool:
                raise ValueError("invalid key action")
        elif kind == "button" and set(action) == {"at","type","button","down"}:
            if action["button"] not in ("left","right") or type(action["down"]) is not bool:
                raise ValueError("invalid button action")
        elif kind == "mouse" and set(action) == {"at","type","x","y"}:
            _integer(action["x"],-32768,32767,"mouse x")
            _integer(action["y"],-32768,32767,"mouse y")
        elif kind != "release" or set(action) != {"at","type"}:
            raise ValueError("unknown action type or fields")
    return float(duration), actions

class Replay:
    def __init__(self, data, sender: Sender):
        self.duration, self.actions = validate_replay(data)
        self.sender = sender
        self.state = InputState()
        self.index = 0
        self.next_send = 0.0
        self.finished = False

    def tick(self, elapsed: float) -> bool:
        if self.finished:
            return False
        if elapsed >= self.duration:
            self.close()
            return False
        if elapsed < self.next_send:
            return True
        while self.index < len(self.actions) and self.actions[self.index]["at"] <= elapsed:
            action = self.actions[self.index]
            kind = action["type"]
            if kind == "key": self.state.key(action["key"],action["down"])
            elif kind == "button": self.state.button(action["button"],action["down"])
            elif kind == "mouse": self.state.mouse(action["x"],action["y"])
            else: self.state.release()
            self.sender.record({"event":"action", "scheduled_s":action["at"], "elapsed_s":elapsed, "action":action})
            self.index += 1
        self.sender.send(self.state)
        # One current state per due tick. Do not emit backlog packets.
        self.next_send = elapsed + PERIOD
        return True

    def close(self):
        if not self.finished:
            self.finished = True
            self.sender.close(self.state)

def devices():
    from serial.tools import list_ports
    return list(list_ports.comports())

def _serial_transport(port):
    import serial
    transport = serial.Serial(port=None, baudrate=921600, bytesize=8, parity="N", stopbits=1,
        timeout=.03, write_timeout=.03, xonxoff=False, rtscts=False, dsrdtr=False)
    transport.dtr = transport.rts = False
    transport.port = port
    time.sleep(.1)  # Matches official OpenPort setup delay.
    try:
        transport.open()
    except Exception:
        transport.close()
        raise
    return transport

def device_info(device):
    return {name:getattr(device,name,None) for name in
            ("device","description","hwid","vid","pid","serial_number","manufacturer")}

def open_device(port, record):
    matches = [item for item in devices() if item.device.upper() == port.upper()]
    if len(matches) != 1:
        raise ValueError(f"explicit port {port} is unavailable")
    device = matches[0]
    if (device.vid,device.pid,device.serial_number) != (0xfff0,0x0008,"0001A0000000"):
        raise ValueError("port identity differs from the verified DJI CONTROLLER; stopped")
    record({"event":"device", **device_info(device), "baud":921600,"dtr":False,"rts":False,
            "protocol_source":"RoboMaster Client 5.0.2.37"})
    return _serial_transport(device.device)

class DryTransport:
    def write(self,data): return len(data)
    def close(self): pass

class EventLog:
    def __init__(self, path: Path):
        path.parent.mkdir(parents=True,exist_ok=True)
        self.file = path.open("x",encoding="utf-8")
        self.start = time.monotonic()
    def __call__(self, row):
        self.file.write(json.dumps({"time":time.time(),"elapsed_s":time.monotonic()-self.start,**row},ensure_ascii=False)+"\n")
        self.file.flush()
    def close(self): self.file.close()

def run_replay(data, sender, dry_run):
    replay = Replay(data,sender)
    try:
        if not sender.release(replay.state): raise SendError("initial release failed")
        if dry_run:
            index = 0
            while replay.tick(index * PERIOD): index += 1
        else:
            start = time.monotonic()
            while replay.tick(time.monotonic()-start):
                # Also wake at the bounded end, rather than holding past it.
                delay = min(replay.next_send,replay.duration)-(time.monotonic()-start)
                if delay > 0: time.sleep(delay)
    finally:
        replay.close()

def run_live(sender):
    import tkinter as tk
    live = LiveInput(sender)
    root = None
    try:
        if not sender.release(live.state): raise SendError("initial release failed")
        root = tk.Tk()
        root.title("DT7 裸板键鼠诊断 — COM4")
        status = tk.StringVar(value="已暂停。点击开始后，在下方区域操作；失焦立即释放并暂停。")
        tk.Label(root,textvariable=status,wraplength=650).pack(padx=12,pady=8)
        controls = tk.Frame(root); controls.pack()
        area = tk.Canvas(root,width=650,height=300,bg="#172330",takefocus=1)
        area.pack(padx=12,pady=12)
        area.create_text(325,150,text="键盘：W S A D Shift Ctrl Q E R F G Z X C V B\n鼠标：相对移动，左右按钮\nEsc / Ctrl+C / 关闭窗口：释放并退出",fill="white")
        held = {}
        previous = None
        stopped = False
        next_due = time.monotonic()

        def pause(reason="已暂停：需要点击开始才能继续。"):
            nonlocal previous
            live.pause(); held.clear(); previous = None
            status.set(reason)
            sender.record({"event":"pause", "reason":reason})

        def start():
            nonlocal previous
            try:
                held.clear(); previous = None
                live.start(); area.focus_set()
                status.set("转发中（33ms 周期）。窗口失焦会释放并暂停。")
                sender.record({"event":"start"})
            except SendError as error: status.set(f"发送已停止：{error}")

        def close(event=None):
            nonlocal stopped
            stopped = True
            live.close(); root.destroy()
            return "break"

        def key(event, down):
            if not live.active: return
            symbol = event.keysym.upper()
            canonical = {"SHIFT_L":"SHIFT","SHIFT_R":"SHIFT","CONTROL_L":"CTRL","CONTROL_R":"CTRL"}.get(symbol,symbol)
            if down and canonical in KEYS: held[event.keycode] = canonical
            if not down: held.pop(event.keycode,None)
            for name in KEYS: live.key(name,name in held.values())
            if canonical in KEYS:
                sender.record({"event":"key", "key":canonical,"down":down})

        def mouse(event):
            nonlocal previous
            if live.active and previous is not None:
                dx,dy = event.x-previous[0],event.y-previous[1]
                try:
                    live.mouse(dx,dy)
                    sender.record({"event":"mouse", "x":dx,"y":dy})
                except ValueError as error: pause(f"鼠标增量越界，已释放暂停：{error}")
            previous = (event.x,event.y) if live.active else None

        def button(name,down):
            live.button(name,down)
            if live.active: sender.record({"event":"button", "button":name,"down":down})

        def lost_focus():
            if not stopped and root.focus_get() is None and live.active: pause("窗口失焦，已释放暂停。请点击开始。")

        def clear_mouse_anchor(event):
            nonlocal previous
            previous = None

        def pump():
            nonlocal next_due
            if stopped: return
            now = time.monotonic()
            if now >= next_due:
                try: live.tick()
                except SendError as error: pause(f"发送失败，已停止；释放未确认：{error}")
                next_due = now + PERIOD
            root.after(max(1,math.ceil((next_due-time.monotonic())*1000)),pump)

        tk.Button(controls,text="开始 / 重新开始",command=start).pack(side="left",padx=8)
        tk.Button(controls,text="释放并暂停",command=pause).pack(side="left",padx=8)
        tk.Button(controls,text="释放并退出",command=close).pack(side="left",padx=8)
        root.bind("<KeyPress>",lambda event:key(event,True))
        root.bind("<KeyRelease>",lambda event:key(event,False))
        root.bind("<Escape>",close); root.bind("<Control-c>",close)
        root.bind("<FocusOut>",lambda event:root.after_idle(lost_focus))
        area.bind("<Motion>",mouse); area.bind("<Leave>",clear_mouse_anchor)
        for number,name in ((1,"left"),(3,"right")):
            area.bind(f"<ButtonPress-{number}>",lambda event,n=name:button(n,True))
            area.bind(f"<ButtonRelease-{number}>",lambda event,n=name:button(n,False))
        root.protocol("WM_DELETE_WINDOW",close)
        root.after(0,pump)
        root.mainloop()
    finally:
        live.close()
        if root is not None:
            try: root.destroy()
            except tk.TclError: pass

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command")
    commands.add_parser("list")
    replay = commands.add_parser("replay")
    replay.add_argument("actions",type=Path)
    replay.add_argument("--dry-run",action="store_true")
    live = commands.add_parser("live")
    for command in (replay,live):
        command.add_argument("--port",help="explicit verified DJI CONTROLLER port")
        command.add_argument("--log",type=Path)
    args = parser.parse_args(argv)
    if args.command in (None,"list"):
        print(json.dumps([device_info(device) for device in devices()],ensure_ascii=False,indent=2))
        return 0
    dry_run = args.command == "replay" and args.dry_run
    if not dry_run and not args.port: parser.error("transmission requires --port")
    data = None
    if args.command == "replay":
        try:
            data = json.loads(args.actions.read_text(encoding="utf-8"))
            validate_replay(data)
        except (ValueError,OSError) as error: parser.error(str(error))
    log_path = args.log or Path("build/dt7-input")/f"{args.command}-{time.time_ns()}-{os.getpid()}.jsonl"
    record = EventLog(log_path)
    sender = None
    try:
        record({"event":"session", "command":args.command,"dry_run":dry_run,"period_s":PERIOD})
        transport = DryTransport() if dry_run else open_device(args.port,record)
        sender = Sender(transport,record)
        if args.command == "replay": run_replay(data,sender,dry_run)
        else: run_live(sender)
        if sender.failed: raise SendError("transmission failed; release not confirmed")
        print(json.dumps({"result":"completed", "dry_run":dry_run,"log":str(log_path)},ensure_ascii=False))
        return 0
    except KeyboardInterrupt:
        record({"event":"interrupted", "reason":"Ctrl+C"})
        return 130
    except Exception as error:
        record({"event":"error", "error":str(error)})
        print(f"stopped: {error}; log={log_path}",file=sys.stderr)
        return 2
    finally:
        if sender and not sender.closed: sender.close(InputState())
        record.close()

if __name__ == "__main__":
    raise SystemExit(main())
