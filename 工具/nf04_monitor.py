#!/usr/bin/env python3
"""NF-04 USB 串口心跳监控面板。

启动示例：
    python3 工具/nf04_monitor.py --port /dev/ttyACM0 --csv nf04-heartbeats.csv

接收端输出 JSONL；本程序同时支持多台设备、每份物理接收记录逐条追加、CSV
追加、原始接收事件持久化、重复标记、序号跳变和超时显示。图形界面只在 ``main``
中创建，主机测试可以直接复用模型而无需 DISPLAY 或串口设备。
"""

from __future__ import annotations

import argparse
import csv
import datetime as _datetime
import json
import os
import queue
import threading
from typing import Any, Callable, Dict, Iterable, Optional

from nf04_protocol import parse_receiver_line

HEARTBEAT_PERIOD_SECONDS = 30 * 60
TIMEOUT_MARGIN_SECONDS = 2 * 60
DEFAULT_TIMEOUT_SECONDS = HEARTBEAT_PERIOD_SECONDS + TIMEOUT_MARGIN_SECONDS
MAX_SEEN_DEVICES = 100
DEFAULT_LOG_FILENAME = "nf04-monitor.log"
RAW_HEARTBEAT_EVENT = "heartbeat_raw"

CSV_FIELDS = [
    "record_kind", "host_receive_time", "device_id", "version", "message_type",
    "seq", "uptime_ms", "qmi_ok", "icp_ok", "gzp_ok", "mts4_ok",
    "sd_init_ok", "log_active", "sd_full", "duplicate", "seq_jump", "error_kind",
    "error_message",
]


def host_time() -> str:
    return _datetime.datetime.now().astimezone().isoformat(timespec="milliseconds")


def _csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    return value


class HeartbeatMonitor:
    """无界面的状态模型，逐条保存每份物理接收记录。

    ``log_path`` 是原始接收日志的权威追加文件。每一份通过协议校验的
    心跳都会写入一个 ``heartbeat_raw`` 事件；CSV 也为每一份心跳追加一行。
    ``duplicate`` 只标记同一设备/序号的重复副本，不会阻止记录进入列表、
    CSV 或 JSONL。``devices`` 仍保留每台设备的最新状态，供超时统计使用，
    但不再代替可见的物理接收记录列表。
    """

    def __init__(self, csv_path: str, timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS,
                 log_path: Optional[str] = None):
        self.csv_path = csv_path
        self.log_path = log_path or os.path.join(
            os.path.dirname(os.path.abspath(csv_path)), DEFAULT_LOG_FILENAME)
        self.timeout_seconds = float(timeout_seconds)
        self.devices: Dict[str, Dict[str, Any]] = {}
        self.records: list[Dict[str, Any]] = []
        self.display_records: list[Dict[str, Any]] = []
        # This bounded table only annotates repeated physical copies.  It is
        # never used as an admission gate for a heartbeat record.
        self._seen: Dict[str, Dict[str, Any]] = {}
        self._seen_tick = 0
        self._logical_last: Dict[str, Dict[str, Any]] = {}
        self.duplicate_counts: Dict[str, int] = {}
        self.errors: list[Dict[str, Any]] = []
        self.display_errors: list[Dict[str, Any]] = []
        self.last_status = ""
        self._online_state: Dict[str, bool] = {}
        self.history_count = 0
        self._csv_fields = list(CSV_FIELDS)
        self._ensure_csv()
        self._ensure_log()
        self._restore_log_state()

    def _ensure_csv(self) -> None:
        parent = os.path.dirname(os.path.abspath(self.csv_path))
        if parent:
            os.makedirs(parent, exist_ok=True)
        needs_header = not os.path.exists(self.csv_path) or os.path.getsize(self.csv_path) == 0
        if needs_header:
            with open(self.csv_path, "a", newline="", encoding="utf-8") as output:
                csv.writer(output).writerow(CSV_FIELDS)
            return
        # Never rewrite an existing local log just to add the new marker
        # column.  A legacy header is kept byte-for-byte; new files use the
        # complete schema above, while JSONL remains lossless for all files.
        try:
            with open(self.csv_path, "r", newline="", encoding="utf-8") as source:
                header = next(csv.reader(source), None)
            if header:
                self._csv_fields = header
        except (OSError, csv.Error):
            self._csv_fields = list(CSV_FIELDS)

    def _ensure_log(self) -> None:
        parent = os.path.dirname(os.path.abspath(self.log_path))
        if parent:
            os.makedirs(parent, exist_ok=True)
        if not os.path.exists(self.log_path):
            with open(self.log_path, "a", encoding="utf-8"):
                pass

    def _append_event(self, event: str, received_at: str,
                      values: Optional[Dict[str, Any]] = None) -> None:
        payload: Dict[str, Any] = {
            "event": event,
            "host_receive_time": received_at,
        }
        if values:
            payload.update(values)
        with open(self.log_path, "a", encoding="utf-8") as output:
            output.write(json.dumps(payload, ensure_ascii=False,
                                    sort_keys=True, separators=(",", ":")))
            output.write("\n")
        self.history_count += 1

    def _restore_log_state(self) -> None:
        """恢复记录/状态，但不重写或重新产生历史事件。"""
        try:
            with open(self.log_path, "r", encoding="utf-8") as source:
                for line in source:
                    try:
                        event = json.loads(line)
                    except (TypeError, ValueError):
                        continue
                    if not isinstance(event, dict):
                        continue
                    self.history_count += 1
                    event_name = event.get("event")
                    if event_name == RAW_HEARTBEAT_EVENT:
                        self._restore_raw_heartbeat(event)
                    elif event_name == "heartbeat" and event.get("raw_record") is not False:
                        # Compatibility with logs written before heartbeat_raw
                        # existed. New summary events carry raw_record=false.
                        self._restore_record(event, duplicate=False)
                    elif event_name == "duplicate" and event.get("raw_record") is not False:
                        # Old logs may contain a full duplicate event without
                        # a corresponding raw record. Restore it as a visible
                        # physical record, but never emit another event.
                        self._restore_record(event, duplicate=True)
        except OSError:
            # A read-only or newly created data directory must not prevent
            # the panel from opening; subsequent append errors are reported
            # as monitor events by the caller.
            return

    def _remember_identity(self, record: Dict[str, Any], duplicate: bool) -> None:
        """恢复/更新重复标记状态；此表从不阻止物理记录交付。"""
        device_id = record.get("device_id")
        seq = record.get("seq")
        uptime = record.get("uptime_ms")
        if (not isinstance(device_id, str) or
                isinstance(seq, bool) or not isinstance(seq, int) or
                isinstance(uptime, bool) or not isinstance(uptime, int)):
            return
        if device_id not in self._seen and len(self._seen) >= MAX_SEEN_DEVICES:
            oldest = min(self._seen, key=lambda key: self._seen[key]["tick"])
            self._seen.pop(oldest, None)
            self.devices.pop(oldest, None)
            self._logical_last.pop(oldest, None)
        state = self._seen.setdefault(
            device_id, {"last_uptime": uptime, "recent": {}, "tick": 0})
        self._seen_tick += 1
        state["tick"] = self._seen_tick
        if uptime < state["last_uptime"] and seq == 0:
            state["recent"] = {}
        state["last_uptime"] = max(state["last_uptime"], uptime)
        state["recent"][seq] = uptime
        while len(state["recent"]) > 4:
            state["recent"].pop(next(iter(state["recent"])))

    def _mark_duplicate(self, record: Dict[str, Any]) -> bool:
        """返回重复标记，但无论结果如何都记住并交付该记录。"""
        device_id = record["device_id"]
        seq = record["seq"]
        uptime = record["uptime_ms"]
        state = self._seen.get(device_id)
        if state is None:
            if len(self._seen) >= MAX_SEEN_DEVICES:
                oldest = min(self._seen, key=lambda key: self._seen[key]["tick"])
                self._seen.pop(oldest, None)
                self.devices.pop(oldest, None)
                self._logical_last.pop(oldest, None)
            state = {"last_uptime": uptime, "recent": {}, "tick": 0}
            self._seen[device_id] = state
        elif uptime < state["last_uptime"] and seq == 0:
            # A restarted device may legitimately reuse seq=0.  For every
            # other sequence, a lower-uptime copy is still the same logical
            # heartbeat (for example, radio delivery can be out of order).
            state["recent"] = {}
        self._seen_tick += 1
        state["tick"] = self._seen_tick
        recent = state["recent"]
        duplicate = seq in recent
        state["last_uptime"] = max(state["last_uptime"], uptime)
        recent[seq] = uptime
        while len(recent) > 4:
            recent.pop(next(iter(recent)))
        return duplicate

    def _restore_record(self, event: Dict[str, Any], duplicate: bool) -> None:
        """把已有 JSONL 中的一份物理心跳恢复到记录列表。"""
        device_id = event.get("device_id")
        if not isinstance(device_id, str):
            return
        restored = dict(event)
        restored["record_kind"] = "heartbeat"
        restored["duplicate"] = bool(duplicate)
        restored.setdefault("seq_jump", None)
        self.records.append(restored)
        self.display_records.append(restored)
        self.devices[device_id] = restored
        self._remember_identity(restored, duplicate)
        if duplicate:
            self.duplicate_counts[device_id] = self.duplicate_counts.get(device_id, 0) + 1
        else:
            self._logical_last[device_id] = restored

    def _restore_raw_heartbeat(self, event: Dict[str, Any]) -> None:
        self._restore_record(event, duplicate=event.get("duplicate") is True)

    def _record_heartbeat(self, record: Dict[str, Any], received_at: str,
                          duplicate: Optional[bool] = None,
                          persist: bool = True) -> Dict[str, Any]:
        """将一份有效物理报文加入全部记录；duplicate 只影响标记。"""
        if duplicate is None:
            duplicate = self._mark_duplicate(record)
        else:
            self._remember_identity(record, duplicate)
        previous = self._logical_last.get(record["device_id"])
        seq_jump = (False if duplicate or previous is None else
                    record["seq"] != ((previous["seq"] + 1) & 0xFFFFFFFF))
        raw_record = dict(record)
        raw_record.update({
            "host_receive_time": received_at,
            "duplicate": bool(duplicate),
            "seq_jump": seq_jump if not duplicate else None,
            "record_kind": "heartbeat",
        })
        self.records.append(raw_record)
        self.display_records.append(raw_record)
        # The latest physical copy is the freshest device status, while
        # sequence-gap statistics only advance on the first copy.
        self.devices[record["device_id"]] = raw_record
        if not duplicate:
            self._logical_last[record["device_id"]] = raw_record
        if duplicate:
            self.duplicate_counts[record["device_id"]] = \
                self.duplicate_counts.get(record["device_id"], 0) + 1
        if persist:
            self._append_event(RAW_HEARTBEAT_EVENT, received_at,
                               dict(raw_record, record_kind=RAW_HEARTBEAT_EVENT))
            self._append(raw_record)
            if seq_jump:
                self._append_event("sequence_gap", received_at, {
                    "device_id": record["device_id"], "seq": record["seq"],
                    "message": "检测到序号跳变",
                })
        return raw_record

    def _append(self, values: Dict[str, Any]) -> None:
        row = {field: _csv_value(values.get(field)) for field in self._csv_fields}
        with open(self.csv_path, "a", newline="", encoding="utf-8") as output:
            csv.DictWriter(output, fieldnames=self._csv_fields).writerow(row)

    def record_event(self, event: str, message: str,
                     received_at: Optional[str] = None) -> None:
        """持久化串口连接、异常和其它非心跳事件。"""
        self._append_event(event, received_at or host_time(), {"message": message})

    def ingest_line(self, line: str, received_at: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """消费一行接收端输出；坏行记录到 CSV/错误列表但不抛出。"""
        received_at = received_at or host_time()
        line = line.strip()
        if not line:
            return None
        if line.startswith("NF04_READY "):
            try:
                ready = json.loads(line[len("NF04_READY "):])
                if not isinstance(ready, dict):
                    raise ValueError("NF04_READY JSON 不是对象")
                self.last_status = "接收端就绪：" + str(ready.get("format", "NF04"))
            except (ValueError, TypeError):
                self.last_status = "接收端就绪记录格式异常"
                self._append_event("receiver_error", received_at,
                                   {"message": self.last_status})
            return None
        if line.startswith("NF04_HEARTBEAT "):
            try:
                record = parse_receiver_line(line)
            except ValueError as exc:
                error = {"record_kind": "error", "host_receive_time": received_at,
                         "error_kind": "invalid_heartbeat", "error_message": str(exc)}
                self.errors.append(error)
                self.display_errors.append(error)
                self._append(error)
                self._append_event("receiver_error", received_at, error)
                return None
            return self._record_heartbeat(record, received_at)
        if line.startswith("NF04_ERROR "):
            try:
                error_obj = json.loads(line[len("NF04_ERROR "):])
                if not isinstance(error_obj, dict):
                    kind, message = "malformed_error", "NF04_ERROR JSON 不是对象"
                else:
                    kind = str(error_obj.get("kind", "receiver"))
                    message = str(error_obj.get("message", "接收端错误"))
            except (ValueError, TypeError) as exc:
                kind, message = "malformed_error", str(exc)
            error = {"record_kind": "error", "host_receive_time": received_at,
                     "error_kind": kind, "error_message": message}
            self.errors.append(error)
            self.display_errors.append(error)
            self._append(error)
            self._append_event("receiver_error", received_at, error)
            return None
        # READY/banners and unknown serial text are harmless but visible.
        error = {"record_kind": "error", "host_receive_time": received_at,
                 "error_kind": "unknown_record", "error_message": line[:240]}
        self.errors.append(error)
        self.display_errors.append(error)
        self._append(error)
        self._append_event("receiver_error", received_at, error)
        return None

    def clear_display(self) -> None:
        """只清空当前界面缓存；持久日志、状态和后续接收不受影响。"""
        self.display_records.clear()
        # ``errors`` is the in-memory error display used by callers from the
        # original model API; persistent error events remain in JSONL.
        self.errors.clear()
        self.display_errors.clear()
        self.last_status = ""

    def rows(self, now: Optional[float] = None) -> Iterable[Dict[str, Any]]:
        now = now if now is not None else _datetime.datetime.now().timestamp()
        # Timeout is a per-device property of the newest physical record, but
        # the visible rows are every received copy in receive order.
        online_by_device: Dict[str, bool] = {}
        for device_id, latest in self.devices.items():
            try:
                received = _datetime.datetime.fromisoformat(
                    latest["host_receive_time"]).timestamp()
                age = max(0.0, now - received)
            except (KeyError, ValueError, TypeError):
                age = float("inf")
            online = age <= self.timeout_seconds
            online_by_device[device_id] = online
            previous_online = self._online_state.get(device_id)
            if previous_online is True and not online:
                self._append_event("timeout", host_time(), {
                    "device_id": device_id,
                    "message": "超过心跳超时阈值，疑似漏收或设备停止",
                })
            elif previous_online is False and online:
                self._append_event("recovered", host_time(), {
                    "device_id": device_id,
                    "message": "设备重新在线",
                })
            elif previous_online is None and not online:
                # A restarted panel has no in-memory transition history, but
                # a stale restored heartbeat still represents a timeout that
                # must be visible in the persistent event stream.
                self._append_event("timeout", host_time(), {
                    "device_id": device_id,
                    "message": "恢复历史后发现设备已超过心跳超时阈值",
                })
            self._online_state[device_id] = bool(online)
        for record in self.display_records:
            # Test callers may supply ISO time; preserve each record's own
            # receive time while showing current device online state.
            try:
                received = _datetime.datetime.fromisoformat(
                    record["host_receive_time"]).timestamp()
                age = max(0.0, now - received)
            except (KeyError, ValueError, TypeError):
                age = float("inf")
            row = dict(record)
            row["age_seconds"] = age
            row["online"] = online_by_device.get(record.get("device_id"), False)
            yield row


def consume_serial_line(model: HeartbeatMonitor, line: str,
                        received_at: str) -> Optional[Dict[str, Any]]:
    """消费串口线程入队的消息；received_at 必须来自 readline() 返回时。"""
    return model.ingest_line(line, received_at)


def _display_bool(value: Any) -> str:
    if value is None:
        return "未知"
    return "是" if value else "否"


def available_serial_ports(
        provider: Optional[Callable[[], Iterable[Any]]] = None) -> list[Dict[str, str]]:
    """列出当前串口；provider 仅供无硬件主机测试注入。"""
    if provider is None:
        try:
            from serial.tools import list_ports  # type: ignore
            provider = list_ports.comports
        except ImportError:
            return []
    ports = []
    try:
        for info in provider():
            device = str(getattr(info, "device", "") or "")
            if not device:
                continue
            description = str(getattr(info, "description", "") or "未知设备")
            ports.append({"device": device, "description": description,
                          "display": f"{device} — {description}"})
    except Exception:
        return []
    return sorted(ports, key=lambda item: item["device"])


class SerialReaderController:
    """可停止、可重连的串口读取线程；不依赖 Tk，便于主机模拟测试。"""

    def __init__(self, open_serial: Callable[..., Any],
                 on_line: Callable[[str, str], None],
                 on_event: Callable[[str, str], None]):
        self._open_serial = open_serial
        self._on_line = on_line
        self._on_event = on_event
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._connection = None
        self._lock = threading.Lock()
        self.port: Optional[str] = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def connect(self, port: str, baud: int) -> bool:
        if self.running or not port:
            return False
        self.port = port
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(port, baud), daemon=True)
        self._thread.start()
        return True

    def disconnect(self) -> None:
        self._stop.set()
        with self._lock:
            connection = self._connection
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)

    def _run(self, port: str, baud: int) -> None:
        connection = None
        try:
            connection = self._open_serial(port, baudrate=baud, timeout=1)
            with self._lock:
                self._connection = connection
            self._on_event("connected", f"已连接 {port}")
            while not self._stop.is_set():
                raw = connection.readline()
                if raw:
                    # 时间戳必须紧跟 readline() 返回；GUI 队列/刷新延迟
                    # 不能改变电脑实际收到该行的时刻。
                    received_at = host_time()
                    line = raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else str(raw)
                    self._on_line(line, received_at)
        except Exception as exc:
            if not self._stop.is_set():
                self._on_event("error", f"串口异常：{exc}")
        finally:
            with self._lock:
                self._connection = None
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
            self._on_event("disconnected", "已断开，可重新选择并连接")


def run_panel(port: Optional[str], baud: int, csv_path: str,
              timeout_seconds: float, log_path: Optional[str] = None) -> int:
    try:
        import serial  # type: ignore
        import tkinter as tk
        from tkinter import ttk
    except ImportError as exc:
        print(f"无法启动监控面板：缺少串口或 Tk 依赖（{exc}）")
        return 2

    model = HeartbeatMonitor(csv_path, timeout_seconds, log_path)
    incoming: "queue.Queue[tuple[str, Any]]" = queue.Queue()

    def on_line(line: str, received_at: str) -> None:
        incoming.put(("line", (line, received_at)))

    def on_event(kind: str, message: str) -> None:
        incoming.put(("event", (kind, message)))

    reader = SerialReaderController(
        lambda device, **kwargs: serial.Serial(device, **kwargs), on_line, on_event)

    root = tk.Tk()
    root.title("NF-04 心跳监控")
    root.geometry("1450x500")
    control = ttk.Frame(root)
    control.pack(fill="x", padx=8, pady=(8, 0))
    ttk.Label(control, text="接收端串口：").pack(side="left")
    port_var = tk.StringVar()
    port_combo = ttk.Combobox(control, textvariable=port_var, state="readonly", width=48)
    port_combo.pack(side="left", padx=4)
    port_map: Dict[str, str] = {}
    preferred_port = port

    status_label = tk.StringVar()

    def refresh_ports() -> None:
        nonlocal port_map
        entries = available_serial_ports()
        port_map = {entry["display"]: entry["device"] for entry in entries}
        displays = list(port_map)
        if preferred_port and preferred_port not in port_map.values():
            display = f"{preferred_port} — 命令行预选（当前未发现）"
            port_map[display] = preferred_port
            displays.append(display)
        port_combo["values"] = displays
        if preferred_port and preferred_port in port_map.values():
            port_var.set(next(label for label, device in port_map.items()
                              if device == preferred_port))
        elif displays and not port_var.get():
            port_var.set(displays[0])
        elif not displays:
            port_var.set("")
            status_label.set("未发现接收端；请插入接收端后点击刷新串口")
        if displays and not reader.running:
            status_label.set("请选择串口后点击连接")

    def connect_selected() -> None:
        display = port_var.get()
        device = port_map.get(display)
        if not device:
            status_label.set("未选择串口；请先刷新串口列表")
            return
        if reader.connect(device, baud):
            status_label.set(f"正在连接 {device}……")
        else:
            status_label.set("串口已连接或仍在断开，请稍候")

    def disconnect_selected() -> None:
        reader.disconnect()
        status_label.set("正在断开串口……")

    ttk.Button(control, text="刷新串口", command=refresh_ports).pack(side="left", padx=4)
    ttk.Button(control, text="连接", command=connect_selected).pack(side="left", padx=4)
    ttk.Button(control, text="断开", command=disconnect_selected).pack(side="left", padx=4)
    columns = ("device_id", "online", "host_time", "seq", "uptime", "qmi", "icp",
               "gzp", "mts4", "sd_init", "log", "sd_full", "duplicate", "seq_jump")
    headings = ("设备ID", "在线", "电脑接收时间", "序号", "运行时间(ms)", "QMI", "ICP",
                "GZP", "MTS4", "SD初始化", "正在记录", "SD已满", "重复副本", "序号跳变")
    table = ttk.Treeview(root, columns=columns, show="headings", height=17)
    for column, heading in zip(columns, headings):
        table.heading(column, text=heading)
        table.column(column, width=105 if column != "host_time" else 190, anchor="center")
    table.pack(fill="both", expand=True, padx=8, pady=8)
    ttk.Label(root, textvariable=status_label, anchor="w").pack(fill="x", padx=8)
    error_text = tk.StringVar()
    ttk.Label(root, textvariable=error_text, anchor="w", foreground="#a00000").pack(
        fill="x", padx=8)

    def process_incoming(update_status: bool = True) -> None:
        while True:
            try:
                kind, value = incoming.get_nowait()
            except queue.Empty:
                break
            if kind == "line":
                line, received_at = value
                consume_serial_line(model, line, received_at)
                if update_status and model.display_errors:
                    last = model.display_errors[-1]
                    error_text.set(f"错误：{last['error_kind']} {last['error_message']}")
            elif kind == "event":
                event_kind, message = value
                status_label.set(message)
                model.record_event("serial_" + event_kind, message)
                if update_status and event_kind == "error":
                    error_text.set(message)

    def clear_display() -> None:
        model.clear_display()
        for item in table.get_children():
            table.delete(item)
        error_text.set("")
        status_label.set("已清空当前显示；CSV/JSONL 未改动，仍会继续接收")

    ttk.Button(control, text="清空显示", command=clear_display).pack(side="left", padx=4)

    def refresh() -> None:
        process_incoming()
        for item in table.get_children():
            table.delete(item)
        # Newest physical receive record is shown first, but all records stay
        # in the tree; no device row is overwritten by a duplicate copy.
        for record in reversed(list(model.rows())):
            table.insert("", "end", values=(
                record["device_id"], "在线" if record["online"] else "超时",
                record["host_receive_time"], record["seq"], record["uptime_ms"],
                _display_bool(record["qmi_ok"]), _display_bool(record["icp_ok"]),
                _display_bool(record["gzp_ok"]), _display_bool(record["mts4_ok"]),
                _display_bool(record["sd_init_ok"]), _display_bool(record["log_active"]),
                _display_bool(record["sd_full"]),
                "是" if record["duplicate"] else "否",
                "是" if record["seq_jump"] else "否"))
        root.after(500, refresh)

    status_label.set(f"监控已打开；超时阈值 {timeout_seconds:.0f} 秒；CSV：{csv_path}；日志：{model.log_path}")
    refresh_ports()
    refresh()
    def close_panel() -> None:
        # Stop/join the reader first, then consume anything it queued after the
        # last GUI refresh.  The timestamp was captured in the reader thread;
        # draining here preserves it instead of dropping a line on shutdown.
        reader.disconnect()
        process_incoming(update_status=False)
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close_panel)
    root.mainloop()
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="NF-04 JSONL 心跳监控面板")
    parser.add_argument("--port", default=None,
                        help="可选：预选 ESP32-C3 USB 串口，例如 /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--csv", default="nf04-heartbeats.csv")
    parser.add_argument("--log", default=None,
                        help="可选：本地追加事件日志；默认与 CSV 同目录的 nf04-monitor.log")
    parser.add_argument("--timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS,
                        help=f"超时阈值，默认 {DEFAULT_TIMEOUT_SECONDS:.0f} 秒")
    args = parser.parse_args(argv)
    return run_panel(args.port, args.baud, args.csv, args.timeout_seconds, args.log)


if __name__ == "__main__":
    raise SystemExit(main())
