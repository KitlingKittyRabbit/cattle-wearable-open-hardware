#!/usr/bin/env python3
"""NF-04 心跳 V3 的纯 Python 编解码与机器记录格式。

无线载荷固定为 32 字节，接收端再输出一行 JSONL：
``NF04_HEARTBEAT {json}``。本模块不依赖串口或图形界面，便于主机测试和
监控面板共同复用。V2 仍可读取，但 V2 没有 SD 状态位；未知版本严格拒绝。
"""

from __future__ import annotations

import json
from typing import Any, Dict

FRAME_LEN = 32
HEARTBEAT_TYPE = 1
PROTOCOL_V2 = 2
PROTOCOL_V3 = 3
SUPPORTED_VERSIONS = (PROTOCOL_V2, PROTOCOL_V3)

STATUS_OFFSETS = {
    "qmi_ok": 13,
    "icp_ok": 14,
    "gzp_ok": 15,
    "mts4_ok": 16,
}
SD_INIT_OK = 1 << 0
LOG_ACTIVE = 1 << 1
SD_FULL = 1 << 2


def _u32_be(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 4], "big")


def _status_ok(value: int, expected_high_nibble: int) -> bool:
    return value == (expected_high_nibble | 1)


def _device_id_text(device_id: bytes) -> str:
    if len(device_id) != 3:
        raise ValueError("设备 ID 必须是 3 字节")
    return device_id.hex().upper()


def encode_heartbeat(
    device_id: bytes,
    seq: int,
    uptime_ms: int,
    *,
    qmi_ok: bool = True,
    icp_ok: bool = True,
    gzp_ok: bool = True,
    mts4_ok: bool = True,
    sd_init_ok: bool = True,
    log_active: bool = True,
    sd_full: bool = False,
    version: int = PROTOCOL_V3,
) -> bytes:
    """生成一个用于主机测试的正式 32 字节心跳载荷。"""
    if version not in SUPPORTED_VERSIONS:
        raise ValueError(f"未知心跳协议版本 {version}")
    if not 0 <= seq <= 0xFFFFFFFF or not 0 <= uptime_ms <= 0xFFFFFFFF:
        raise ValueError("序号或单调时间超出 uint32")
    if len(device_id) != 3:
        raise ValueError("设备 ID 必须是 3 字节")
    payload = bytearray(FRAME_LEN)
    payload[0:3] = device_id
    payload[3] = version
    payload[4] = HEARTBEAT_TYPE
    payload[5:9] = seq.to_bytes(4, "big")
    payload[9:13] = uptime_ms.to_bytes(4, "big")
    payload[13] = 0xA1 if qmi_ok else 0xA0
    payload[14] = 0xB1 if icp_ok else 0xB0
    payload[15] = 0xC1 if gzp_ok else 0xC0
    payload[16] = 0xD1 if mts4_ok else 0xD0
    if version == PROTOCOL_V3:
        payload[17] = ((SD_INIT_OK if sd_init_ok else 0) |
                       (LOG_ACTIVE if log_active else 0) |
                       (SD_FULL if sd_full else 0))
    return bytes(payload)


def decode_heartbeat(payload: bytes) -> Dict[str, Any]:
    """严格解析无线 32 字节载荷；未知版本、类型和长度均失败关闭。"""
    if not isinstance(payload, (bytes, bytearray)):
        raise ValueError("心跳载荷不是字节串")
    if len(payload) != FRAME_LEN:
        raise ValueError(f"心跳长度错误：得到 {len(payload)}，应为 {FRAME_LEN}")
    version = payload[3]
    if version not in SUPPORTED_VERSIONS:
        raise ValueError(f"未知心跳协议版本 {version}")
    if payload[4] != HEARTBEAT_TYPE:
        raise ValueError(f"未知心跳消息类型 {payload[4]}")
    if payload[0:3] == b"\x00\x00\x00":
        raise ValueError("设备 ID 为空")
    record: Dict[str, Any] = {
        "version": version,
        "message_type": payload[4],
        "device_id": _device_id_text(bytes(payload[0:3])),
        "seq": _u32_be(payload, 5),
        "uptime_ms": _u32_be(payload, 9),
        "qmi_ok": _status_ok(payload[13], 0xA0),
        "icp_ok": _status_ok(payload[14], 0xB0),
        "gzp_ok": _status_ok(payload[15], 0xC0),
        "mts4_ok": _status_ok(payload[16], 0xD0),
        "sd_init_ok": None,
        "log_active": None,
        "sd_full": None,
    }
    if version == PROTOCOL_V3:
        flags = payload[17]
        record["sd_init_ok"] = bool(flags & SD_INIT_OK)
        record["log_active"] = bool(flags & LOG_ACTIVE)
        record["sd_full"] = bool(flags & SD_FULL)
    return record


def record_to_json(record: Dict[str, Any]) -> str:
    """将解析结果转换成稳定的 JSON 单行，供 USB 串口面板消费。"""
    return json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def parse_receiver_line(line: str) -> Dict[str, Any]:
    """解析接收端输出的一行 NF04_HEARTBEAT JSONL。"""
    if not isinstance(line, str):
        raise ValueError("接收记录不是字符串")
    prefix = "NF04_HEARTBEAT "
    if not line.startswith(prefix):
        raise ValueError("不是 NF04_HEARTBEAT 记录")
    try:
        record = json.loads(line[len(prefix):])
    except json.JSONDecodeError as exc:
        raise ValueError(f"心跳 JSON 损坏：{exc.msg}") from exc
    if not isinstance(record, dict):
        raise ValueError("心跳 JSON 不是对象")
    required = ("version", "message_type", "device_id", "seq", "uptime_ms",
                "qmi_ok", "icp_ok", "gzp_ok", "mts4_ok", "sd_init_ok",
                "log_active", "sd_full")
    if any(key not in record for key in required):
        raise ValueError("心跳 JSON 缺少字段")
    version = record["version"]
    if isinstance(version, bool) or not isinstance(version, int):
        raise ValueError("协议版本类型错误")
    if version not in SUPPORTED_VERSIONS:
        raise ValueError(f"未知心跳协议版本 {version}")
    message_type = record["message_type"]
    if isinstance(message_type, bool) or not isinstance(message_type, int):
        raise ValueError("消息类型类型错误")
    if message_type != HEARTBEAT_TYPE:
        raise ValueError("未知心跳消息类型")
    device_id = record["device_id"]
    if (not isinstance(device_id, str) or len(device_id) != 6 or
            any(char not in "0123456789abcdefABCDEF" for char in device_id)):
        raise ValueError("设备 ID 字段错误")
    for name in ("seq", "uptime_ms"):
        value = record[name]
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFFFFFFFF:
            raise ValueError(f"{name} 必须是 uint32")
    for name in ("qmi_ok", "icp_ok", "gzp_ok", "mts4_ok"):
        value = record[name]
        if not isinstance(value, bool):
            raise ValueError(f"{name} 必须是布尔值")
    for name in ("sd_init_ok", "log_active", "sd_full"):
        value = record[name]
        if isinstance(value, bool):
            continue
        if version == PROTOCOL_V2 and value is None:
            continue
        raise ValueError(f"{name} 必须是布尔值" + ("或 null" if version == PROTOCOL_V2 else ""))
    normalized = dict(record)
    normalized["device_id"] = device_id.upper()
    return normalized
