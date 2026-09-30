#!/usr/bin/env python3
"""NF-03 单向三副本广播与 NF-04 物理报文逐条交付回归。"""

from pathlib import Path
import importlib.util
import sys
import types


ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "固件" / "源码" / "main.c").read_text(encoding="utf-8")
PY_RX = (ROOT / "工具" / "nf04_rx.py").read_text(encoding="utf-8")
INO_RX = (ROOT / "工具" / "esp32_nf04_rx.ino").read_text(encoding="utf-8")


def test_register_contract():
    for needle in (
        "#define NF_EN_AA_DISABLED  0x00U",
        "#define NF_EN_RXADDR_DISABLED 0x00U",
        "#define NF_SETUP_RETR_DISABLED 0x00U",
        "#define NF_STATUS_TX_DS    0x20U",
        "nf_write_reg(NF_REG_EN_AA, NF_EN_AA_DISABLED);",
        "nf_write_reg(NF_REG_SETUP_RETR, NF_SETUP_RETR_DISABLED);",
        "TX_DS means only that this copy left the radio",
        "issue3_nf_broadcast_complete(&nf_broadcast);",
    ):
        assert needle in MAIN, needle
    assert "issue3_nf_tx_status" not in MAIN

    for needle in (
        "reg_write(REG_EN_AA, 0x00)",
        "reg_write(REG_SETUP_RETR, SETUP_RETR_DISABLED)",
        "addr_write(REG_RX_ADDR_P0, RF_ADDR)",
        "addr_write(REG_TX_ADDR, RF_ADDR)",
        "def receive_once():",
        "def prx_resume():",
        "clear_rx_dr()",
    ):
        assert needle in PY_RX, needle
    assert "LogicalHeartbeatDedup" not in PY_RX
    assert "logical_dedup.accept" not in PY_RX

    for needle in (
        "radio.setAutoAck(false);",
        "radio.openWritingPipe(addr);",
        "radio.openReadingPipe(0, addr);",
        "radio.startListening();",
    ):
        assert needle in INO_RX, needle
    assert "deliverLogicalHeartbeat" not in INO_RX
    assert "LogicalSeen" not in INO_RX
    assert "errorRecord(\"unknown_version\"" in INO_RX
    assert "errorRecord(\"unknown_type\"" in INO_RX
    assert "errorRecord(\"invalid_device\"" in INO_RX
    assert "radio.stopListening()" not in INO_RX
    assert "radio.setRetries" not in INO_RX


def test_one_way_broadcast_contract():
    # The radio no longer retries/ACKs; deterministic copy timing and delivery
    # of every physical copy are tested by the shared C and receiver tests.
    assert "SETUP_RETR_DISABLED" in MAIN
    assert "NF_EN_AA_DISABLED" in MAIN
    assert "radio.setAutoAck(false);" in INO_RX


class _FakePin:
    OUT = 1

    def __init__(self, number, mode=None, value=0):
        self.number = number
        self.level = value

    def on(self):
        self.level = 1

    def off(self):
        self.level = 0

    def value(self, new_value=None):
        if new_value is not None:
            self.level = new_value
        return self.level


class _FakeSpi:
    def __init__(self, *_args, **_kwargs):
        self.statuses = []
        self.payloads = []
        self.last_command = None
        self.fail_payload_once = False

    def write(self, data):
        self.last_command = data[0]

    def read(self, length):
        if length == 1 and self.last_command == 0x07:
            return bytes([self.statuses.pop(0)])
        if length == 32 and self.last_command == 0x61:
            if self.fail_payload_once:
                self.fail_payload_once = False
                self.payloads.pop(0)
                raise OSError("模拟 FIFO 读取失败")
            return self.payloads.pop(0)
        return bytes([0] * length)


def _load_nf04_rx(fake_spi):
    machine = types.ModuleType("machine")
    machine.Pin = _FakePin
    machine.SPI = lambda *args, **kwargs: fake_spi
    old_machine = sys.modules.get("machine")
    sys.modules["machine"] = machine
    try:
        spec = importlib.util.spec_from_file_location(
            "nf04_rx_regression", ROOT / "工具" / "nf04_rx.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module, old_machine
    except Exception:
        if old_machine is None:
            sys.modules.pop("machine", None)
        else:
            sys.modules["machine"] = old_machine
        raise


def _heartbeat_payload(seq):
    data = bytearray(32)
    data[0:3] = b"ABC"
    data[3] = 3
    data[4] = 1
    data[5:9] = seq.to_bytes(4, "big")
    data[9:13] = (100 + seq).to_bytes(4, "big")
    data[13:17] = bytes((0xA1, 0xB1, 0xC1, 0xD1))
    data[17] = 0x07
    return bytes(data)


def test_micropython_prx_continuity_and_error_recovery():
    """执行 nf04_rx.py 的实际收包函数，而非复制一份理想接收模型。"""
    fake_spi = _FakeSpi()
    module, old_machine = _load_nf04_rx(fake_spi)
    try:
        # 两次相隔很久的心跳：清 RX_DR 后 CE 必须仍为高，第二包仍可交付。
        fake_spi.statuses = [0x40, 0x40]
        fake_spi.payloads = [_heartbeat_payload(0), _heartbeat_payload(1)]
        module.prx_resume()
        first = module.receive_once()
        assert first["seq"] == 0
        assert module.CE.value() == 1
        second = module.receive_once()
        assert second["seq"] == 1
        assert module.CE.value() == 1

        # FIFO 读取中途失败后，实际 finally 路径仍清状态并恢复 PRX；
        # 下一完整包不需要重新启动脚本即可收到。
        fake_spi.statuses = [0x40, 0x40]
        fake_spi.payloads = [_heartbeat_payload(2)]
        fake_spi.fail_payload_once = True
        assert module.receive_once() is None
        assert module.CE.value() == 1
        fake_spi.payloads.append(_heartbeat_payload(3))
        recovered = module.receive_once()
        assert recovered["seq"] == 3
        assert module.CE.value() == 1

        # Three physical copies of one logical heartbeat are all delivered;
        # a later uptime rollback still represents a reboot and is delivered.
        fake_spi.statuses = [0x40, 0x40, 0x40, 0x40]
        fake_spi.payloads = [_heartbeat_payload(4), _heartbeat_payload(4),
                             _heartbeat_payload(4), _heartbeat_payload(0)]
        assert module.receive_once()["seq"] == 4
        assert module.receive_once()["seq"] == 4
        assert module.receive_once()["seq"] == 4
        reboot = module.receive_once()
        assert reboot["seq"] == 0
        assert module.CE.value() == 1

        # A malformed physical payload is still reported as an error rather
        # than silently treated as a duplicate or valid heartbeat.
        invalid = bytearray(_heartbeat_payload(5))
        invalid[3] = 99
        fake_spi.statuses = [0x40]
        fake_spi.payloads = [bytes(invalid)]
        assert module.receive_once() is None
        assert module.CE.value() == 1
    finally:
        if old_machine is None:
            sys.modules.pop("machine", None)
        else:
            sys.modules["machine"] = old_machine


if __name__ == "__main__":
    test_register_contract()
    test_one_way_broadcast_contract()
    test_micropython_prx_continuity_and_error_recovery()
    print("NF-03 单向三副本广播、两端逐条交付与 MicroPython 连续收包回归：通过")
