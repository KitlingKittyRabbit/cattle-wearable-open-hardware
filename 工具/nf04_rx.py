"""NF-04 (BK2425) 接收端：输出稳定的 JSONL 记录供电脑面板读取。

每个有效包只输出一行 ``NF04_HEARTBEAT {...}``；错误也使用
``NF04_ERROR {...}``，不会依赖会变化的自然语言日志。设备 ID 位于心跳
payload，不参与 RF 地址，因此一个接收器可同时接收多台设备。
"""
from machine import Pin, SPI
import time
try:
    import ujson as json
except ImportError:
    import json

CSN = Pin(10, Pin.OUT, value=1)
CE = Pin(9, Pin.OUT, value=0)
spi = SPI(1, baudrate=2000000, sck=Pin(6), miso=Pin(2), mosi=Pin(7))

# CE 由本模块集中管理。寄存器/命令事务可以暂时拉低 CE，但事务结束
# 必须恢复进入 PRX 的状态；这样清 RX_DR 不会把接收端永久留在待机态。
_prx_enabled = False

RF_ADDR = b"PCB32"
FRAME_LEN = 32
HEARTBEAT_TYPE = 1
PROTOCOL_V2 = 2
PROTOCOL_V3 = 3
SD_INIT_OK = 1 << 0
LOG_ACTIVE = 1 << 1
SD_FULL = 1 << 2

# Si24R1/nRF24-compatible register contract used by NF-03.  Pipe 0 is the
# only receive pipe.  ACK/retry are disabled: the sender emits three
# one-way copies and every valid physical copy is delivered to the host.
REG_CONFIG = 0x00
REG_EN_AA = 0x01
REG_EN_RXADDR = 0x02
REG_SETUP_AW = 0x03
REG_SETUP_RETR = 0x04
REG_RF_CH = 0x05
REG_RF_SETUP = 0x06
REG_STATUS = 0x07
REG_RX_ADDR_P0 = 0x0A
REG_TX_ADDR = 0x10
REG_RX_PW_P0 = 0x11
REG_DYNPD = 0x1C
CMD_FLUSH_TX = 0xE1
CMD_FLUSH_RX = 0xE2
STATUS_RX_DR = 0x40
STATUS_TX_DS = 0x20
STATUS_MAX_RT = 0x10
CONFIG_EN_CRC = 0x08
CONFIG_PWR_UP = 0x02
CONFIG_PRIM_RX = 0x01
CONFIG_RX = CONFIG_EN_CRC | CONFIG_PWR_UP | CONFIG_PRIM_RX
SETUP_RETR_DISABLED = 0x00


def emit(kind, record):
    try:
        text = json.dumps(record, separators=(",", ":"))
    except TypeError:  # 部分 MicroPython ujson 版本不接受 separators
        text = json.dumps(record)
    print(kind + " " + text)


def reg_read(reg):
    was_prx = _ce_transaction_begin()
    try:
        CSN.off()
        spi.write(bytes([reg & 0x1F]))
        buf = spi.read(1)
        return buf[0]
    finally:
        CSN.on()
        _ce_transaction_end(was_prx)


def reg_write(reg, val):
    was_prx = _ce_transaction_begin()
    try:
        CSN.off()
        spi.write(bytes([0x20 | (reg & 0x1F)]))
        spi.write(bytes([val]))
    finally:
        CSN.on()
        _ce_transaction_end(was_prx)


def addr_write(reg, addr_bytes):
    was_prx = _ce_transaction_begin()
    try:
        CSN.off()
        spi.write(bytes([0x20 | (reg & 0x1F)]) + addr_bytes)
    finally:
        CSN.on()
        _ce_transaction_end(was_prx)


def command(command_byte):
    was_prx = _ce_transaction_begin()
    try:
        CSN.off()
        spi.write(bytes([command_byte]))
    finally:
        CSN.on()
        _ce_transaction_end(was_prx)


def _ce_transaction_begin():
    """暂停 PRX 并返回事务前是否处于 PRX。"""
    was_prx = _prx_enabled
    if was_prx:
        CE.off()
    return was_prx


def _ce_transaction_end(was_prx):
    """恢复事务前的 CE 状态，即使 SPI 抛异常也不泄漏状态。"""
    if was_prx:
        CE.on()


def prx_pause():
    """初始化、刷新 FIFO 等必须在 CE 低电平时执行。"""
    global _prx_enabled
    CE.off()
    _prx_enabled = False


def prx_resume():
    """进入并保持 Pipe 0 PRX 监听。"""
    global _prx_enabled
    CE.on()
    _prx_enabled = True


def clear_rx_dr():
    """清 RX_DR，同时保证清除后仍恢复 PRX。"""
    reg_write(REG_STATUS, STATUS_RX_DR)


def fifo_read():
    """读取一个固定长度 payload，并保证 CSN/CE 在异常时恢复。"""
    was_prx = _ce_transaction_begin()
    try:
        CSN.off()
        spi.write(bytes([0x61]))
        return spi.read(FRAME_LEN)
    finally:
        CSN.on()
        _ce_transaction_end(was_prx)


def u32_be(data, offset):
    return ((data[offset] << 24) | (data[offset + 1] << 16) |
            (data[offset + 2] << 8) | data[offset + 3])


def decode(data):
    if data is None or len(data) != FRAME_LEN:
        raise ValueError("心跳长度错误")
    version = data[3]
    if version not in (PROTOCOL_V2, PROTOCOL_V3):
        raise ValueError("未知心跳协议版本")
    if data[4] != HEARTBEAT_TYPE:
        raise ValueError("未知心跳消息类型")
    if data[0] == 0 and data[1] == 0 and data[2] == 0:
        raise ValueError("设备 ID 为空")
    flags = data[17] if version == PROTOCOL_V3 else None
    return {
        "version": version,
        "message_type": data[4],
        "device_id": "%02X%02X%02X" % (data[0], data[1], data[2]),
        "seq": u32_be(data, 5),
        "uptime_ms": u32_be(data, 9),
        "qmi_ok": data[13] == 0xA1,
        "icp_ok": data[14] == 0xB1,
        "gzp_ok": data[15] == 0xC1,
        "mts4_ok": data[16] == 0xD1,
        "sd_init_ok": None if flags is None else bool(flags & SD_INIT_OK),
        "log_active": None if flags is None else bool(flags & LOG_ACTIVE),
        "sd_full": None if flags is None else bool(flags & SD_FULL),
    }


def receive_once():
    """处理一次轮询；返回解码后的记录、None（无包）或错误记录。"""
    status = reg_read(REG_STATUS)
    if not (status & STATUS_RX_DR):
        return None
    try:
        data = fifo_read()
        try:
            record = decode(data)
        except Exception as exc:
            emit("NF04_ERROR", {"kind": "invalid_packet", "message": str(exc)})
            return None
        # Do not collapse the sender's three one-way copies here.  The host
        # monitor receives and persists every protocol-valid physical packet;
        # it may annotate repeated (device_id, seq) records, but must not lose
        # their receive time or status fields.
        emit("NF04_HEARTBEAT", record)
        return record
    except Exception as exc:
        emit("NF04_ERROR", {"kind": "receiver_io", "message": str(exc)})
        return None
    finally:
        # 无论 payload 读取或解码是否失败，都清除当前 RX_DR；reg_write
        # 会恢复事务前的 CE 高电平。异常清理失败时也显式恢复 PRX，避免
        # 一次 SPI 故障让后续三十分钟心跳永久无人接收。
        try:
            clear_rx_dr()
        finally:
            prx_resume()


def init_receiver():
    """配置 NF-04 并进入 PRX；初始化失败由调用者报告。"""
    prx_pause()
    time.sleep_ms(5)
    reg_write(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT)
    command(CMD_FLUSH_TX)
    command(CMD_FLUSH_RX)
    reg_write(REG_CONFIG, CONFIG_RX)
    reg_write(REG_EN_AA, 0x00)       # 单向广播，不发送自动 ACK
    reg_write(REG_EN_RXADDR, 0x01)   # only the common Pipe 0 is enabled
    reg_write(REG_SETUP_AW, 0x03)
    reg_write(REG_SETUP_RETR, SETUP_RETR_DISABLED)
    reg_write(REG_RF_CH, 0x4C)
    reg_write(REG_RF_SETUP, 0x26)
    reg_write(REG_RX_PW_P0, FRAME_LEN)
    reg_write(REG_DYNPD, 0x00)
    reg_write(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT)
    setup = reg_read(REG_CONFIG)
    en_aa = reg_read(REG_EN_AA)
    retr = reg_read(REG_SETUP_RETR)
    rf = reg_read(REG_RF_SETUP)
    if (setup != CONFIG_RX or en_aa != 0x00 or
            retr != SETUP_RETR_DISABLED):
        raise RuntimeError("NF-04 无响应")
    addr_write(REG_RX_ADDR_P0, RF_ADDR)
    addr_write(REG_TX_ADDR, RF_ADDR)
    reg_write(REG_CONFIG, CONFIG_RX)
    prx_resume()
    return {"format": "NF04_HEARTBEAT_JSONL", "channel": 76,
            "payload_bytes": 32, "rf_setup": rf,
            "auto_ack_pipe0": False, "setup_retr": SETUP_RETR_DISABLED}


def main():
    emit("NF04_READY", {"format": "NF04_HEARTBEAT_JSONL", "versions": [2, 3]})
    try:
        ready = init_receiver()
        emit("NF04_READY", ready)
    except Exception as exc:
        prx_pause()
        emit("NF04_ERROR", {"kind": "receiver_init", "message": str(exc)})
        return

    while True:
        try:
            if receive_once() is None:
                time.sleep_ms(10)
        except Exception as exc:
            # receive_once 自身已恢复 CE；这里再兜底，覆盖状态读取异常。
            prx_resume()
            emit("NF04_ERROR", {"kind": "receiver_io", "message": str(exc)})
            time.sleep_ms(100)


if __name__ == "__main__":
    main()
