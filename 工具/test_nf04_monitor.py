#!/usr/bin/env python3
"""NF-04 启动首包、V3 状态位、接收端 JSONL/CSV/超时回归。"""

import csv
import json
import os
import tempfile
import unittest

from nf04_monitor import (DEFAULT_TIMEOUT_SECONDS, HeartbeatMonitor,
                          consume_serial_line)
from nf04_protocol import decode_heartbeat, encode_heartbeat, parse_receiver_line, record_to_json


class HeartbeatMonitorTests(unittest.TestCase):
    def test_v3_status_and_big_endian_codec(self):
        payload = encode_heartbeat(b"A01", 0x01020304, 0x10203040,
                                   qmi_ok=False, icp_ok=True, gzp_ok=False,
                                   mts4_ok=True, sd_init_ok=True,
                                   log_active=False, sd_full=True)
        record = decode_heartbeat(payload)
        self.assertEqual(record["version"], 3)
        self.assertEqual(record["device_id"], "413031")
        self.assertEqual(record["seq"], 0x01020304)
        self.assertEqual(record["uptime_ms"], 0x10203040)
        self.assertFalse(record["qmi_ok"])
        self.assertTrue(record["sd_init_ok"])
        self.assertFalse(record["log_active"])
        self.assertTrue(record["sd_full"])

    def test_multi_device_jump_errors_csv_and_timeout(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "heartbeat.csv")
            model = HeartbeatMonitor(path, timeout_seconds=120)
            t0 = "1970-01-01T00:00:00+00:00"
            t1 = "1970-01-01T00:00:30+00:00"
            a0 = encode_heartbeat(b"A01", 0, 10)
            a2 = encode_heartbeat(b"A01", 2, 20)
            b0 = encode_heartbeat(b"B02", 0, 10)
            model.ingest_line("NF04_HEARTBEAT " + record_to_json(decode_heartbeat(a0)), t0)
            model.ingest_line("NF04_HEARTBEAT " + record_to_json(decode_heartbeat(a2)), t1)
            model.ingest_line("NF04_HEARTBEAT " + record_to_json(decode_heartbeat(b0)), t1)
            self.assertEqual(len(model.devices), 2)
            self.assertTrue(model.devices["413031"]["seq_jump"])
            self.assertFalse(model.devices["423032"]["seq_jump"])
            model.ingest_line('NF04_HEARTBEAT {"version":99}', t1)
            model.ingest_line('NF04_HEARTBEAT {bad json}', t1)
            model.ingest_line('NF04_ERROR {"kind":"radio_crc","message":"损坏"}', t1)
            self.assertEqual(len(model.errors), 3)
            rows = list(model.rows(now=31.0))
            self.assertTrue(all(row["online"] for row in rows))
            rows = list(model.rows(now=200.0))
            self.assertTrue(all(not row["online"] for row in rows))
            with open(path, newline="", encoding="utf-8") as source:
                csv_rows = list(csv.DictReader(source))
            self.assertEqual(len(csv_rows), 6)  # 3 心跳 + 3 错误
            self.assertEqual(csv_rows[0]["sd_init_ok"], "1")
            self.assertEqual(csv_rows[1]["seq_jump"], "1")
            self.assertEqual(csv_rows[-1]["error_kind"], "radio_crc")

    def test_three_broadcast_copies_are_all_records_and_device_limit_is_only_state(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "heartbeat.csv")
            model = HeartbeatMonitor(path)
            copies = [encode_heartbeat(b"A01", 0, value)
                      for value in (100, 105, 120)]
            rows = [model.ingest_line(
                "NF04_HEARTBEAT " + record_to_json(decode_heartbeat(payload)),
                "2026-01-01T00:00:00+00:0%d" % index)
                    for index, payload in enumerate(copies)]
            self.assertEqual([row["uptime_ms"] for row in rows], [100, 105, 120])
            self.assertEqual([row["duplicate"] for row in rows], [False, True, True])
            self.assertEqual(model.duplicate_counts["413031"], 2)
            self.assertEqual(model.devices["413031"]["uptime_ms"], 120)

            # Uptime rollback proves a reboot, so the new seq=0 is accepted.
            reboot = decode_heartbeat(encode_heartbeat(b"A01", 0, 10))
            accepted = model.ingest_line(
                "NF04_HEARTBEAT " + record_to_json(reboot),
                "2026-01-01T00:01:00+00:00")
            self.assertEqual(accepted["uptime_ms"], 10)
            with open(path, newline="", encoding="utf-8") as source:
                csv_rows = list(csv.DictReader(source))
            self.assertEqual(len(csv_rows), 4)
            self.assertEqual([row["duplicate"] for row in csv_rows], ["0", "1", "1", "0"])

            # The state table is bounded at 100 devices; oldest state is
            # evicted rather than growing without limit.
            for number in range(100):
                device = number.to_bytes(3, "big")
                if device == b"\x00\x00\x00":
                    device = b"\x00\x00\x01"
                payload = decode_heartbeat(encode_heartbeat(device, 0, 1))
                model.ingest_line("NF04_HEARTBEAT " + record_to_json(payload),
                                  "2026-01-01T00:02:00+00:00")
            self.assertLessEqual(len(model.devices), 100)
            self.assertLessEqual(len(model._seen), 100)

    def test_every_valid_copy_is_complete_ordered_raw_record(self):
        """三份物理副本都进入 CSV/JSONL，完整保留状态字段。"""
        with tempfile.TemporaryDirectory() as directory:
            csv_path = os.path.join(directory, "heartbeat.csv")
            log_path = os.path.join(directory, "nf04-monitor.log")
            model = HeartbeatMonitor(csv_path, log_path=log_path)
            source_records = [
                decode_heartbeat(encode_heartbeat(
                    b"A01", 7, 700, qmi_ok=True, icp_ok=False,
                    gzp_ok=True, mts4_ok=False, sd_init_ok=True,
                    log_active=True, sd_full=False)),
                decode_heartbeat(encode_heartbeat(
                    b"A01", 7, 705, qmi_ok=False, icp_ok=True,
                    gzp_ok=False, mts4_ok=True, sd_init_ok=False,
                    log_active=False, sd_full=True)),
                decode_heartbeat(encode_heartbeat(
                    b"A01", 7, 720, qmi_ok=True, icp_ok=True,
                    gzp_ok=False, mts4_ok=False, sd_init_ok=True,
                    log_active=False, sd_full=True)),
            ]
            receive_times = [
                "2026-01-01T00:00:00.100+00:00",
                "2026-01-01T00:00:05.200+00:00",
                "2026-01-01T00:00:20.300+00:00",
            ]
            for source, received_at in zip(source_records, receive_times):
                line = "NF04_HEARTBEAT " + record_to_json(source)
                model.ingest_line(line, received_at)

            with open(log_path, encoding="utf-8") as source:
                events = [json.loads(line) for line in source if line.strip()]
            raw = [event for event in events if event.get("event") == "heartbeat_raw"]
            self.assertEqual(len(raw), 3)
            self.assertEqual([event["host_receive_time"] for event in raw], receive_times)
            self.assertEqual([event["duplicate"] for event in raw], [False, True, True])
            required = (
                "host_receive_time", "device_id", "version", "message_type",
                "seq", "uptime_ms", "qmi_ok", "icp_ok", "gzp_ok", "mts4_ok",
                "sd_init_ok", "log_active", "sd_full", "duplicate",
            )
            for expected, actual in zip(source_records, raw):
                for field in required:
                    self.assertIn(field, actual)
                for field in required[1:-1]:
                    self.assertEqual(actual[field], expected[field])

            # CSV and JSONL both retain every physical copy in receive order.
            with open(csv_path, newline="", encoding="utf-8") as source:
                csv_rows = list(csv.DictReader(source))
            self.assertEqual(len(csv_rows), 3)
            self.assertEqual([row["duplicate"] for row in csv_rows], ["0", "1", "1"])
            self.assertEqual(len([event for event in events
                                  if event.get("event") == "heartbeat_raw"]), 3)

    def test_raw_records_survive_restart_without_rewriting_history(self):
        """重新打开面板后，原始副本和重复标记状态均从同一追加文件恢复。"""
        with tempfile.TemporaryDirectory() as directory:
            csv_path = os.path.join(directory, "heartbeat.csv")
            log_path = os.path.join(directory, "nf04-monitor.log")
            model = HeartbeatMonitor(csv_path, log_path=log_path)
            for uptime, received_at in ((100, "2026-01-01T00:00:00+00:00"),
                                         (110, "2026-01-01T00:00:05+00:00"),
                                         (120, "2026-01-01T00:00:15+00:00")):
                model.ingest_line(
                    "NF04_HEARTBEAT " + record_to_json(
                        decode_heartbeat(encode_heartbeat(
                            b"A01", 0, uptime, qmi_ok=(uptime != 110)))),
                    received_at)
            with open(log_path, encoding="utf-8") as source:
                before = source.read().splitlines()

            restarted = HeartbeatMonitor(csv_path, log_path=log_path)
            self.assertEqual(restarted.duplicate_counts["413031"], 2)
            with open(log_path, encoding="utf-8") as source:
                self.assertEqual(source.read().splitlines(), before)
            with open(log_path, encoding="utf-8") as source:
                all_events = [json.loads(line) for line in source if line.strip()]
            raw = [event for event in all_events
                   if event.get("event") == "heartbeat_raw"]
            self.assertEqual(len(raw), 3)
            self.assertEqual([event["duplicate"] for event in raw], [False, True, True])

            # A reboot reuses seq=0 with a lower uptime.  Its copy is appended
            # as a new logical heartbeat and never overwrites the old records.
            restarted.ingest_line(
                "NF04_HEARTBEAT " + record_to_json(
                    decode_heartbeat(encode_heartbeat(b"A01", 0, 10))),
                "2026-01-01T00:01:00+00:00")
            with open(log_path, encoding="utf-8") as source:
                after = source.read().splitlines()
            self.assertEqual(after[:len(before)], before)
            all_after = [json.loads(line) for line in after if line.strip()]
            raw_after = [event for event in all_after
                         if event.get("event") == "heartbeat_raw"]
            self.assertEqual(len(raw_after), 4)
            self.assertFalse(raw_after[-1]["duplicate"])

    def test_schema_rejects_bad_types_then_accepts_next_normal_packet(self):
        with tempfile.TemporaryDirectory() as directory:
            log_path = os.path.join(directory, "events.jsonl")
            model = HeartbeatMonitor(os.path.join(directory, "heartbeat.csv"),
                                     log_path=log_path)
            valid = decode_heartbeat(encode_heartbeat(b"A01", 0, 10))
            bad = dict(valid)
            bad["seq"] = "0"
            model.ingest_line("NF04_HEARTBEAT " + record_to_json(bad), "2026-01-01T00:00:00+00:00")
            self.assertEqual(model.devices, {})
            # The rejected string sequence must not poison the next packet's
            # previous["seq"] + 1 path.
            model.ingest_line("NF04_HEARTBEAT " + record_to_json(valid),
                              "2026-01-01T00:00:01+00:00")
            self.assertEqual(model.devices["413031"]["seq"], 0)

            for key, value in (("version", "3"), ("message_type", True),
                               ("device_id", "GG0000"), ("seq", -1),
                               ("uptime_ms", 0x100000000), ("qmi_ok", "true")):
                malformed = dict(valid)
                malformed[key] = value
                model.ingest_line("NF04_HEARTBEAT " + record_to_json(malformed),
                                  "2026-01-01T00:00:02+00:00")
            self.assertEqual(model.devices["413031"]["seq"], 0)
            # V2 may carry null for fields that did not exist in its payload.
            v2 = decode_heartbeat(encode_heartbeat(b"A02", 1, 20, version=2))
            self.assertIsNone(v2["sd_init_ok"])
            model.ingest_line("NF04_HEARTBEAT " + record_to_json(v2),
                              "2026-01-01T00:00:03+00:00")
            self.assertIn("413032", model.devices)
            model.ingest_line("NF04_ERROR []", "2026-01-01T00:00:04+00:00")
            model.ingest_line("NF04_ERROR \"坏记录\"", "2026-01-01T00:00:05+00:00")
            self.assertEqual(model.errors[-2]["error_kind"], "malformed_error")
            self.assertEqual(model.errors[-1]["error_kind"], "malformed_error")
            with open(log_path, encoding="utf-8") as source:
                events = [__import__("json").loads(row)["event"]
                          for row in source if row.strip()]
            self.assertIn("receiver_error", events)

    def test_serial_receive_time_survives_delayed_gui_consumption(self):
        with tempfile.TemporaryDirectory() as directory:
            model = HeartbeatMonitor(os.path.join(directory, "heartbeat.csv"))
            line = "NF04_HEARTBEAT " + record_to_json(
                decode_heartbeat(encode_heartbeat(b"A01", 0, 10)))
            receive_time = "2026-01-01T01:02:03.456+08:00"
            # Simulate the serial worker enqueueing at readline() return and
            # the GUI consuming later: the original timestamp must be kept.
            delayed_message = (line, receive_time)
            record = consume_serial_line(model, *delayed_message)
            self.assertEqual(record["host_receive_time"], receive_time)
            self.assertEqual(model.devices["413031"]["host_receive_time"], receive_time)
            with open(os.path.join(directory, "heartbeat.csv"), encoding="utf-8") as source:
                self.assertIn(receive_time, source.read())

    def test_unknown_and_corrupt_receiver_records_fail_closed(self):
        valid = decode_heartbeat(encode_heartbeat(b"A01", 0, 0))
        line = "NF04_HEARTBEAT " + record_to_json(valid)
        self.assertEqual(parse_receiver_line(line)["device_id"], "413031")
        with self.assertRaises(ValueError):
            parse_receiver_line('NF04_HEARTBEAT {"version":99}')
        with self.assertRaises(ValueError):
            decode_heartbeat(b"\x00" * 31)
        self.assertEqual(DEFAULT_TIMEOUT_SECONDS, 32 * 60)

    def test_persistent_event_log_survives_reconnect_and_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = os.path.join(directory, "nf04-heartbeats.csv")
            log_path = os.path.join(directory, "nf04-monitor.log")
            model = HeartbeatMonitor(csv_path, timeout_seconds=10,
                                     log_path=log_path)
            valid = record_to_json(decode_heartbeat(
                encode_heartbeat(b"A01", 0, 10)))
            line = "NF04_HEARTBEAT " + valid
            model.ingest_line(line, "2026-01-01T00:00:00+00:00")
            model.ingest_line(line, "2026-01-01T00:00:01+00:00")
            model.record_event("serial_disconnected", "模拟断开")
            model.record_event("serial_connected", "模拟重连")
            list(model.rows(now=1767225605.0))
            list(model.rows(now=2000000000.0))  # timeout event is persisted once
            with open(log_path, encoding="utf-8") as source:
                before = source.read().splitlines()
            self.assertGreaterEqual(len(before), 5)

            # A fresh monitor uses the same local file and restores the last
            # device state without truncating or reordering old events.
            restarted = HeartbeatMonitor(csv_path, timeout_seconds=10,
                                         log_path=log_path)
            list(restarted.rows(now=2000000000.0))
            restarted.record_event("serial_connected", "再次重连")
            with open(log_path, encoding="utf-8") as source:
                after = source.read().splitlines()
            self.assertEqual(after[:len(before)], before)
            self.assertEqual(restarted.devices["413031"]["seq"], 0)
            events = [__import__("json").loads(row)["event"] for row in after]
            self.assertEqual(events.count("heartbeat_raw"), 2)
            self.assertIn("timeout", events)
            self.assertEqual(events[-1], "serial_connected")

    def test_clear_display_keeps_files_and_accepts_future_records(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = os.path.join(directory, "heartbeat.csv")
            log_path = os.path.join(directory, "nf04-monitor.log")
            model = HeartbeatMonitor(csv_path, log_path=log_path)
            line = "NF04_HEARTBEAT " + record_to_json(
                decode_heartbeat(encode_heartbeat(b"A01", 3, 30)))
            model.ingest_line(line, "2026-01-01T00:00:00+00:00")
            model.ingest_line('NF04_ERROR {"kind":"radio_crc","message":"损坏"}',
                              "2026-01-01T00:00:01+00:00")
            with open(csv_path, "rb") as source:
                csv_before = source.read()
            with open(log_path, "rb") as source:
                log_before = source.read()
            self.assertEqual(len(list(model.rows(now=1767225600.0))), 1)
            self.assertEqual(len(model.display_errors), 1)

            model.clear_display()
            self.assertEqual(list(model.rows(now=1767225600.0)), [])
            self.assertEqual(model.errors, [])
            self.assertEqual(model.display_errors, [])
            with open(csv_path, "rb") as source:
                self.assertEqual(source.read(), csv_before)
            with open(log_path, "rb") as source:
                self.assertEqual(source.read(), log_before)

            next_line = "NF04_HEARTBEAT " + record_to_json(
                decode_heartbeat(encode_heartbeat(b"A01", 4, 40)))
            received = model.ingest_line(next_line, "2026-01-01T00:00:02+00:00")
            self.assertIsNotNone(received)
            self.assertEqual(len(list(model.rows(now=1767225602.0))), 1)
            with open(csv_path, newline="", encoding="utf-8") as source:
                self.assertEqual(len(list(csv.DictReader(source))), 3)


if __name__ == "__main__":
    unittest.main()
