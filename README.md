# Cattle Wearable Research Platform

An open hardware and firmware prototype for long-duration monitoring of dairy
cows during the transition period. The device combines motion, pressure,
temperature, storage, and low-rate radio telemetry in a belt-mounted enclosure.

This repository documents a research prototype. It does not claim validated
disease prediction, veterinary effectiveness, animal-safety certification, or
production readiness.

## Research scope

The long-term research goal is to investigate whether wearable signals can
support early warning of periparturient disease. Behavior recognition and
respiratory measurements are intermediate capabilities, not established
clinical outcomes. Short farm trials and video alignment are engineering
pretests rather than evidence of diagnostic performance.

## Hardware overview

| Component | Role |
| --- | --- |
| STM32G030K8T6 | Main microcontroller |
| QMI8658A | Six-axis motion sensing |
| GZP pressure sensor | Pneumatic respiratory signal |
| ICP-20100 | Pressure-assisted posture transitions |
| MTS4 | Surface-temperature sensing |
| SD card | Primary sensor-data storage |
| NF-03 radio | Low-rate health heartbeat only |
| OLED | Startup diagnostics only |

The NF-03 link intentionally carries device-health heartbeats rather than the
sensor stream. Full-rate measurements remain on the SD card.

## Repository layout

```text
3D打印模型/        Editable FreeCAD models and STL exports
原理图/            Exported schematic
硬件资料/          Bill of materials
固件/              Bare-metal STM32 firmware and linker script
工具/              Log conversion, radio monitoring, and regression tests
项目文档/          Validation plans and evidence templates
```

Directory names are retained from the original engineering workspace so that
existing build and validation scripts remain traceable.

Slicer project files are intentionally omitted from the public release because
they can embed local printer profiles and account identifiers. Generate a new
print project from the included CAD or STL source for your own printer.

## Firmware build

Install the Arm GNU toolchain and run:

```bash
make -C 固件
```

The output is written to `固件/编译输出/`, which is ignored by Git.

## Data conversion

The current binary log format is V4. Historical V1–V3 frames remain readable.

```bash
python3 工具/bin2csv.py INPUT.bin OUTPUT.csv
```

The logger records timestamp validity, sensor status, storage status, and
sampling diagnostics. A successful conversion does not establish sensor
accuracy; calibration and field validation are separate requirements.

## Radio heartbeat monitor

The 32-byte V3 heartbeat contains a device identifier, sequence number,
monotonic uptime, sensor status, and SD-card status. Each logical heartbeat is
broadcast three times; the receiver records every physical packet and marks
duplicates instead of discarding them.

```bash
python3 工具/nf04_monitor.py
```

Use `--port` to preselect a serial port and `--csv` to choose the append-only
CSV log. Local CSV and JSONL runtime outputs are excluded from Git.

## Validation status

- firmware timing and binary-log parsing have host-side regression tests;
- the CAD enclosure and belt fixtures are included as editable sources;
- the schematic is currently provided as an exported PDF;
- PCB source, manufacturing outputs, and complete assembly instructions are not
  yet included;
- animal fit, durability, battery life, measurement validity, and welfare must
  be evaluated before field deployment.

For detailed historical engineering notes in Chinese, see
[`docs/README.zh-CN.md`](docs/README.zh-CN.md).

## Licensing

- software and firmware: Apache-2.0;
- original mechanical and hardware design files: CERN-OHL-P-2.0;
- original documentation and photographs: CC-BY-4.0.

See [LICENSES.md](LICENSES.md). Component names and third-party specifications
remain the property of their respective owners.

## Animal welfare and safety

Any animal study should receive the approvals required by the relevant
institution and jurisdiction. Independently assess fit, skin contact, pressure,
thermal behavior, battery protection, water ingress, entanglement, and emergency
removal. Do not use this prototype for veterinary decisions without appropriate
validation and professional oversight.
