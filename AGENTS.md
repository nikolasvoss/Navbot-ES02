# Agent notes

## Where files go

The folder list below is closed. Never create a top-level folder, and never create a folder inside `docs/`, `lab/`, or `cad/` that this table does not name. If a file fits nowhere, stop and ask the user. After adding, moving, or deleting a file, run `python3 tests/layout/test_repo_layout.py`.

| Path | Contains | Never contains |
| --- | --- | --- |
| Repository root | `README.md` (intro, quick start, links into docs), `AGENTS.md`, `CLAUDE.md`, `.gitignore`, `firmware-build-environment.json` (written by the build script) | Other files |
| `src/<target>/` | Code and files built for or installed on that target: `ES-02/` (robot firmware), `ros2_pc/`, `navbot_link/` (shared by both), `libraries/` (vendored archives) | Docs, tests |
| `scripts/` | Host tools | Tests, test fixtures, docs |
| `tests/<area>/` | Tests, fixtures, and stubs of one area. `tests/scripts/` tests the host tools. ROS 2 package tests stay in their package. | Tools |
| `cad/<system>/` | Files opened in a CAD or EDA tool, a slicer, or a geometry generator. Flat; a part made of several files gets one folder named after the part. | Markdown |
| `docs/<system>/README.md` | Index of the system's pages; links every page below it | Content |
| `docs/<system>/hardware/` | Facts that stay true with different firmware: components, pins, wiring, power, dimensions | Procedures, behavior, plans |
| `docs/<system>/software/` | Implemented behavior and procedures: firmware, ROS 2 nodes, protocols, build, flash, diagnostics, tuning | Plans, dated observations |
| `docs/<system>/plans/` | `roadmap.md` and one file per proposal that is not implemented | Implemented behavior |
| `docs/<system>/assets/` | Every non-Markdown file the docs show or link: images, PDFs, drawings, spreadsheets, vendor documents | Markdown |
| `lab/<system>/` | Dated notes `YYYY-MM-DD-topic.md` with observation, evidence, practical consequence, and a `Status: open` or `Status: concluded` line | Current-state documentation |
| `lab/<system>/captures/` | Raw captures that a lab note or docs page cites | Uncited captures |
| `work/` | Local task files: scope, todo, handoff, uncited captures. Ignored by git. | Anything others need |

`<system>` is one of:

- `robot`: the ESP32 board and everything attached to it.
- `ros2_pc`: the onboard computer and everything attached to it.
- `integration`: the connection between the two, such as the UART link, the cabling between them, and end-to-end bring-up.

Rules:

- Write docs and lab notes in English.
- `docs/` describes the current state. When a proposal is implemented, move its content into `hardware/` or `software/` and delete the proposal.
- A concluded lab note stays as the dated record. Put its result in `docs/` and link the note as evidence. Update a lab note only by changing its status or appending a dated entry.
- Add every new docs page to `docs/<system>/README.md`.
- Name files in lowercase kebab-case without spaces. ROS 2 packages and Python modules keep snake_case.
- `AGENTS.md` holds instructions for agents. Facts about the system belong in `docs/`, and this file links to them.

## Build and flash

Before changing USB, serial, telemetry, or flashing behavior, read the [USB/serial documentation](docs/robot/software/usb-serial-flashing.md). Recheck device names and board settings before each flash.

This machine (the CM5) has no ESP32 toolchain. To build remotely and flash:

1. Commit your changes and run `git push`.
2. Run `scripts/build_remote.sh [ssh-target]` (default `desktop`). It builds the pushed branch on the desktop PC and delivers the binaries to `build/flash`.
3. Flash the absolute path of `build/flash` with `flash_firmware`.

Do not check the desktop for `arduino-cli` or the repo yourself: both are installed there, but only in a login shell, which the script uses.

On a machine with the toolchain, build the default firmware from the project root with `python3 scripts/build_firmware.py`. The script writes Arduino CLI output to `build/flash`; pass `--help` for sketch, board, output, and build-property options. To flash a compiled build, use the configured `navbot_flash` MCP tool `flash_firmware` and pass the absolute build directory from the active worktree. Do not infer that the board is disconnected from an empty sandbox `/dev` or Arduino CLI `operation not permitted`; check the serial MCP `list_ports` before reporting it as disconnected. Close the serial MCP connection before flashing. Keep balance disabled and motor outputs safe.

## How to use the hardware documentation

1. For a hardware-related task, read the [robot hardware overview](docs/robot/hardware/system-overview.md) first. It identifies the relevant board, subsystem, and source of evidence.
2. Open only the relevant section of the [robot electronics reference](docs/robot/hardware/electronics-reference.md) for power paths, GPIOs, connectors, and firmware interfaces. For an exact component pad or unnamed net, search the [netlist](docs/robot/hardware/netlist.md) by designator or net name; do not load the entire table by default.
3. Distinguish sources: the PCB netlist describes intended electrical connections, the current firmware describes configured behavior, the BOM lists proposed purchased parts, and `lab/` records observations on the existing robot. State discrepancies instead of silently choosing one source.
4. Open `cad/robot/ProPrj_NavBot-ES02.epro` or inspect the physical board when the task depends on layout, connector orientation, actual assembly, or a connection the docs do not resolve. Before applying power or changing wiring, verify the relevant pinout and voltage on the physical board.
5. When a new hardware finding changes the overview or reference, update the affected docs page and record the observation, evidence, and practical consequence as a dated note in `lab/robot/`.

The ros2_pc starts at the [ros2_pc index](docs/ros2_pc/README.md). Treat its hardware and attached peripherals as pending until their page marks them verified; do not infer pinouts, power paths, or software behavior.

Run Haiku calls with network access enabled (`sandbox_permissions: "require_escalated"`), because sandboxed calls can stall.
