# Agent notes

Before changing USB, serial, telemetry, or flashing behavior, read the agent-only [USB/serial notes](agent_notes/robot/usb-serial.md) and the canonical [USB/serial documentation](docs/robot/software/development/usb-serial-flashing.md). Keep new hardware findings in `agent_notes/` with the observation, its evidence, and its practical consequence. Recheck device names and board settings before each flash.

Use `docs/robot/` and `docs/cm5/` for complete hardware and software documentation. `dev_notes/` is for concise developer reminders; `agent_notes/` is agent-only working context and evidence, not a normal developer entry point.

Build the default firmware from the project root with `python3 scripts/build_firmware.py`. The script writes Arduino CLI output to `build/flash`; pass `--help` for sketch, board, output, and build-property options. To flash a compiled build, use the configured `navbot_flash` MCP tool `flash_firmware` and pass the absolute build directory from the active worktree. Check the serial MCP `list_ports` before reporting that the board is disconnected, and close the serial MCP connection before flashing. Keep balance disabled and motor outputs safe.

## How to use the hardware documentation

1. For a hardware-related task, read the canonical [robot hardware overview](docs/robot/hardware/system-overview.md) first. It identifies the relevant board, subsystem, and source of evidence.
2. Open only the relevant section of the [robot hardware reference](docs/robot/hardware/electronics/reference.md) for power paths, GPIOs, connectors, and firmware interfaces. For an exact component pad or unnamed net, search the [hardware netlist](docs/robot/hardware/electronics/netlist.md) by designator or net name; do not load the entire table by default.
3. Distinguish sources: the PCB netlist describes intended electrical connections, the current firmware describes configured behavior, the BOM lists proposed purchased parts, and `agent_notes/` records observations on the existing robot. State discrepancies instead of silently choosing one source.
4. Open `hardware/pcb/ProPrj_NavBot-ES02.epro` or inspect the physical board when the task depends on layout, connector orientation, actual assembly, or a connection the notes do not resolve. Before applying power or changing wiring, verify the relevant pinout and voltage on the physical board.
5. When a new hardware finding changes the overview or reference, update the affected canonical page and record the observation, evidence, and practical consequence in `agent_notes/robot/hardware-findings/` so the next agent can find it without reopening the PCB project.

The upcoming CM5 has its own canonical documentation tree under `docs/cm5/`. Treat its hardware and attached peripherals as pending until they are selected and verified; do not infer pinouts, power paths, or software behavior.
