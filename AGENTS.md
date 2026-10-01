# Agent notes

Before changing USB, Serial, telemetry, or flashing behavior, read [agent_notes/usb-serial.md](agent_notes/usb-serial.md). Keep new hardware findings in `agent_notes/` with the observation, its evidence, and the practical consequence. Recheck device names and board settings before each flash.

Build the default firmware from the project root with `python3 scripts/build_firmware.py`. The script writes Arduino CLI output to `build/flash`; pass `--help` for sketch, board, output, and build-property options. To flash that compiled build, use `python3 scripts/upload_firmware.py build/flash`. The upload helper checks the CH340 port and verifies the flash. See [agent_notes/usb-serial.md](agent_notes/usb-serial.md) for USB details.

## How to use the hardware documentation

1. For a hardware-related task, read [agent_notes/hardware-overview.md](agent_notes/hardware-overview.md) first. It identifies the relevant board, subsystem, and source of evidence.
2. Open only the relevant section of [agent_notes/hardware-reference.md](agent_notes/hardware-reference.md) for power paths, GPIOs, connectors, and firmware interfaces. For an exact component pad or unnamed net, search [agent_notes/hardware-netlist.md](agent_notes/hardware-netlist.md) by designator or net name; do not load the entire table by default.
3. Distinguish sources: the PCB netlist describes intended electrical connections, the current firmware describes configured behavior, the BOM lists proposed purchased parts, and `agent_notes/` records observations on the existing robot. State discrepancies instead of silently choosing one source.
4. Open `hardware/pcb/ProPrj_NavBot-ES02.epro` or inspect the physical board when the task depends on layout, connector orientation, actual assembly, or a connection that the notes do not resolve. Before applying power or changing wiring, verify the relevant pinout and voltage on the physical board.
5. When a new hardware finding changes the overview or reference, update the affected note with the observation, evidence, and practical consequence so the next agent can find it without reopening the PCB project.
