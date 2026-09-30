# Agent notes

## Documentation structure

Use the Diátaxis modes to choose the structure for each document. Keep each document focused on one mode. Split mixed content into separate documents and link between them.

- **Tutorials** teach through a sequence of steps that produces a visible result.
- **How-to guides** give the steps for completing a specific task. Leave background and teaching to linked docs.
- **Reference docs** state facts for lookup. Mirror the structure of the code or system, and generate content from code when practical.
- **Explanations** describe one topic and its reasons, constraints, history, or alternatives.

Choose the mode by the reader's need: tutorials help readers learn by doing, how-to guides help them get work done, reference docs help them look up facts, and explanations help them understand. Use sentence-case headings and link related modes instead of mixing them into one document.

Before changing USB, Serial, telemetry, or flashing behavior, read [agent_notes/usb-serial.md](agent_notes/usb-serial.md). Keep new hardware findings in `agent_notes/` with the observation, its evidence, and the practical consequence. Recheck device names and board settings before each flash.

For direct serial access, every agent must use the configured serial MCP server. Call `list_ports` before choosing a device, then use its `open`, `read`, `write`, and `close` tools as needed. Set RTS and DTR low after opening unless the task requires otherwise, because these lines can reset the board. Do not send serial data unless the task requires it. Use documented project scripts when their workflow-specific behavior is needed; see [agent_notes/usb-serial.md](agent_notes/usb-serial.md).

For an already compiled Arduino build, use `python3 scripts/upload_firmware.py BUILD_DIR` from the project root. The helper checks the CH340 port and uploads with the documented ESP32-S3 board options. See [agent_notes/usb-serial.md](agent_notes/usb-serial.md) for the build command and USB details.

## How to use the hardware documentation

1. For a hardware-related task, read [agent_notes/hardware-overview.md](agent_notes/hardware-overview.md) first. It identifies the relevant board, subsystem, and source of evidence.
2. Open only the relevant section of [agent_notes/hardware-reference.md](agent_notes/hardware-reference.md) for power paths, GPIOs, connectors, and firmware interfaces. For an exact component pad or unnamed net, search [agent_notes/hardware-netlist.md](agent_notes/hardware-netlist.md) by designator or net name; do not load the entire table by default.
3. Distinguish sources: the PCB netlist describes intended electrical connections, the current firmware describes configured behavior, the BOM lists proposed purchased parts, and `agent_notes/` records observations on the existing robot. State discrepancies instead of silently choosing one source.
4. Open `hardware/pcb/ProPrj_NavBot-ES02.epro` or inspect the physical board when the task depends on layout, connector orientation, actual assembly, or a connection that the notes do not resolve. Before applying power or changing wiring, verify the relevant pinout and voltage on the physical board.
5. When a new hardware finding changes the overview or reference, update the affected note with the observation, evidence, and practical consequence so the next agent can find it without reopening the PCB project.
