# Firmware flashing MCP reference

The configured local MCP server `navbot_flash` exposes one tool, `flash_firmware`. It runs on the USB-owning host and delegates to `scripts/upload_firmware.py`. It uses the official Python MCP SDK over stdio. It does not compile firmware.

## Tool arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `build_dir` | `build/flash` | Compiled Arduino output folder. Relative paths refer to the primary checkout. Pass an absolute path for a worktree build. |
| `port` | Automatic CH340 discovery | Select a port only when several CH340 devices are connected. |

The uploader checks CH340 identity and required binaries and calls Arduino CLI with flash verification enabled. When `firmware-build.json` is present, it validates the image hashes and uses the recorded board settings. For older project builds, the MCP reads the board setting from the sibling `.<build folder name>.arduino-build/build.options.json` and passes it to the uploader. Without either metadata file, the uploader uses its documented legacy defaults.

The tool returns success only after Arduino CLI exits successfully. Upload failures are MCP errors with the uploader's output. The server retains at most 12,000 characters in the response, limits each upload to 300 seconds, and rejects overlapping uploads from other instances of this server. A timeout or cancellation kills the upload process group.

Flash only in an explicit safe maintenance state with balance disabled and motor outputs safe. Close any serial MCP connection before uploading. These are agent preconditions; the tool does not measure the robot's operating state.

## Host configuration

The host's `~/.codex/config.toml` registers the primary checkout's `scripts/flash_mcp.py`, launched by `build/flash-mcp-venv/bin/python`. Dependencies are pinned in `scripts/flash_mcp_requirements.txt`. The client timeout is 360 seconds. Only `flash_firmware` is enabled, with `tools.flash_firmware.approval_mode = "approve"`.

The host environment contains Arduino CLI on `PATH` and read/write access to the CH340 device. The Python environment is separate from firmware output under `build/flash`.

## Sandbox and old worktrees

An empty `/dev/ttyUSB*` list in a shell sandbox does not establish that the board is disconnected. Check the serial MCP's `list_ports` and use the flashing MCP. On 2026-10-02, both MCPs discovered the CH340 at `/dev/ttyUSB0` while shell port discovery returned no devices or `operation not permitted`.

Older worktrees can contain instructions that still select the shell uploader. The host's agent instructions select the MCP for Navbot uploads across worktrees. Use the absolute build directory from the active worktree to avoid flashing the primary checkout's build accidentally.
