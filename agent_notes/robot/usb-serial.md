# USB and serial reminder for agents

The robot has appeared as a CH340 USB serial adapter, but the device path can change. Check the serial MCP's `list_ports` before concluding that the board is disconnected. Close the serial MCP connection before flashing.

Use the configured `navbot_flash` MCP tool with the absolute build directory from the active worktree. Keep the balance feature disabled and motor outputs safe. See the complete [USB, serial, and flashing guide](../../docs/robot/software/development/usb-serial-flashing.md).
