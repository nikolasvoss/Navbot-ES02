# Wireless logger hardware finding

The user confirmed on 2026-09-28 that the assembled board uses ESP32-S3-WROOM-1 without external RAM and that its pin assignment is correct. The exported PCB netlist labels U4 as ESP32-S3-WROOM-1-N8R8 while assigning GPIO35–37 to other signals. Treat the user's assembly confirmation as the hardware fact and retain the netlist label as a design discrepancy.

The practical consequence is to budget internal SRAM and leave PSRAM disabled. The free buffer capacity under firmware and Wi-Fi load remains unmeasured. The IMU interrupt pins appear in the U5 symbol but are not connected to U4 in the exported netlist; inspect the physical board before planning interrupt-based FIFO capture.

The full hardware constraints and source distinctions are documented in [docs/robot/hardware/wireless-logging-constraints.md](../../../docs/robot/hardware/wireless-logging-constraints.md).
