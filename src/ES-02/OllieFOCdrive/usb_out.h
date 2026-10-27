#ifndef USB_OUT_H
#define USB_OUT_H

#include <stddef.h>
#include <stdint.h>

// Ordered byte path to the USB port (UART0 / CH340). Text and CSV lines from the logger and NavBot link frames share
// one queue and one writer task (link_uart.cpp), so a line never lands inside a frame.
// Waits for queue space, so call it only from a task that may block (the logger sender). Returns the bytes accepted,
// which is len or 0.
size_t UsbWriteBlocking(const uint8_t *data, size_t len);

#endif
