#!/usr/bin/env bash
set -euo pipefail

echo "lsusb (USB devices):"
lsusb
echo "Expected: a line mentioning your USB-C Ethernet adapter (often Realtek/ASIX)." 
echo

echo "NetworkManager devices:"
nmcli device status
echo "Expected: a new ethernet device (e.g., enx*) appears as connected or disconnected."
echo

echo "Plug/unplug the USB-C adapter now. Capturing kernel messages for 10 seconds..."
timeout 10s dmesg -w || true
echo
echo "Expected dmesg patterns:"
echo "- new USB device found"
echo "- usb X-Y: New USB device"
echo "- r8152 or cdc_ether or asix (driver names)"
echo

echo "If no new USB device appears, the adapter may not be enumerating."
echo "If a USB device appears but no network interface, a driver may be missing."
