#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-}"

if [[ -z "$IFACE" ]]; then
  # Prefer USB Ethernet (enx*) with link up first.
  IFACE="$(ip -o link show | awk -F': ' '{print $2 " " $9}' | grep -E '^enx' | grep -v 'NO-CARRIER' | awk '{print $1}' | head -n 1 || true)"
fi
if [[ -z "$IFACE" ]]; then
  # Then prefer wired interface names (en*) that are actually linked (LOWER_UP).
  IFACE="$(ip -o link show | awk -F': ' '{print $2 " " $9}' | grep -E '^en' | grep -v 'NO-CARRIER' | awk '{print $1}' | head -n 1 || true)"
fi
if [[ -z "$IFACE" ]]; then
  # Fall back to the first wired interface (en*), even if not linked.
  IFACE="$(ip -o link show | awk -F': ' '{print $2}' | grep -E '^en' | head -n 1 || true)"
fi
if [[ -z "$IFACE" ]]; then
  # Final fallback: first non-loopback interface.
  IFACE="$(ip -o link show | awk -F': ' '{print $2}' | grep -v '^lo$' | head -n 1 || true)"
fi

if [[ -z "$IFACE" ]]; then
  echo "Could not auto-detect an interface. Pass one explicitly." >&2
  echo "Example: $0 enp0s31f6" >&2
  exit 1
fi

echo "Using interface: $IFACE"
echo

echo "Link state:"
ip link show "$IFACE"
echo

link_state="$(ip -o link show "$IFACE" | awk '{print $9}' | tr -d '<>')"
if echo "$link_state" | grep -q "NO-CARRIER"; then
  echo "Diagnosis: NO-CARRIER detected. Physical link is down."
  echo "Checks:"
  echo "- Reseat both ends of the Ethernet cable."
  echo "- Try a different cable."
  echo "- Confirm you are plugged into the robot's Ethernet port."
  echo "- If using a USB-Ethernet adapter, try a different port/adapter."
  echo "- Look for link/activity LEDs on both ends."
  echo
fi

echo "IP address on interface:"
ip addr show "$IFACE"
echo

if command -v arp-scan >/dev/null 2>&1; then
  echo "ARP scan (192.168.123.0/24):"
  sudo arp-scan --interface="$IFACE" 192.168.123.0/24
else
  echo "arp-scan not installed. Install with:"
  echo "  sudo apt update && sudo apt install arp-scan"
fi
