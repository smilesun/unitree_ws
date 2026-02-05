#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-}"

if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root (use sudo)." >&2
  exit 1
fi

if [[ -z "$IFACE" ]]; then
  # Prefer USB Ethernet (enx*) with link up.
  IFACE="$(ip -o link show | awk -F': ' '{print $2 " " $9}' | grep -E '^enx' | grep -v 'NO-CARRIER' | awk '{print $1}' | head -n 1 || true)"
fi
if [[ -z "$IFACE" ]]; then
  echo "Could not auto-detect USB Ethernet interface. Pass one explicitly." >&2
  echo "Example: sudo $0 enx00e04c68564c" >&2
  exit 1
fi

echo "Keeping interface: $IFACE"

other_ifaces="$(ip -o -4 addr show | awk '{print $2, $4}' | grep -v "^${IFACE} " | awk '{print $1}' | sort -u)"
if [[ -z "$other_ifaces" ]]; then
  echo "No other IPv4 interfaces to flush."
  exit 0
fi

for dev in $other_ifaces; do
  echo "Flushing and disabling: $dev"
  sudo ip addr flush dev "$dev"
  sudo ip link set "$dev" down
done
