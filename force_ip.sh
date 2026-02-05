#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-}"
NEW_IP_CIDR="${2:-192.168.123.10/24}"  # Pass desired IP/mask as second argument.

if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root (use sudo)." >&2
  exit 1
fi

if [[ -z "$IFACE" ]]; then
  # Prefer USB Ethernet (enx*) with link up.
  IFACE="$(ip -o link show | awk -F': ' '{print $2 " " $9}' | grep -E '^enx' | grep -v 'NO-CARRIER' | awk '{print $1}' | head -n 1 || true)"
fi
if [[ -z "$IFACE" ]]; then
  # Then prefer wired en* with link up.
  IFACE="$(ip -o link show | awk -F': ' '{print $2 " " $9}' | grep -E '^en' | grep -v 'NO-CARRIER' | awk '{print $1}' | head -n 1 || true)"
fi
if [[ -z "$IFACE" ]]; then
  # Fall back to first en* interface.
  IFACE="$(ip -o link show | awk -F': ' '{print $2}' | grep -E '^en' | head -n 1 || true)"
fi

if [[ -z "$IFACE" ]]; then
  echo "Could not auto-detect a wired interface. Please pass one explicitly." >&2
  echo "Example: sudo $0 enx00e04c68564c 192.168.123.10/24" >&2
  exit 1
fi

echo "Using interface: $IFACE"
echo "Before:"
ip -o -4 addr show "$IFACE" || true

sudo ip addr flush dev "$IFACE"
sudo ip addr add "$NEW_IP_CIDR" dev "$IFACE"
sudo ip link set "$IFACE" up

echo "After:"
ip -o -4 addr show "$IFACE" || true
