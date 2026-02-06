#!/usr/bin/env bash
# Recreate a FIFO (named pipe)
# Default: /tmp/run_barrier
# Usage:
#   ./recreate_fifo.sh
#   ./recreate_fifo.sh /custom/path

set -euo pipefail

FIFO_PATH="${1:-/tmp/run_barrier}"
# default to /tmp/run_barrier if no argument is provided

echo "Target FIFO: $FIFO_PATH"

# Ensure parent directory exists
DIR="$(dirname "$FIFO_PATH")"
mkdir -p "$DIR"

# If something exists at that path
if [ -e "$FIFO_PATH" ]; then
    if [ -p "$FIFO_PATH" ]; then
        echo "Existing FIFO found. Removing..."
    else
        echo "Warning: Existing file is NOT a FIFO. Removing anyway..."
    fi
    rm -f "$FIFO_PATH"
fi

# Create fresh FIFO
mkfifo "$FIFO_PATH"

echo "FIFO recreated successfully."
ls -l "$FIFO_PATH"

