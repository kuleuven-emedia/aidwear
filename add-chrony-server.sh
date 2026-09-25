#!/bin/bash
set -euo pipefail

# Check root privileges
if [ "$EUID" -ne 0 ]; then
  echo "Error: Please run as root (or use sudo)." >&2
  exit 1
fi

# Usage helper
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <server_address> [minpoll] [maxpoll] [iburst (0|1)]"
  echo "Example: $0 192.168.1.100 3 6 1"
  echo "Example: $0 pool.ntp.org 16 16 0"
  exit 1
fi

SERVER="$1"
MINPOLL="${2:-}"
MAXPOLL="${3:-}"
IBURST="${4:-1}" # Default to iburst enabled (1)

CONFIG_FILE="/etc/chrony.conf"

# 1. Prevent duplicate entries
if grep -qE "^\s*(server|pool)\s+${SERVER}\b" "$CONFIG_FILE"; then
  echo "Warning: Server '${SERVER}' already exists in ${CONFIG_FILE}. Skipping."
  exit 0
fi

# 2. Build the configuration line
NEW_LINE="server ${SERVER}"

[ -n "$MINPOLL" ] && NEW_LINE="${NEW_LINE} minpoll ${MINPOLL}"
[ -n "$MAXPOLL" ] && NEW_LINE="${NEW_LINE} maxpoll ${MAXPOLL}"
[ "$IBURST" -eq 1 ] && NEW_LINE="${NEW_LINE} iburst"

# 3. Append to chrony.conf
echo "Adding line: '${NEW_LINE}'"
echo "$NEW_LINE" >> "$CONFIG_FILE"

# 4. Validate configuration syntax before restarting
if command -v chronyd &> /dev/null; then
  if chronyd -p &> /dev/null; then
    echo "Configuration valid. Restarting chronyd..."
    systemctl restart chronyd
    echo "Done! Server added successfully."
  else
    echo "Error: Invalid chrony configuration detected! Reverting line." >&2
    sed -i "$ d" "$CONFIG_FILE" # Remove last line
    exit 1
  fi
else
  systemctl restart chronyd
fi
