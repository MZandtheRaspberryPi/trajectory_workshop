#!/usr/bin/env bash
set -e

export IFACE="eno1"                 # modify here
export PC_IP="192.168.123.222/24"   # modify here

# Select robot IP based on ROBOT_TYPE (expected: Go2 or Go2W)
export ROBOT_TYPE="${ROBOT_TYPE:-Go2}"
if [[ "$ROBOT_TYPE" == "Go2W" ]]; then
  export ROBOT_IP="192.168.123.161"
elif [[ "$ROBOT_TYPE" == "Go2" ]]; then
  export ROBOT_IP="192.168.123.99"
else
  echo "Unknown ROBOT_TYPE: ${ROBOT_TYPE}. Expected: Go2 or Go2W." >&2
  exit 1
fi

echo "[1/4] Bringing interface up: $IFACE"
sudo ip link set "$IFACE" up

echo "[2/4] Clearing old IP addresses on $IFACE"
sudo ip addr flush dev "$IFACE"

echo "[3/4] Assigning IP $PC_IP to $IFACE"
sudo ip addr add "$PC_IP" dev "$IFACE"

echo "[4/4] Testing connectivity to $ROBOT_TYPE ($ROBOT_IP)"
ping -c 3 "$ROBOT_IP" && \
  echo "✅ Robot reachable!" || \
  echo "❌ Cannot reach robot. Check cable / robot IP."
