#!/usr/bin/env bash
set -e

export IFACE="eno1"                 # modify here
export PC_IP="192.168.123.222/24"   # modify here

GO2_IP="192.168.123.161"  # Default IP

echo "[1/4] Bringing interface up: $IFACE"
sudo ip link set "$IFACE" up

echo "[2/4] Clearing old IP addresses on $IFACE"
sudo ip addr flush dev "$IFACE"

echo "[3/4] Assigning IP $PC_IP to $IFACE"
sudo ip addr add "$PC_IP" dev "$IFACE"

echo "[4/4] Testing connectivity to Go2 ($GO2_IP)"
ping -c 3 "$GO2_IP" && \
  echo "✅ Go2 reachable!" || \
  echo "❌ Cannot reach Go2. Check cable / Go2 IP."

