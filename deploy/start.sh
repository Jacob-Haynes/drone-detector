#!/usr/bin/env bash
set -euo pipefail

cd /opt/drone-detector
source .venv/bin/activate

# Find USB WiFi adapter (not wlan0 which is the built-in)
IFACE=""
for dev in /sys/class/net/wlan*; do
    name=$(basename "$dev")
    [ "$name" = "wlan0" ] && continue
    IFACE="$name"
    break
done

if [ -z "$IFACE" ]; then
    echo "No USB WiFi adapter found — starting in simulation mode"
    exec python -m detector --simulate
fi

echo "Found USB WiFi adapter: $IFACE"

# Put interface into monitor mode
ip link set "$IFACE" down 2>/dev/null || true
iw "$IFACE" set monitor control 2>/dev/null || {
    echo "Failed to set monitor mode on $IFACE — starting in simulation mode"
    ip link set "$IFACE" up 2>/dev/null || true
    exec python -m detector --simulate
}
ip link set "$IFACE" up

MON_IFACE="$IFACE"
echo "Monitor mode enabled on $MON_IFACE"

exec python -m detector --interface "$MON_IFACE"