#!/usr/bin/env bash
# Put the USB Wi-Fi adapter into monitor mode and start the detector.
# Run by drone-detector.service as root. Settings: /etc/drone-detector.env
#   WIFI_IFACE     capture adapter (default: first wireless interface that is not HOTSPOT_IFACE)
#   HOTSPOT_IFACE  the Pi's onboard Wi-Fi running the ATAK hotspot (default wlan0) — never touched
#   SIMULATE=1     fake detections instead of hardware
set -euo pipefail

cd /opt/drone-detector
source .venv/bin/activate
[ -f /etc/drone-detector.env ] && set -a && . /etc/drone-detector.env && set +a
# shellcheck disable=SC2206
EXTRA=(${DETECTOR_ARGS:-})

if [ "${SIMULATE:-0}" = "1" ]; then
    echo "SIMULATE=1 — starting with simulated detections"
    exec python -m detector --simulate "${EXTRA[@]}" "$@"
fi

# Bluetooth Remote ID uses the onboard Bluetooth; make sure it is not soft-blocked
rfkill unblock bluetooth 2>/dev/null || true

HOTSPOT_IFACE="${HOTSPOT_IFACE:-wlan0}"
IFACE="${WIFI_IFACE:-}"
if [ -z "$IFACE" ]; then
    for dev in /sys/class/net/*; do
        name=$(basename "$dev")
        [ -e "$dev/phy80211" ] || continue       # wireless interfaces only (wlan1, wlx<mac>, ...)
        [ "$name" = "$HOTSPOT_IFACE" ] && continue
        IFACE="$name"
        break
    done
fi

if [ -z "$IFACE" ] || [ ! -e "/sys/class/net/$IFACE" ]; then
    echo "WARNING: no USB Wi-Fi adapter found (besides $HOTSPOT_IFACE) — Wi-Fi detection OFF, Bluetooth Remote ID only"
    exec python -m detector --no-wifi "${EXTRA[@]}" "$@"
fi
echo "Capture adapter: $IFACE"

# NetworkManager would otherwise reconfigure the adapter back to managed mode
if command -v nmcli >/dev/null 2>&1; then
    nmcli device set "$IFACE" managed no 2>/dev/null || true
fi

if ! iw dev "$IFACE" info 2>/dev/null | grep -q "type monitor"; then
    ip link set "$IFACE" down 2>/dev/null || true
    iw dev "$IFACE" set monitor control 2>/dev/null || iw dev "$IFACE" set type monitor 2>/dev/null || true
    ip link set "$IFACE" up 2>/dev/null || true
fi

if ! iw dev "$IFACE" info 2>/dev/null | grep -q "type monitor"; then
    echo "WARNING: $IFACE does not support monitor mode — Wi-Fi detection OFF, Bluetooth Remote ID only"
    exec python -m detector --no-wifi "${EXTRA[@]}" "$@"
fi

echo "Monitor mode enabled on $IFACE"
exec python -m detector --interface "$IFACE" "${EXTRA[@]}" "$@"
