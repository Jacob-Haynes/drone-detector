#!/usr/bin/env bash
# Install Drone Detector on the Pi. Idempotent and non-interactive.
# Does NOT enable the service at boot — pi-profiles starts it.
# Usage: sudo bash deploy/install.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
APP_DIR="/opt/drone-detector"
DATA_DIR="/var/lib/drone-detector"
ENV_FILE="/etc/drone-detector.env"

echo "=== Drone Detector — Pi Install ==="

echo "[1/5] Installing system packages..."
apt-get update -qq
apt-get install -y -qq python3 python3-venv python3-pip iw rfkill bluez

echo "[2/5] Copying project to $APP_DIR..."
mkdir -p "$APP_DIR" "$DATA_DIR"
if [ "$REPO_DIR" != "$APP_DIR" ]; then
    rm -rf "$APP_DIR/detector"
    cp -r "$REPO_DIR/detector" "$REPO_DIR/requirements.txt" "$REPO_DIR/deploy" "$REPO_DIR/profiles.d" "$APP_DIR/"
fi
chmod +x "$APP_DIR/deploy/start.sh"

echo "[3/5] Setting up Python environment..."
[ -x "$APP_DIR/.venv/bin/python" ] || python3 -m venv "$APP_DIR/.venv"
"$APP_DIR/.venv/bin/pip" install --upgrade pip -q
"$APP_DIR/.venv/bin/pip" install -r "$APP_DIR/requirements.txt" -q

echo "[4/5] Config..."
if [ ! -f "$ENV_FILE" ]; then
    install -m 600 "$REPO_DIR/deploy/drone-detector.env.example" "$ENV_FILE"
    echo "Created $ENV_FILE"
elif grep -qE '^WEB_PORT=8080\s*$' "$ENV_FILE"; then
    # 8080 is OpenTAKServer's nginx; the kit moved the detector to 8060
    sed -i 's/^WEB_PORT=8080\s*$/WEB_PORT=8060/' "$ENV_FILE"
    echo "Moved WEB_PORT 8080 -> 8060 in $ENV_FILE (8080 is OpenTAKServer's nginx)"
fi

echo "[5/5] Installing systemd service (not enabled at boot — pi-profiles starts it)..."
cp "$REPO_DIR/deploy/drone-detector.service" /etc/systemd/system/
systemctl daemon-reload

if [ -d "/opt/pi-profiles/profiles.d" ]; then
    cp "$REPO_DIR/profiles.d/"*.conf /opt/pi-profiles/profiles.d/
    echo "Installed pi-profiles profile: drone-detector"
fi

PORT=$(grep -E '^WEB_PORT=' "$ENV_FILE" | cut -d= -f2)
echo ""
echo "=== Install complete ==="
echo "  Web UI:  http://$(hostname -s).local:${PORT:-8060}/"
echo "  Start:   sudo pi-profiles drone-detector   (or: sudo systemctl start drone-detector)"
echo "  Logs:    journalctl -u drone-detector -f"
if [ "${PORT:-8060}" = "8080" ]; then
    echo ""
    echo "  WARNING: WEB_PORT=8080 clashes with OpenTAKServer's nginx. Use 8060 (set in $ENV_FILE)."
fi
