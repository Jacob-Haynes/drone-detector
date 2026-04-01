#!/usr/bin/env bash
# Install Drone Detector on the Pi
# Usage: sudo bash deploy/install.sh
set -euo pipefail

APP_DIR="/opt/drone-detector"

echo "=== Drone Detector — Pi Install ==="

# 1. System packages
echo "[1/4] Installing system packages..."
apt-get update -qq
apt-get install -y -qq python3 python3-venv python3-pip iw

# 2. Copy project files
echo "[2/4] Copying project to $APP_DIR..."
mkdir -p "$APP_DIR"
cp -r detector requirements.txt deploy "$APP_DIR/"

# 3. Python venv + deps
echo "[3/4] Setting up Python environment..."
python3 -m venv "$APP_DIR/.venv"
"$APP_DIR/.venv/bin/pip" install --upgrade pip -q
"$APP_DIR/.venv/bin/pip" install -r "$APP_DIR/requirements.txt" -q

# 4. Systemd service
echo "[4/4] Installing systemd service..."
cp "$APP_DIR/deploy/drone-detector.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable drone-detector

# Make start script executable
chmod +x "$APP_DIR/deploy/start.sh"

# Drop pi-profiles configs if pi-profiles is installed
if [ -d "/opt/pi-profiles/profiles.d" ]; then
    echo "Detected pi-profiles — installing profile configs..."
    cp profiles.d/*.conf /opt/pi-profiles/profiles.d/
fi

echo ""
echo "=== Install complete ==="
echo ""
echo "Services:"
echo "  Drone Detector → drone-detector.service (http://<pi-ip>:8080)"
echo ""
echo "Usage:"
echo "  sudo systemctl start drone-detector    # start now"
echo "  journalctl -u drone-detector -f         # view logs"
echo "  sudo pi-profiles drone-detector         # switch to standalone profile"
echo "  sudo pi-profiles bft-drone              # run alongside BFT"