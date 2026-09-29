#!/bin/sh
# Install the camera relay as a launchd daemon: a root-owned copy of camrelay/ in /usr/local/lib/camrelay
# with its own environment (nothing of it writable by a user, since it runs as root), started at boot,
# restarted whenever it stops, serving the cameras on 127.0.0.1:8820 and keeping 192.168.10.200 on en14.
# Run from the repo, once (and again after changing camrelay/):
#   sudo camrelay/install.sh
#   sudo camrelay/install.sh --remove     # stop it and take it away
# Its log: /Library/Logs/camrelay.log
set -eu
LABEL=com.myaloha.camrelay
DEST=/usr/local/lib/camrelay
PLIST=/Library/LaunchDaemons/$LABEL.plist
LOG=/Library/Logs/camrelay.log
SRC=$(cd "$(dirname "$0")" && pwd)
[ "$(id -u)" = 0 ] || { echo "camrelay/install.sh: run it with sudo" >&2; exit 1; }

launchctl bootout "system/$LABEL" 2>/dev/null || true
if [ "${1:-}" = --remove ]; then
    rm -rf "$DEST" "$PLIST"
    echo "camrelay removed"
    exit 0
fi

UV=""
for u in /opt/homebrew/bin/uv /usr/local/bin/uv "/Users/${SUDO_USER:-}/.local/bin/uv" "/Users/${SUDO_USER:-}/.cargo/bin/uv"; do
    [ -x "$u" ] && { UV=$u; break; }
done
[ -n "$UV" ] || { echo "camrelay/install.sh: uv not found" >&2; exit 1; }

rm -rf "$DEST"
mkdir -p "$DEST"
cp -R "$SRC/pyproject.toml" "$SRC/uv.lock" "$SRC/src" "$DEST/"
UV_CACHE_DIR="$DEST/.uv-cache" UV_PYTHON_INSTALL_DIR="$DEST/.python" "$UV" sync --project "$DEST" --frozen --no-dev -q
rm -rf "$DEST/.uv-cache"
chown -R root:wheel "$DEST"
chmod -R go-w "$DEST"

cat > "$PLIST" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>Label</key><string>$LABEL</string>
  <key>ProgramArguments</key>
  <array>
    <string>$DEST/.venv/bin/python</string><string>-B</string><string>-m</string><string>camrelay</string>
    <string>--listen</string><string>127.0.0.1:8820</string>
    <string>--iface</string><string>en14</string><string>--host-ip</string><string>192.168.10.200</string>
  </array>
  <key>WorkingDirectory</key><string>$DEST</string>
  <key>RunAtLoad</key><true/>
  <key>KeepAlive</key><true/>
  <key>ThrottleInterval</key><integer>3</integer>
  <key>StandardOutPath</key><string>$LOG</string>
  <key>StandardErrorPath</key><string>$LOG</string>
</dict>
</plist>
PLIST
chown root:wheel "$PLIST"
chmod 644 "$PLIST"
launchctl bootstrap system "$PLIST"
sleep 2
echo "camrelay installed: serving on 127.0.0.1:8820; log $LOG"
tail -3 "$LOG" 2>/dev/null || true
