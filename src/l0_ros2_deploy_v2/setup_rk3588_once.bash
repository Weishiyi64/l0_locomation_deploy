#!/usr/bin/env bash

# One-time RK3588 setup for Luvbot real-hardware deployment.
# Run from the source tree before launching the robot:
#   sudo bash setup_rk3588_once.bash --user forlinx

set -euo pipefail

TARGET_USER=""

usage() {
    cat <<'EOF'
Usage:
  sudo bash setup_rk3588_once.bash [--user USER]

Options:
  --user USER  Linux user to configure. Defaults to the sudo caller, or forlinx.
  -h, --help   Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --user)
            if [[ -z "${2:-}" ]]; then
                echo "ERROR: --user requires a value" >&2
                exit 1
            fi
            TARGET_USER="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: Unknown argument: $1" >&2
            usage
            exit 1
            ;;
    esac
done

if [[ "$EUID" -ne 0 ]]; then
    echo "ERROR: Run as root: sudo bash setup_rk3588_once.bash --user forlinx" >&2
    exit 1
fi

if [[ -z "$TARGET_USER" ]]; then
    if [[ -n "${SUDO_USER:-}" && "$SUDO_USER" != "root" ]]; then
        TARGET_USER="$SUDO_USER"
    elif id forlinx >/dev/null 2>&1; then
        TARGET_USER="forlinx"
    else
        echo "ERROR: Could not infer target user. Re-run with --user USER." >&2
        exit 1
    fi
fi

if ! id "$TARGET_USER" >/dev/null 2>&1; then
    echo "ERROR: User '$TARGET_USER' does not exist." >&2
    exit 1
fi

if [[ "$(uname -m)" != "aarch64" ]]; then
    echo "ERROR: This setup is only for an aarch64 RK3588 device." >&2
    exit 1
fi

if [[ -r /proc/device-tree/compatible ]] && \
   ! tr '\0' '\n' < /proc/device-tree/compatible | grep -qi 'rk3588'; then
    echo "WARNING: /proc/device-tree/compatible does not identify an RK3588." >&2
fi

require_cmd() {
    local name="$1"
    local path
    path="$(command -v "$name" || true)"
    if [[ -z "$path" ]]; then
        echo "ERROR: Required command not found: $name" >&2
        exit 1
    fi
    echo "$path"
}

UDEVADM_BIN="$(require_cmd udevadm)"
SYSTEMCTL_BIN="$(require_cmd systemctl)"
USERMOD_BIN="$(require_cmd usermod)"
GETENT_BIN="$(require_cmd getent)"
GROUPADD_BIN="$(command -v groupadd || true)"

echo "Configuring RK3588 device for user: $TARGET_USER"

# 1. Stable device name retained for the real-hardware IMU path.
IMU_RULES_FILE="/etc/udev/rules.d/99-luvbot-yesense.rules"
YESENSE_RULE='KERNEL=="ttyACM*", ATTRS{idVendor}=="5953", ATTRS{idProduct}=="5543", MODE:="0777", SYMLINK+="yesense_imu"'
printf '%s\n' "$YESENSE_RULE" > "$IMU_RULES_FILE"
chmod 0644 "$IMU_RULES_FILE"
echo "OK: Installed Yesense IMU rule $IMU_RULES_FILE"

# 2. Realtime scheduling/memory-lock limits used by the controller.
LIMITS_FILE="/etc/security/limits.d/99-luvbot-rt.conf"
cat > "$LIMITS_FILE" <<EOF
# Managed by setup_rk3588_once.bash
$TARGET_USER soft rtprio 99
$TARGET_USER hard rtprio 99
$TARGET_USER soft memlock unlimited
$TARGET_USER hard memlock unlimited
EOF
chmod 0644 "$LIMITS_FILE"
echo "OK: Installed realtime limits file $LIMITS_FILE"

# 3. Serial-port access for the Yesense IMU.
if ! "$GETENT_BIN" group dialout >/dev/null; then
    if [[ -z "$GROUPADD_BIN" ]]; then
        echo "ERROR: dialout group does not exist and groupadd was not found." >&2
        exit 1
    fi
    "$GROUPADD_BIN" dialout
fi
"$USERMOD_BIN" -a -G dialout "$TARGET_USER"
echo "OK: Added $TARGET_USER to dialout group"

# 4. Apply the rules now. Replugging USB devices or rebooting is still safest.
"$UDEVADM_BIN" control --reload-rules
if ! "$SYSTEMCTL_BIN" restart systemd-udevd; then
    echo "WARNING: systemd-udevd restart failed; rules were still reloaded." >&2
fi
if ! "$UDEVADM_BIN" trigger; then
    echo "WARNING: udev trigger failed; reboot or replug the USB devices." >&2
fi

echo
echo "Done. Reboot the RK3588 before the first real-hardware launch:"
echo "  sudo reboot"
