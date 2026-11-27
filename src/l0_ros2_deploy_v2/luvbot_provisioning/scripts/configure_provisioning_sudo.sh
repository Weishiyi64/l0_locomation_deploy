#!/usr/bin/env bash
set -euo pipefail

# Run on the robot to authorize the installed luvbot_provd path; this does not start BLE.
provision_user="${SUDO_USER:-$(id -un)}"
if [[ "$provision_user" == root || ! "$provision_user" =~ ^[a-zA-Z_][a-zA-Z0-9_-]*$ ]]; then
  echo "Run this command as the normal robot user (for example, forlinx)." >&2
  exit 1
fi
provision_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# Match /proc/self/exe resolution in the bridge, including a colcon symlink install.
provision_bridge="$(readlink -f -- "$provision_script_dir/luvbot_bridge")"
provision_binary="$(dirname -- "$provision_bridge")/luvbot_provd"
if [[ ! -x "$provision_binary" || ! "$provision_binary" =~ ^/[a-zA-Z0-9_./+-]+$ ]]; then
  echo "Cannot authorize executable: $provision_binary" >&2
  echo "Build/install luvbot_provisioning and run this script with ros2 run." >&2
  exit 1
fi
provision_rule_tmp="$(mktemp)"
trap 'rm -f -- "$provision_rule_tmp"' EXIT
printf '%s ALL=(root) NOPASSWD: %s\n' \
  "$provision_user" "$provision_binary" > "$provision_rule_tmp"
provision_sudo=()
if [[ "$EUID" -ne 0 ]]; then
  provision_sudo=(sudo)
fi
"${provision_sudo[@]}" visudo -cf "$provision_rule_tmp"
"${provision_sudo[@]}" install -o root -g root -m 0440 \
  "$provision_rule_tmp" "/etc/sudoers.d/99-luvbot-provisioning-$provision_user"
"${provision_sudo[@]}" visudo -c
printf 'Authorized user: %s\nExecutable: %s\n' "$provision_user" "$provision_binary"
echo "Ready. Start the normal ROS launch and publish /luvbot/provision_trigger."
echo "Re-run this command if the installed luvbot_provd path changes."
