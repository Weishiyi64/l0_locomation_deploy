#!/usr/bin/env bash

# Auto-configure Candlelight udev rules
# This script automatically detects USB device tree and generates appropriate udev rules

set -e  # Exit on any error

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_TEMPLATE_DIR="$(cd "$SCRIPT_DIR/../rules" && pwd)"
UDEV_RULES_PATH="/etc/udev/rules.d/candlelight.rules"
LOG_PREFIX="[CANDLELIGHT_AUTO]"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}${LOG_PREFIX} INFO:${NC} $1" >&2
}

log_success() {
    echo -e "${GREEN}${LOG_PREFIX} SUCCESS:${NC} $1" >&2
}

log_warning() {
    echo -e "${YELLOW}${LOG_PREFIX} WARNING:${NC} $1" >&2
}

log_error() {
    echo -e "${RED}${LOG_PREFIX} ERROR:${NC} $1" >&2
}

# Function to parse USB device tree and extract gs_usb device paths
detect_usb_prefix() {
    log_info "Detecting USB device tree..."

    # Get lsusb -t output
    local usb_tree=$(lsusb -t 2>/dev/null)

    if [[ -z "$usb_tree" ]]; then
        log_error "Failed to get USB device tree"
        return 1
    fi

    # Extract all lines with gs_usb driver
    local gs_usb_lines=$(echo "$usb_tree" | grep "Driver=gs_usb" || true)

    if [[ -z "$gs_usb_lines" ]]; then
        log_error "No gs_usb devices found in USB tree"
        log_error "Please check if Candlelight devices are connected"
        return 1
    fi

    log_info "Found gs_usb devices:"
    echo "$gs_usb_lines" | while read line; do
        log_info "  $line"
    done

   # Parse all gs_usb device paths using mawk/gawk
    local devpaths_str=$(echo "$usb_tree" | awk '
        BEGIN {
            in_bus = 0
            for (k in stack) delete stack[k]
        }
        /^\/:[[:space:]]+Bus/ {
            in_bus = 1
            for (k in stack) delete stack[k]
            next
        }
        in_bus {
            match($0, /^[[:space:]|_]*/)
            indent = RLENGTH
            depth = int(indent / 4)

            if (match($0, /Port[[:space:]]+[0-9]+:/)) {
                port_str = substr($0, RSTART, RLENGTH)
                gsub(/[^0-9]/, "", port_str)
                port = port_str + 0

                stack[depth] = port
                for (i = depth + 1; i <= 10; i++) {
                    delete stack[i]
                }

                if (index($0, "Driver=gs_usb") > 0) {
                    path = ""
                    for (i = 1; i <= depth; i++) {
                        if (stack[i] != "") {
                            if (path != "") path = path "."
                            path = path stack[i]
                        }
                    }
                    if (path != "") print path
                }
            }
        }
    ')

    # Convert to array
    local devpaths=()
    while IFS= read -r path; do
        if [[ -n "$path" ]]; then
            devpaths+=("$path")
        fi
    done <<< "$devpaths_str"

    if [[ ${#devpaths[@]} -eq 0 ]]; then
        log_error "Failed to parse device paths from USB tree"
        return 1
    fi

    log_info "Detected device paths: ${devpaths[*]}"

    # Find longest common prefix
    # According to README: ignore Bus layer Port, find common prefix of all device paths
    # For example: if devices are at 4.3.1, 4.3.7, 4.3.5.1, the prefix is "4.3"
    local prefix=""
    local first_path="${devpaths[0]}"

    # Split first path by dots
    IFS='.' read -ra first_parts <<< "$first_path"

    # Try each possible prefix length, starting from longest
    for ((i=${#first_parts[@]}-1; i>=0; i--)); do
        # Build prefix from first i+1 parts
        local test_prefix=""
        for ((j=0; j<=i; j++)); do
            if [[ $j -eq 0 ]]; then
                test_prefix="${first_parts[j]}"
            else
                test_prefix="$test_prefix.${first_parts[j]}"
            fi
        done

        # Check if all paths start with this prefix
        local all_match=true
        for devpath in "${devpaths[@]}"; do
            if [[ ! "$devpath" =~ ^${test_prefix}(\.|$) ]]; then
                all_match=false
                break
            fi
        done

        # If all match, this is our prefix
        if $all_match; then
            prefix="$test_prefix"
            break
        fi
    done

    if [[ -z "$prefix" ]]; then
        log_error "Failed to determine common USB path prefix"
        log_error "Device paths: ${devpaths[*]}"
        return 1
    fi

    log_success "Detected USB prefix: $prefix"
    echo "$prefix"
}

# Function to generate udev rules from template
generate_rules() {
    local robot_type="$1"
    local usb_prefix="$2"

    local template_file="$RULES_TEMPLATE_DIR/candlelight_${robot_type}.rules"

    if [[ ! -f "$template_file" ]]; then
        log_error "Template file not found: $template_file"
        log_error "Robot type '$robot_type' may not be supported"
        return 1
    fi

    log_info "Loading template: $template_file"
    log_info "USB prefix to substitute: '$usb_prefix'"

    # Read template and replace * with usb_prefix
    # Use a different delimiter for sed to avoid issues with dots and slashes
    local rules_content=$(sed "s|\*|${usb_prefix}|g" "$template_file" 2>&1)
    local sed_exit=$?

    if [[ $sed_exit -ne 0 ]]; then
        log_error "sed command failed with exit code $sed_exit"
        log_error "sed output: $rules_content"
        return 1
    fi

    if [[ -z "$rules_content" ]]; then
        log_error "Failed to generate rules from template"
        return 1
    fi

    log_success "Generated udev rules for robot type: $robot_type"
    echo "$rules_content"
}

# Function to rebind gs_usb devices
rebind_gs_usb_devices() {
    log_info "Rebinding gs_usb devices to apply new udev rules..."

    local gs_usb_driver_path="/sys/bus/usb/drivers/gs_usb"

    if [[ ! -d "$gs_usb_driver_path" ]]; then
        log_warning "gs_usb driver path not found: $gs_usb_driver_path"
        return 0
    fi

    # Get list of bound devices
    local bound_devices=$(ls -1 "$gs_usb_driver_path" 2>/dev/null | grep "^[0-9]" || true)

    if [[ -z "$bound_devices" ]]; then
        log_warning "No gs_usb devices currently bound"
        return 0
    fi

    log_info "Found bound gs_usb devices:"
    echo "$bound_devices" | while read dev; do
        log_info "  - $dev"
    done

    # Unbind and rebind devices
    log_info "Unbinding gs_usb devices..."
    local unbind_count=0
    local bind_count=0

    while IFS= read -r device; do
        if [[ -n "$device" ]]; then
            if echo "$device" | sudo tee "$gs_usb_driver_path/unbind" > /dev/null 2>&1; then
                ((unbind_count++))
            fi
        fi
    done <<< "$bound_devices"

    log_info "Unbound $unbind_count devices"

    # Wait a moment for devices to fully unbind
    sleep 1

    # Rebind all devices
    log_info "Rebinding gs_usb devices..."
    while IFS= read -r device; do
        if [[ -n "$device" ]]; then
            if echo "$device" | sudo tee "$gs_usb_driver_path/bind" > /dev/null 2>&1; then
                ((bind_count++))
            fi
        fi
    done <<< "$bound_devices"

    log_info "Rebound $bind_count devices"

    log_success "Device rebinding completed"
    sleep 2
}

# Function to compare rules content
compare_rules() {
    local new_rules="$1"

    if [[ ! -f "$UDEV_RULES_PATH" ]]; then
        log_info "Rules file does not exist, need to create"
        return 1  # Need to apply
    fi

    local existing_rules=$(sudo cat "$UDEV_RULES_PATH" 2>/dev/null)

    # Normalize both strings by removing trailing whitespace and newlines
    # This handles differences in how echo adds newlines
    local normalized_existing=$(echo -n "$existing_rules" | sed -e 's/[[:space:]]*$//')
    local normalized_new=$(echo -n "$new_rules" | sed -e 's/[[:space:]]*$//')

    if [[ "$normalized_existing" == "$normalized_new" ]]; then
        return 0  # Rules are identical
    else
        return 1  # Rules differ
    fi
}

# Function to apply udev rules
apply_rules() {
    local rules_content="$1"

    log_info "Writing rules to $UDEV_RULES_PATH..."

    # Write rules file
    if ! echo "$rules_content" | sudo tee "$UDEV_RULES_PATH" > /dev/null; then
        log_error "Failed to write udev rules file"
        log_error "Please ensure you have configured passwordless sudo"
        log_error "See README for sudo configuration instructions"
        return 1
    fi

    log_success "Rules file written successfully"

    # Reload udev rules
    log_info "Reloading udev rules..."

    if ! sudo udevadm control --reload-rules; then
        log_error "Failed to reload udev rules"
        return 1
    fi

    if ! sudo systemctl restart systemd-udevd; then
        log_error "Failed to restart systemd-udevd"
        return 1
    fi

    log_success "Udev rules reloaded successfully"

    # Rebind devices to apply new rules
    rebind_gs_usb_devices

    log_info "Waiting for devices to be re-enumerated..."
    sleep 2
}

# Function to verify rules are applied
verify_rules() {
    log_info "Verifying candlelight devices..."

    # Check if expected CAN interfaces exist
    local can_interfaces=$(ip link show | grep -o "can_[a-z_0-9]*" | sort -u || true)

    if [[ -z "$can_interfaces" ]]; then
        log_warning "No CAN interfaces found yet"
        log_warning "You may need to unplug and replug the USB cable"
        return 0
    fi

    log_success "Found CAN interfaces:"
    echo "$can_interfaces" | while read iface; do
        log_info "  - $iface"
    done
}

# Main function
main() {
    local robot_type=""
    local show_help=false

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --robot-type)
                if [[ -z "${2:-}" ]]; then
                    log_error "Option --robot-type requires an argument"
                    exit 1
                fi
                robot_type="$2"
                shift 2
                ;;
            --help|-h)
                show_help=true
                shift
                ;;
            *)
                log_error "Unknown option: $1"
                log_error "Use --help for usage information"
                exit 1
                ;;
        esac
    done

    # Show help
    if [[ "$show_help" == true ]]; then
        echo "Usage: $0 --robot-type ROBOT_TYPE"
        echo ""
        echo "Auto-configure Candlelight udev rules based on USB device tree"
        echo ""
        echo "Options:"
        echo "  --robot-type TYPE    Robot type (e.g., y1_1, atom_v3_smallfeet)"
        echo "  --help, -h           Show this help message"
        echo ""
        echo "This script automatically:"
        echo "  1. Detects USB device tree and finds gs_usb devices"
        echo "  2. Determines the common USB path prefix"
        echo "  3. Generates udev rules from template"
        echo "  4. Applies the rules to the system"
        exit 0
    fi

    # Validate robot type
    if [[ -z "$robot_type" ]]; then
        log_error "Robot type not specified"
        log_error "Use --robot-type option to specify robot type"
        exit 1
    fi

    log_info "Auto-configuring Candlelight for robot type: $robot_type"
    echo "========================================================"

    # Step 1: Detect USB prefix
    local usb_prefix=$(detect_usb_prefix)
    if [[ $? -ne 0 ]] || [[ -z "$usb_prefix" ]]; then
        log_error "Failed to detect USB prefix"
        exit 1
    fi

    echo ""

    # Step 2: Generate rules
    log_info "Generating udev rules..."
    local rules_content=$(generate_rules "$robot_type" "$usb_prefix")
    if [[ $? -ne 0 ]] || [[ -z "$rules_content" ]]; then
        log_error "Failed to generate udev rules"
        exit 1
    fi

    echo ""

    # Step 3: Compare and conditionally apply rules
    if compare_rules "$rules_content"; then
        log_success "Rules are already up-to-date, skipping rewrite and rebind"
        log_info "USB prefix: $usb_prefix (unchanged)"
    else
        log_info "Rules differ or don't exist, applying new rules..."
        if ! apply_rules "$rules_content"; then
            log_error "Failed to apply udev rules"
            exit 1
        fi
    fi

    echo ""

    # Step 4: Verify
    verify_rules

    echo ""
    echo "========================================================"
    log_success "Candlelight auto-configuration completed!"
    log_info "USB prefix: $usb_prefix"
    log_info "Robot type: $robot_type"
    log_info "Rules file: $UDEV_RULES_PATH"
}

# Run main function
main "$@"
