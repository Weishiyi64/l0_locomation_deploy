#!/usr/bin/env bash

# Smart CAN Interface Setup Script
# This script intelligently manages CAN interface configuration
# It can detect interface states and avoid redundant operations

set -e  # Exit on any error

# Configuration
BITRATE="1000000"
BATTERY_BITRATE="250000"
RESTART_MS="100"
TXQUEUELEN="5000"
ENABLE_BATTERY="${ENABLE_BATTERY:-true}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_PREFIX="[CAN_SETUP]"

# CAN interfaces to manage (will be loaded from rules file based on robot type)
CAN_INTERFACES=()

# Battery CAN interface (separate bitrate)
BATTERY_INTERFACE="can_battery"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}${LOG_PREFIX} INFO:${NC} $1"
}

log_success() {
    echo -e "${GREEN}${LOG_PREFIX} SUCCESS:${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}${LOG_PREFIX} WARNING:${NC} $1"
}

log_error() {
    echo -e "${RED}${LOG_PREFIX} ERROR:${NC} $1"
}

# Function to parse CAN devices from udev rules file
parse_can_devices_from_rules() {
    local rules_file="$1"

    if [[ ! -f "$rules_file" ]]; then
        log_error "Rules file not found: $rules_file"
        return 1
    fi

    # Extract NAME values from rules file, excluding can_battery
    # Format: NAME="can_interface_name"
    # Note: can_battery is handled separately via BATTERY_INTERFACE and ENABLE_BATTERY
    local can_devices=($(grep -v '^[[:space:]]*#' "$rules_file" | grep -oP 'NAME="\K[a-zA-Z0-9_]+' | grep -v '^can_battery$' | sort -u))

    if [[ ${#can_devices[@]} -eq 0 ]]; then
        log_error "No CAN devices found in $rules_file (excluding can_battery)"
        return 1
    fi

    echo "${can_devices[@]}"
}

# Function to parse CAN devices from YAML configuration file
parse_can_devices_from_yaml() {
    local yaml_file="$1"

    if [[ ! -f "$yaml_file" ]]; then
        log_error "YAML file not found: $yaml_file"
        return 1
    fi

    # Extract can_device values using grep, sort and remove duplicates
    local can_devices=($(grep -oP 'can_device:\s*\K[a-zA-Z0-9_]+' "$yaml_file" | sort -u))

    if [[ ${#can_devices[@]} -eq 0 ]]; then
        log_error "No CAN devices found in $yaml_file"
        return 1
    fi

    echo "${can_devices[@]}"
}

# Function to load CAN interfaces for a specific robot type from rules file
load_can_interfaces_for_robot() {
    local robot_type="$1"

    if [[ -z "$robot_type" ]]; then
        log_error "Robot type not specified" >&2
        return 1
    fi

    # Locate the rules file in the same package
    # Rules file naming convention: candlelight_${robot_type}.rules
    local rules_file="${SCRIPT_DIR}/../rules/candlelight_${robot_type}.rules"

    if [[ ! -f "$rules_file" ]]; then
        log_error "Rules file not found for robot type '$robot_type'" >&2
        log_error "Expected path: $rules_file" >&2
        return 1
    fi

    log_info "Loading CAN configuration from: $rules_file" >&2

    # Parse CAN devices from the rules file
    local devices=($(parse_can_devices_from_rules "$rules_file" 2>&1))

    if [[ $? -ne 0 || ${#devices[@]} -eq 0 ]]; then
        log_error "Failed to parse CAN devices from $rules_file" >&2
        return 1
    fi

    log_info "Found ${#devices[@]} unique CAN interfaces: ${devices[*]}" >&2
    echo "${devices[@]}"
}

# Cache for interface existence (populated once at startup)
declare -A INTERFACE_EXISTS_CACHE

# Function to cache interface existence for all interfaces
cache_interface_existence() {
    for iface in "${CAN_INTERFACES[@]}"; do
        if ip link show "$iface" &>/dev/null; then
            INTERFACE_EXISTS_CACHE["$iface"]="true"
        else
            INTERFACE_EXISTS_CACHE["$iface"]="false"
        fi
    done

    # Also cache battery interface if enabled
    if [[ "$ENABLE_BATTERY" == "true" ]]; then
        if ip link show "$BATTERY_INTERFACE" &>/dev/null; then
            INTERFACE_EXISTS_CACHE["$BATTERY_INTERFACE"]="true"
        else
            INTERFACE_EXISTS_CACHE["$BATTERY_INTERFACE"]="false"
        fi
    fi
}

# Function to check if a CAN interface exists (uses cache)
interface_exists() {
    local iface="$1"
    # Use cache if available
    if [[ -n "${INTERFACE_EXISTS_CACHE[$iface]}" ]]; then
        [[ "${INTERFACE_EXISTS_CACHE[$iface]}" == "true" ]]
    else
        # Fallback to direct check if not in cache
        ip link show "$iface" &>/dev/null
    fi
}

# Function to get interface full state in one call (optimized)
get_interface_full_state() {
    local iface="$1"
    if ! interface_exists "$iface"; then
        echo "NOT_FOUND|NOT_FOUND|false"
        return
    fi

    # One call to get all information
    local details=$(ip -details link show "$iface" 2>/dev/null)
    local state=$(echo "$details" | grep -oP 'state \K\w+' | head -1 | tr -d '\n' || echo "UNKNOWN")
    local can_state=$(echo "$details" | grep -oP 'can state \K\w+' | head -1 | tr -d '\n' || echo "UNKNOWN")

    # Determine if ready
    local is_ready="false"
    if [[ "$state" == "UP" && ("$can_state" == "ERROR-ACTIVE" || "$can_state" == "ERROR-PASSIVE" || "$can_state" == "ERROR") ]]; then
        is_ready="true"
    fi

    echo "$state|$can_state|$is_ready"
}

# Function to get CAN interface state (kept for compatibility, but uses full_state)
get_interface_state() {
    local iface="$1"
    local full_state=$(get_interface_full_state "$iface")
    echo "$full_state" | cut -d'|' -f1
}

# Function to get CAN interface operational state (kept for compatibility, but uses full_state)
get_can_state() {
    local iface="$1"
    local full_state=$(get_interface_full_state "$iface")
    echo "$full_state" | cut -d'|' -f2
}

# Function to check if interface is properly configured (optimized)
is_interface_ready() {
    local iface="$1"
    local full_state=$(get_interface_full_state "$iface")
    local is_ready=$(echo "$full_state" | cut -d'|' -f3)

    [[ "$is_ready" == "true" ]]
}

# Function to configure a single CAN interface
configure_interface() {
    local iface="$1"
    local bitrate="$2"
    local restart_ms="${3:-$RESTART_MS}"
    local txqueuelen="${4:-$TXQUEUELEN}"

    if ! interface_exists "$iface"; then
        log_error "Interface $iface not found. Check hardware connection."
        return 1
    fi

    # Check current state (optimized - single call)
    local full_state=$(get_interface_full_state "$iface")
    local current_state=$(echo "$full_state" | cut -d'|' -f1)
    local current_can_state=$(echo "$full_state" | cut -d'|' -f2)
    local is_ready=$(echo "$full_state" | cut -d'|' -f3)

    log_info "Configuring $iface (current: $current_state/$current_can_state)"

    # If interface is already properly configured, skip immediately (no sleep)
    if [[ "$is_ready" == "true" ]]; then
        log_success "$iface is already UP and ready"
        return 0
    fi

    # If interface is UP but not in the right CAN state, bring it down first
    if [[ "$current_state" == "UP" ]]; then
        log_info "Bringing down $iface to reconfigure..."
        if ! sudo ip link set "$iface" down; then
            log_error "Failed to bring down $iface"
            return 1
        fi
    fi

    # Configure the interface with bitrate
    if ! sudo ip link set "$iface" type can bitrate "$bitrate"; then
        log_error "Failed to set bitrate for $iface"
        return 1
    fi

    # Set restart-ms for automatic restart
    if ! sudo ip link set "$iface" type can restart-ms "$restart_ms"; then
        log_warning "Failed to set restart-ms for $iface, continuing anyway..."
    fi

    # Set transmit queue length to prevent buffer overflow (ENOBUFS error 105)
    if ! sudo ip link set "$iface" txqueuelen "$txqueuelen"; then
        log_warning "Failed to set txqueuelen for $iface, continuing anyway..."
    fi

    # Bring up the interface
    if sudo ip link set "$iface" up; then
        # Wait a moment for the interface to stabilize
        sleep 0.1
        log_info "$iface mount attempted (bitrate: $bitrate, restart-ms: $restart_ms, txqueuelen: $txqueuelen)"
        return 0
    else
        log_error "Failed to bring up $iface"
        return 1
    fi
}

# Function to check USB devices for CANable devices
check_usb_devices() {
    log_info "Checking for CANable USB devices..."
    local canables=$(lsusb | grep -i "candlelight\|gs_usb\|1d50:606f" | wc -l)
    if [[ $canables -gt 0 ]]; then
        log_success "Found $canables CANable device(s)"
        lsusb | grep -i "candlelight\|gs_usb\|1d50:606f" | while read line; do
            log_info "  - $line"
        done
    else
        log_warning "No CANable devices found via lsusb"
    fi
}

# Function to mount interfaces from a list (without immediate verification)
mount_interfaces_batch() {
    local interfaces_to_mount=("$@")
    local bitrate="$BITRATE"

    log_info "Mounting ${#interfaces_to_mount[@]} interfaces..."

    for iface in "${interfaces_to_mount[@]}"; do
        if interface_exists "$iface"; then
            configure_interface "$iface" "$bitrate" "$RESTART_MS" "$TXQUEUELEN"
            # Sleep after each mount attempt as required
            sleep 0.1
        else
            log_warning "Interface $iface not found - skipping"
        fi
    done
}

# Function to check which interfaces failed to mount
get_failed_interfaces() {
    local failed_interfaces=()

    for iface in "${CAN_INTERFACES[@]}"; do
        if interface_exists "$iface" && ! is_interface_ready "$iface"; then
            failed_interfaces+=("$iface")
        fi
    done

    echo "${failed_interfaces[@]}"
}

# Function to display interface summary
show_interface_summary() {
    log_info "CAN Interface Summary:"
    echo "--------------------------------------------------------"
    printf "%-12s %-8s %-12s %-10s\n" "Interface" "Link" "CAN State" "Status"
    echo "--------------------------------------------------------"

    local total_ready=0
    local total_found=0

    # Check motor CAN interfaces (optimized - single call per interface)
    for iface in "${CAN_INTERFACES[@]}"; do
        if interface_exists "$iface"; then
            total_found=$((total_found + 1))
            local full_state=$(get_interface_full_state "$iface")
            local state=$(echo "$full_state" | cut -d'|' -f1)
            local can_state=$(echo "$full_state" | cut -d'|' -f2)
            local is_ready=$(echo "$full_state" | cut -d'|' -f3)
            local status="❌ Not Ready"

            if [[ "$is_ready" == "true" ]]; then
                status="✅ Ready"
                total_ready=$((total_ready + 1))
            elif [[ "$state" == "UP" ]]; then
                status="⚠️ Partial"
            fi

            printf "%-12s %-8s %-12s %s\n" "$iface" "$state" "$can_state" "$status"
        else
            printf "%-12s %-8s %-12s %s\n" "$iface" "N/A" "N/A" "❌ Missing"
        fi
    done

    # Check battery CAN interface (optimized - single call)
    if [[ "$ENABLE_BATTERY" == "true" ]]; then
        if interface_exists "$BATTERY_INTERFACE"; then
            total_found=$((total_found + 1))
            local full_state=$(get_interface_full_state "$BATTERY_INTERFACE")
            local state=$(echo "$full_state" | cut -d'|' -f1)
            local can_state=$(echo "$full_state" | cut -d'|' -f2)
            local is_ready=$(echo "$full_state" | cut -d'|' -f3)
            local status="❌ Not Ready"

            if [[ "$is_ready" == "true" ]]; then
                status="✅ Ready"
                total_ready=$((total_ready + 1))
            elif [[ "$state" == "UP" ]]; then
                status="⚠️ Partial"
            fi

            printf "%-12s %-8s %-12s %s\n" "$BATTERY_INTERFACE" "$state" "$can_state" "$status"
        else
            printf "%-12s %-8s %-12s %s\n" "$BATTERY_INTERFACE" "N/A" "N/A" "❌ Missing"
        fi
    fi

    echo "--------------------------------------------------------"
    local expected_interfaces_num=${#CAN_INTERFACES[@]}
    if [[ "$ENABLE_BATTERY" == "true" ]]; then
        expected_interfaces_num=$((expected_interfaces_num + 1))  # +1 for battery
    fi
    if [[ $total_ready -eq $total_found && $total_found -eq $expected_interfaces_num ]]; then
        log_info "Summary: All interfaces ready"
    else
        log_info "Summary: $total_ready/$total_found interfaces ready"
    fi
}

# Main execution with proper mounting logic
main() {
    log_info "Smart CAN Interface Setup (bitrate: $BITRATE bps)"
    if [[ "$ENABLE_BATTERY" == "true" ]]; then
        log_info "Battery interface: ENABLED"
    else
        log_info "Battery interface: DISABLED"
    fi
    echo "========================================================"

    # Cache interface existence once at startup
    cache_interface_existence

    # Show initial state
    log_info "Initial interface states:"
    show_interface_summary
    echo

    # Check if all required interfaces exist
    local missing_interfaces=()
    local available_interfaces=()

    for iface in "${CAN_INTERFACES[@]}"; do
        if interface_exists "$iface"; then
            available_interfaces+=("$iface")
        else
            missing_interfaces+=("$iface")
        fi
    done

    # Report missing motor interfaces
    if [[ ${#missing_interfaces[@]} -gt 0 ]]; then
        log_error "Missing ${#missing_interfaces[@]} required motor interfaces: ${missing_interfaces[*]}"
        log_error "Expected ${#CAN_INTERFACES[@]} motor interfaces but found only ${#available_interfaces[@]}"
        echo
        log_info "Checking USB devices for diagnostics..."
        check_usb_devices
        exit 1
    fi

    log_info "Found all ${#available_interfaces[@]} required motor interfaces: ${available_interfaces[*]}"

    # Check battery interface if enabled
    if [[ "$ENABLE_BATTERY" == "true" ]]; then
        if ! interface_exists "$BATTERY_INTERFACE"; then
            log_error "Battery interface $BATTERY_INTERFACE is required but not found"
            log_error "Either connect the battery interface or disable it with ENABLE_BATTERY=false"
            echo
            log_info "Checking USB devices for diagnostics..."
            check_usb_devices
            exit 1
        fi
        log_info "Battery interface $BATTERY_INTERFACE found"
    fi

    # Check if all interfaces are already ready (optimization)
    log_info "Checking if all interfaces are already configured..."
    local all_ready=true
    local interfaces_need_config=()

    for iface in "${available_interfaces[@]}"; do
        if ! is_interface_ready "$iface"; then
            all_ready=false
            interfaces_need_config+=("$iface")
        fi
    done

    local battery_needs_config=false
    if [[ "$ENABLE_BATTERY" == "true" ]]; then
        if ! is_interface_ready "$BATTERY_INTERFACE"; then
            all_ready=false
            battery_needs_config=true
        fi
    fi

    if [[ "$all_ready" == true ]]; then
        echo
        log_success "All interfaces are already configured and ready!"
        log_info "Skipping mounting cycle - no configuration needed"
        echo "========================================================"
        exit 0
    fi

    # Some interfaces need configuration
    if [[ ${#interfaces_need_config[@]} -gt 0 ]]; then
        log_info "Motor interfaces needing configuration: ${interfaces_need_config[*]}"
    fi
    if [[ "$battery_needs_config" == true ]]; then
        log_info "Battery interface needs configuration: $BATTERY_INTERFACE"
    fi
    echo

    # Start mounting cycle - initially try to mount all available interfaces
    local interfaces_to_mount=("${available_interfaces[@]}")
    local cycle_count=0
    local max_cycles=10

    # Track if battery needs retry
    local retry_battery=false
    if [[ "$ENABLE_BATTERY" == "true" ]]; then
        retry_battery=true
    fi

    while [[ $cycle_count -lt $max_cycles ]]; do
        cycle_count=$((cycle_count + 1))
        log_info "Mounting cycle $cycle_count/$max_cycles..."

        # Step 1: Mount motor interfaces in the list
        mount_interfaces_batch "${interfaces_to_mount[@]}"

        # Step 1b: Also mount battery if needed
        if [[ "$retry_battery" == true ]]; then
            log_info "Mounting battery interface $BATTERY_INTERFACE..."
            configure_interface "$BATTERY_INTERFACE" "$BATTERY_BITRATE" "$RESTART_MS" "$TXQUEUELEN"
            sleep 0.1
        fi

        # Step 2: Sleep after all mounting attempts
        log_info "Waiting 1s for interfaces to stabilize..."
        sleep 1

        # Step 3: Check which interfaces failed to mount
        local failed_interfaces=($(get_failed_interfaces))

        # Also check battery interface
        local battery_failed=false
        if [[ "$ENABLE_BATTERY" == "true" ]]; then
            if ! is_interface_ready "$BATTERY_INTERFACE"; then
                battery_failed=true
            fi
        fi

        # Show current status
        show_interface_summary
        echo

        # Check success conditions: all motor interfaces + battery (if enabled)
        local all_success=false

        if [[ ${#failed_interfaces[@]} -eq 0 ]]; then
            # All motor interfaces are ready
            if [[ "$ENABLE_BATTERY" == "true" ]]; then
                # Battery is required - check it too
                if [[ "$battery_failed" == false ]]; then
                    # Both motor and battery interfaces are ready
                    log_success "All interfaces (motor + battery) mounted successfully after $cycle_count cycles!"
                    all_success=true
                else
                    # Motor interfaces ready, but battery failed
                    log_warning "All motor interfaces ready, but battery interface failed"
                fi
            else
                # Battery not required - only motor interfaces matter
                log_success "All motor interfaces mounted successfully after $cycle_count cycles!"
                all_success=true
            fi
        else
            # Some motor interfaces failed
            log_warning "Cycle $cycle_count: ${#failed_interfaces[@]} motor interfaces still not ready: ${failed_interfaces[*]}"
        fi

        # Exit if all required interfaces are ready
        if [[ "$all_success" == true ]]; then
            exit 0
        fi

        # Prepare for next cycle
        if [[ $cycle_count -lt $max_cycles ]]; then
            # Retry failed motor interfaces
            interfaces_to_mount=("${failed_interfaces[@]}")

            # Set battery retry flag
            retry_battery="$battery_failed"

            # Log what will be retried
            if [[ ${#interfaces_to_mount[@]} -gt 0 ]]; then
                log_info "Will retry ${#interfaces_to_mount[@]} failed motor interfaces in next cycle: ${interfaces_to_mount[*]}"
            fi
            if [[ "$retry_battery" == true ]]; then
                log_info "Will retry battery interface: $BATTERY_INTERFACE"
            fi

            log_info "Waiting 2 seconds before next mounting cycle..."
            sleep 2
        else
            # Max cycles reached - report all failures
            local error_msg="Maximum cycles ($max_cycles) reached."
            if [[ ${#failed_interfaces[@]} -gt 0 ]]; then
                error_msg="$error_msg ${#failed_interfaces[@]} motor interfaces failed: ${failed_interfaces[*]}."
            fi
            if [[ "$battery_failed" == true ]]; then
                error_msg="$error_msg Battery interface failed: $BATTERY_INTERFACE."
            fi
            log_error "$error_msg"
            exit 1
        fi
    done
}

# Parse command line arguments
ROBOT_TYPE=""
SHOW_STATUS=false
SHOW_HELP=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --robot-type)
            if [[ -z "${2:-}" ]]; then
                log_error "Option --robot-type requires an argument"
                exit 1
            fi
            ROBOT_TYPE="$2"
            shift 2
            ;;
        --status|-s)
            SHOW_STATUS=true
            shift
            ;;
        --help|-h)
            SHOW_HELP=true
            shift
            ;;
        *)
            log_error "Unknown option: $1"
            log_error "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Handle help request
if [[ "$SHOW_HELP" == true ]]; then
    echo "Usage: $0 --robot-type TYPE [OPTIONS]"
    echo ""
    echo "Required:"
    echo "  --robot-type TYPE        Specify robot type (required)"
    echo ""
    echo "Options:"
    echo "  --status, -s             Show current CAN interface status"
    echo "  --help, -h               Show this help message"
    echo ""
    echo "Configuration:"
    echo "  Motor CAN bitrate: 1000000 bps (fixed)"
    echo "  Battery CAN bitrate: 250000 bps (fixed)"
    echo "  Battery interface: Controlled by ENABLE_BATTERY environment variable (default: true)"
    echo ""
    echo "This script intelligently configures CAN interfaces, avoiding"
    echo "redundant operations and providing detailed status information."
    echo ""
    echo "The script automatically loads the CAN interface list from the"
    echo "udev rules file: rules/candlelight_\${robot_type}.rules"
    exit 0
fi

# Handle status request
if [[ "$SHOW_STATUS" == true ]]; then
    log_info "Current CAN interface status:"
    show_interface_summary
    exit 0
fi

# Load CAN interfaces based on robot type (required)
if [[ -z "$ROBOT_TYPE" ]]; then
    log_error "Robot type is required. Please specify with --robot-type option"
    log_error "Use --help for usage information"
    exit 1
fi

log_info "Robot type specified: $ROBOT_TYPE"
CAN_INTERFACES=($(load_can_interfaces_for_robot "$ROBOT_TYPE"))
if [[ $? -ne 0 || ${#CAN_INTERFACES[@]} -eq 0 ]]; then
    log_error "Failed to load CAN interfaces for robot type: $ROBOT_TYPE"
    exit 1
fi

# Run main function
main
