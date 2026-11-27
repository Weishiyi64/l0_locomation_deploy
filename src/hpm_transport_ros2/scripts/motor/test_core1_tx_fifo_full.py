#!/usr/bin/env python3
"""Compile the actual Core1 TX branch with FIFO-full and bus-off CAN ports."""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parents[5] / "app/lwip_with_tcp_udp_dual_core_printf/core1/src/main_core1.c"
text = source.read_text()
start = text.index("                    can = mcan_from_port(info->port);")
end_marker = "                    g_tx_cmd_consumed_count++;"
end = text.index(end_marker, start) + len(end_marker)
branch = text[start:end]
harness = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define MCAN_CH_COUNT 4
#define TX_RING_CAPACITY 8
#define FRAME_IDE_EXTENDED 1
#define status_success 0
#define status_mcan_txfifo_full 1
typedef struct { int port; } MCAN_Type;
typedef struct { uint8_t data_8[8], dlc, use_ext_id, canfd_frame, bitrate_switch; uint32_t std_id, ext_id; } mcan_tx_frame_t;
typedef struct { uint8_t port, ide, dlc, data[8]; uint32_t can_id; } frame_info;
typedef struct { uint32_t tx_attempt, tx_queued, tx_fifo_full, tx_error, tx_drop; } port_diag_t;
typedef struct { uint32_t tail, head; frame_info entries[TX_RING_CAPACITY]; } tx_ring_t;
static MCAN_Type cans[MCAN_CH_COUNT] = {{0}, {1}, {2}, {3}};
static volatile port_diag_t g_mcan_port_diag[MCAN_CH_COUNT];
static volatile bool g_mcan_bus_off_latched[MCAN_CH_COUNT];
static unsigned g_tx_fifo_full_count, g_tx_error_count, g_tx_cmd_success_count;
static unsigned g_tx_cmd_drop_count, g_tx_cmd_consumed_count, contexts, cycle;
static bool fifo_diag_printed;
static unsigned fifo_wait_cyc = 3;
static tx_ring_t storage = {.head = 4}, *ring = &storage;
static MCAN_Type *mcan_from_port(uint8_t port) { return port < MCAN_CH_COUNT ? &cans[port] : NULL; }
static bool mcan_port_is_classic_can(uint8_t port) { (void)port; return false; }
static int mcan_transmit_via_txfifo_nonblocking(MCAN_Type *can, mcan_tx_frame_t *tx, void *unused) { (void)tx; (void)unused; return can->port == 0 ? status_mcan_txfifo_full : status_success; }
static bool mcan_is_txfifo_full(MCAN_Type *can) { return can->port == 0; }
static unsigned hpm_csr_get_core_mcycle(void) { return cycle++; }
static void core1_mcan_tx_diag_print(uint8_t port, uint8_t ide, uint32_t id, MCAN_Type *can) { (void)port; (void)ide; (void)id; (void)can; }
static void remember_motor_context(const frame_info *info) { (void)info; contexts++; }
static void tx_ring_fence(void) {}
int main(void) {
    ring->entries[0] = (frame_info){.port = 0, .can_id = 1, .dlc = 8};
    ring->entries[1] = (frame_info){.port = 1, .can_id = 2, .dlc = 8};
    ring->entries[2] = (frame_info){.port = 3, .can_id = 3, .dlc = 8};
    ring->entries[3] = (frame_info){.port = 2, .can_id = 4, .dlc = 8};
    g_mcan_bus_off_latched[1] = true;
    g_mcan_bus_off_latched[3] = true;
    for (unsigned n = 0; n < 4; n++) {
        const frame_info *info = &ring->entries[ring->tail];
        MCAN_Type *can;
        mcan_tx_frame_t tx;
        int st;
        bool classic_can;
""" + branch + r"""
    }
    assert(ring->tail == 4 && g_tx_cmd_consumed_count == 4);
    assert(g_tx_fifo_full_count == 1 && g_tx_cmd_drop_count == 3);
    assert(g_tx_cmd_success_count == 1 && contexts == 1);
    assert(g_mcan_port_diag[1].tx_attempt == 0 && g_mcan_port_diag[1].tx_drop == 1);
    assert(g_mcan_port_diag[3].tx_attempt == 0 && g_mcan_port_diag[3].tx_drop == 1);
    assert(g_mcan_port_diag[2].tx_queued == 1);
    puts("PASS: bus-off ports are dropped; healthy CAN proceeds after FIFO-full CAN");
}
"""
with tempfile.TemporaryDirectory() as directory:
    c = Path(directory) / "check.c"
    binary = Path(directory) / "check"
    c.write_text(harness)
    subprocess.run(["cc", "-std=c11", str(c), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
