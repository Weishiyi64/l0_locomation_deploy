# Unified application OTA

The sender supports HPM6E80 TCP OTA, HPM5151 direct UART OTA, and ESP32-P4
application OTA through the HPM UART5 gateway or a direct UART adapter.
Deploy the entire scripts/ota directory, including ota_send.py,
hpm_serial_ota.py and p4_ota.py. Only UART modes require pyserial.

## Hardware and initial installation

HPM LUV J17 pin 2 (UART5 TX / PC23) connects to P4 GPIO5 (UART1 RX).
J17 pin 3 (UART5 RX / PC22) connects to P4 GPIO4 (UART1 TX).
J17 pin 4 connects to GND. Pin 1 is 5 V power; leave it unconnected when both
boards have their own supply. UART signals use the board's 3.3 V levels.
Both ends use 1000000 bit/s, 8N1, no RTS/CTS.

P4 UART0 at 115200 remains the log/download console. Protocol traffic uses
UART1 exclusively. Screen initialization defaults off; its source is preserved.
P4 retains the v1.x variant, 360 MHz, 16 MB Flash and two 1700 KiB App slots.

Before the first framed OTA, perform a complete local P4 flash with the new
bootloader, existing partition table, initial otadata and baseline App:

    cd /home/a/myws/l0_backscreen/uart_dma_ota
    . /home/a/.espressif/v6.0/esp-idf/export.sh
    idf.py build
    idf.py merge-bin -o factory_full.bin -f raw
    idf.py -p <P4-console-port> flash monitor

The merge command is offline. The flash command resets/programs the board.
merge-bin writes its relative output inside build/; the merged raw image starts
at Flash offset 0x0.
Do not send factory_full.bin or bootloader.bin through application OTA.
The baseline must show APP_ENTER, OTA_LISTENING UHCI/DMA, APP_RUNNING,
APP_VALID and periodic APP_HEARTBEAT before testing another update.

Build the HPM gateway with compile_app.sh. Deploy its normal
build/ota/pack/update_sign.bin through the existing HPM update path first.
The new gateway listens on TCP/5002. TCP/5001 remains HPM self-update;
the two update paths share a maintenance token.

## Sender commands

Replace device/IP/file examples with actual values. A dry run never opens
a device or socket:

    python3 ota_send.py hpm6e80 --host 192.168.1.10 --file update_sign.bin --dry-run
    python3 ota_send.py hpm5151 --serial /dev/ttyS5 --file hpm5151.bin --dry-run
    python3 ota_send.py esp32p4 --file uart_dma_ota.bin --dry-run

    python3 ota_send.py hpm6e80 --host 192.168.1.10 --file update_sign.bin
    python3 ota_send.py hpm5151 --serial /dev/ttyS5 --file hpm5151.bin
    python3 ota_send.py esp32p4 --host 192.168.1.10 --file uart_dma_ota.bin
    python3 ota_send.py esp32p4 --serial /dev/ttyUSB1 --file uart_dma_ota.bin
    python3 ota_send.py esp32p4 --host 192.168.1.10 --query
    python3 ota_send.py esp32p4 --host 192.168.1.10 --query --file uart_dma_ota.bin

Serial device examples are not automatically discovered or verified.
Stop the process owning the UART first. The sender requests exclusive access.
The legacy ota_send.py HOST PACKAGE syntax remains HPM6E80 TCP/5001.

HPM packages require the original 4 KiB header, matching device ID (0x6750
for this HPM6E80 format, 0x5151 for HPM5151), length and SHA256.
HPM OTA_OK means the image was accepted before reset; it is not boot proof.

P4 uses a native unsigned ESP-IDF App image. Before connecting, the sender
checks its chip ID, App descriptor, segment bounds, checksum, appended SHA
(when present) and size; then it checks actual chip revision/slot capacity.
Signed/encrypted and merged images are outside this release.

P4 --timeout defaults to 2 seconds, --retries to 3, BEGIN/END to 30 seconds,
and --boot-timeout to 30 seconds. Exit 0 requires BOOT_CONFIRMED with the
expected running partition, valid boot state and full-file SHA256.
Exit 3 means COMMIT may have executed but its final result was not proved.
Query/inspect the device before repeating an update after exit 3.

## P4UP v1 wire contract

All integers are little endian. The 20-byte header is <4sBBHIIHH>:
Magic P4UP, version 1, message type, zero flags, session ID, sequence,
payload length and zero reserved. Request session IDs are nonzero.
Payload is at most 1100 bytes, followed by CRC32/ISO-HDLC over header+payload.
CRC vector: ASCII 123456789 -> 0xCBF43926. Maximum frame size is 1124 bytes.

Messages: HELLO=0x01, BEGIN=0x30, DATA=0x31, END=0x32, COMMIT=0x33,
ABORT=0x34, QUERY=0x35, ACK=0x70, NACK=0x71, BOOT_REPORT=0x72.
ACK/NACK echo request session/sequence; BOOT_REPORT uses zero IDs.

BEGIN is <IIII32sB3x> (52 bytes): total image length, numeric version (0),
board ID 0x50340001, asset ABI (0), full-file SHA256, image type APP (0),
then three zero bytes. DATA is offset:u32 plus 1..1024 bytes. The first block
must contain the native image header and App descriptor (288 bytes).
HELLO, END, COMMIT and ABORT have empty payloads.
QUERY contains a u32 length: 0 requests status, a nonzero bounded length
requests SHA256 of that range of the actual running application partition.
This full-file SHA is different from the ELF digest in App metadata.

Every status payload is exactly 100 bytes:

* Bytes 0..11: <HHII> status, OTA state, next offset, native error/detail.
* Bytes 12..35: <IHHIIIB3x> board ID, chip ID, full chip revision,
  next-slot capacity, running address, session target address,
  IDF boot state and three reserved bytes.
* Bytes 36..67: native App version, fixed 32 bytes.
* Bytes 68..99: requested running-partition SHA256, otherwise zero.

States: 0 IDLE, 1 PREPARING, 2 RECEIVING, 3 VERIFYING, 4 VERIFIED,
5 COMMITTING, 6 FAILED. IDF boot VALID is 2; UNDEFINED is 255.
Errors: 0 OK, 1 BAD_FRAME, 2 SESSION, 3 STATE, 4 LENGTH, 5 OFFSET,
6 SEQUENCE, 7 HASH, 8 FLASH, 9 TARGET, 10 BUSY, 11 TIMEOUT,
12 OVERFLOW, 13 NOT_READY.

P4 responses have DMA alignment zeros outside the frame; parsers ignore them.
Idle EOF terminates a DMA transaction, not a protocol frame or firmware.
Frames may span DMA transactions.

The host waits for the P4 write ACK before sending another block. Retries use
the identical sequence/offset/data. The device compares the complete previous
block, never double-writes or double-hashes it. Corrupt frames are discarded.
A matching duplicate BEGIN does not erase again; duplicate END in VERIFIED
returns the verified state.

The callback copies bytes into preallocated queue items. Flash writes happen
only in ota_task. UHCI and ota_task share core 0, so SDK RX/FSM cleanup has
finished before task-side re-arm. RX is re-armed before parsing/sending ACK.
RX start also defers the local EOF ISR until the v6.0 driver publishes its
RUN state, avoiding a reproduced immediate-EOF / RX state overwrite race.
No queued pointer borrows memory that DMA may overwrite. Overflow aborts the
session. Startup without a BEGIN never erases the spare App.

END checks length, streaming SHA, esp_ota_end and Flash readback SHA.
Only COMMIT changes the boot partition. A session expires after 60 idle seconds.
After a pre-commit disconnect, restart at byte zero once that session expires.
Across-reset resume is not implemented.

The gateway exchanges ESPOTA1\n / ESP_READY\n before forwarding frames.
Gateway READY is not P4 READY. UART errors close the connection rather than
report success. The gateway never writes HPM Flash, claims the motor feedback
host, or uses motor shared rings/cross-core transport.

## Checks and runtime acceptance

From the HPM repository:

    PYTHONDONTWRITEBYTECODE=1 python3 deploy_ws/src/hpm_transport_ros2/scripts/ota/test_ota_send.py
    PYTHONDONTWRITEBYTECODE=1 python3 deploy_ws/src/hpm_transport_ros2/scripts/ota/test_p4_ota.py
    python3 hpm5100/scripts/ota/ota_send.py --self-test

From the P4 repository:

    cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
      tests/test_p4_ota_protocol.c -o /tmp/test_p4_wire
    /tmp/test_p4_wire
    cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
      tests/test_p4_ota_session.c -o /tmp/test_p4_session
    /tmp/test_p4_session
    python3 tests/check_app_running.py /dev/ttyUSB0 --reset --seconds 600

The session test invokes the real firmware command handler with fake SDK/Flash.
It checks erase/write/commit ordering, idempotence and failure cleanup.
Its fake hash checks byte accounting, not cryptographic correctness.

The opt-in pytest hardware test uses the existing pytest-embedded setup:
P4_OTA_SENDER points to unified ota_send.py, P4_OTA_PORT selects the UART1
adapter, and the DUT console is a separate port.

Hardware acceptance: 10 minutes idle without a Flash erase; direct UART and
gateway transfers; 10 A/B swaps; lost/corrupt frames, disconnections, and resets
during write, verification, commit and first boot. Verify running partition,
full image SHA and boot state. Observe HPM UDP feedback passively during P4
OTA; do not command motor motion for this check.

The baseline confirms itself after UART initialization and two healthy task
heartbeats. A pending App failing this self-test requests rollback at 10 seconds.
Future versions must retain the service/self-test. The IDF bootloader does not
provide a UART OTA listener or reset a running App that never confirms itself.
Local USB/UART download is the recovery path when the App cannot provide OTA.
