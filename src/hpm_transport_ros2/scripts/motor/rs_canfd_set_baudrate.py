#!/usr/bin/env python3
"""Set an RS06 CAN-FD data rate through the isolated MCAN0 baud tool image."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import sys
import time

import find_rs_motor as rs
from hpm_udp import UdpBusinessSocket


DEFAULT_IP = "192.168.2.10"
DEFAULT_PORT = 5001
MASTER_ID = 0xFD
CAN_PORT = 0
GETID_DATA = bytes(8)
SET_5M_DATA = bytes.fromhex("01 02 03 04 05 06 05 00")
SAVE_PARAMETERS_DATA = bytes.fromhex("01 02 03 04 05 06 07 08")
RESPONSE_MARK = 0xFE
MAX_MOTOR_ID = 0x7F
WORKSPACE = Path(__file__).resolve().parents[2]


def response_motor(frame: dict, motor_id: int) -> int | None:
    if (
        frame["msg_id"] != 0x10 + (motor_id & 0x0F)
        or frame["port"] != CAN_PORT
        or frame["ide"] != rs.IDE_EXT
        or frame["dlc"] != 8
    ):
        return None
    found = rs.unpack_getid_rsp(frame["can_id"])
    return found if found == motor_id else None


def response_saved_parameters(frame: dict, motor_id: int) -> bool:
    expected_can_id = (0x02 << 24) | (motor_id << 8) | MASTER_ID
    return (
        frame["msg_id"] == 0x10 + (motor_id & 0x0F)
        and frame["port"] == CAN_PORT
        and frame["ide"] == rs.IDE_EXT
        and frame["dlc"] == 8
        and frame["can_id"] == expected_can_id
    )


def valid_uid(uid: int) -> bool:
    return uid not in (0, (1 << 64) - 1)


def append_record(path: Path, **record: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps({"time": datetime.now(timezone.utc).isoformat(), **record}) + "\n")


def log_getid_request(path: Path, motor_id: int, stage: str) -> None:
    append_record(path, type="request", operation="getid", stage=stage,
                  motor_id=motor_id, bus=CAN_PORT, master_id=MASTER_ID,
                  can_id=rs.rs_getid_tx(MASTER_ID, motor_id), data=GETID_DATA.hex())


class RsCanClient:
    def __init__(self, ip: str, port: int, timeout_ms: int) -> None:
        self.sock = UdpBusinessSocket((ip, port), timeout=5)
        self.sock.settimeout(2)
        self.rx = rs.RecvBuf(self.sock)
        self.timeout_s = timeout_ms / 1000
        self.heartbeat = 0
        self.started = time.monotonic()

    def close(self) -> None:
        self.sock.close()

    def send(self, can_id: int, data: bytes) -> None:
        self.heartbeat = 1 if self.heartbeat == 0xFFFF else self.heartbeat + 1
        timestamp = int((time.monotonic() - self.started) * 1000) & 0xFFFFFFFF
        wire = rs.serialize_frame(self.heartbeat, timestamp, CAN_PORT, can_id, data)
        if len(wire) != rs.WIRE:
            raise ValueError(f"UDP business frame length mismatch: {len(wire)}")
        self.sock.sendall(wire)

    def getid(self, motor_id: int) -> int:
        self.rx.drain(0.05)
        can_id = rs.rs_getid_tx(MASTER_ID, motor_id)
        self.send(can_id, GETID_DATA)
        frames = self.rx.drain(self.timeout_s)
        uids = {
            rs.decode_mcu_uid(frame["data"])
            for frame in frames
            if response_motor(frame, motor_id) == motor_id
        }
        if not uids:
            raise TimeoutError(f"ID 0x{motor_id:02X} GETID timed out")
        if len(uids) != 1:
            raise RuntimeError(f"Multiple different MCU UIDs replied for ID 0x{motor_id:02X}")
        uid = uids.pop()
        if not valid_uid(uid):
            raise RuntimeError(f"Invalid MCU UID 0x{uid:016X}")
        return uid

    def save_parameters(self, motor_id: int) -> dict | None:
        self.rx.drain(0.05)
        can_id = (0x16 << 24) | (MASTER_ID << 8) | motor_id
        self.send(can_id, SAVE_PARAMETERS_DATA)
        return next(
            (frame for frame in self.rx.drain(self.timeout_s)
             if response_saved_parameters(frame, motor_id)),
            None,
        )


def load_set_record(path: Path) -> tuple[int, int]:
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()
               if line.strip()]
    request_index = next((index for index in range(len(records) - 1, -1, -1)
                          if records[index].get("type") == "request"
                          and records[index].get("operation") == "set_5m"), None)
    if request_index is None:
        raise ValueError(f"No set_5m record found in {path}")
    selected = records[request_index]
    motor_id, uid = selected.get("motor_id"), selected.get("uid")
    after_request = records[request_index + 1:]
    set_ack = next((index for index, item in enumerate(after_request)
                    if item.get("type") == "response" and item.get("operation") == "set_5m"
                    and item.get("stage") == "set_baudrate" and item.get("result") == "ACK_MATCHED"
                    and item.get("motor_id") == motor_id and item.get("uid") == uid), None)
    save_ack = next((index for index, item in enumerate(after_request)
                     if item.get("type") == "response" and item.get("operation") == "save_parameters"
                     and item.get("result") == "ACK_MATCHED_TYPE2"
                     and item.get("motor_id") == motor_id and item.get("uid") == uid), None)
    if set_ack is None or save_ack is None or save_ack <= set_ack:
        raise ValueError("Record lacks matched type-23 and type-22 save acknowledgements; it cannot verify a saved baud rate")
    if not isinstance(motor_id, int) or not 0 <= motor_id <= MAX_MOTOR_ID:
        raise ValueError("Record contains an invalid motor ID")
    if not isinstance(uid, int) or not valid_uid(uid):
        raise ValueError("Record contains an invalid MCU UID")
    return motor_id, uid


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="RS06 CAN-FD data-rate migration via HPM MCAN0")
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument("--query", action="store_true", help="query the MCU UID at a known motor ID")
    operation.add_argument("--set-5m", action="store_true", help="send one type-23 command selecting 5 Mbps")
    operation.add_argument("--verify-5m", action="store_true", help="verify the recorded MCU after restoring 1M/5M firmware")
    parser.add_argument("--motor-id", type=lambda value: int(value, 0), help="RS CAN_ID (0x00..0x7F)")
    parser.add_argument("--record", type=Path, help="JSONL log; defaults to ros2_ws/log/rs_canfd_baudrate/")
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--dry-run", action="store_true", help="show the CAN request without connecting")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = make_parser()
    args = parser.parse_args(argv)
    if not 1 <= args.timeout_ms <= 60000:
        parser.error("--timeout-ms must be in 1..60000")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in 1..65535")
    if args.verify_5m:
        if args.motor_id is not None or args.record is None:
            parser.error("--verify-5m requires --record and takes motor ID from the record")
        try:
            motor_id, expected_uid = load_set_record(args.record)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            parser.error(str(exc))
    else:
        if args.motor_id is None or not 0 <= args.motor_id <= MAX_MOTOR_ID:
            parser.error("--query and --set-5m require --motor-id in 0x00..0x7F")
        motor_id = args.motor_id
        expected_uid = None

    record = args.record or WORKSPACE / "log" / "rs_canfd_baudrate" / (
        datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + ".jsonl"
    )
    query_can_id = rs.rs_getid_tx(MASTER_ID, motor_id)
    set_can_id = (0x17 << 24) | (MASTER_ID << 8) | motor_id
    print(f"HPM={args.ip}:{args.port} MCAN0 RS-ID=0x{motor_id:02X} host-ID=0x{MASTER_ID:02X}")
    if args.query:
        print(f"GETID ext-ID=0x{query_can_id:08X} data={GETID_DATA.hex()}")
    elif args.set_5m:
        print(f"SET 5M ext-ID=0x{set_can_id:08X} data={SET_5M_DATA.hex()}")
        save_can_id = (0x16 << 24) | (MASTER_ID << 8) | motor_id
        response_can_id = (0x02 << 24) | (motor_id << 8) | MASTER_ID
        print(f"SAVE ext-ID=0x{save_can_id:08X} data={SAVE_PARAMETERS_DATA.hex()} expects type-2 ID=0x{response_can_id:08X}")
    else:
        print(f"VERIFY 5M after production firmware restore; expected UID=0x{expected_uid:016X}")
    print(f"JSONL={record}")
    if args.dry_run:
        print("DRY RUN: no TCP connection or CAN command was made")
        return 0

    if args.verify_5m:
        try:
            answer = input("确认已恢复 MCAN0 1M/5M 生产固件，并已将该电机断电重启？输入 yes 继续: ")
        except (EOFError, KeyboardInterrupt):
            return 1
        if answer.strip() != "yes":
            print("取消复检")
            return 1

    client = None
    try:
        client = RsCanClient(args.ip, args.port, args.timeout_ms)
        if args.query:
            log_getid_request(record, motor_id, "query")
            uid = client.getid(motor_id)
            append_record(record, type="response", operation="query", motor_id=motor_id,
                          bus=CAN_PORT, uid=uid, result="CONFIRMED")
            print(f"CONFIRMED ID=0x{motor_id:02X} UID=0x{uid:016X}")
            return 0

        if args.set_5m:
            log_getid_request(record, motor_id, "preflight")
            uid = client.getid(motor_id)
            print(f"当前电机 UID=0x{uid:016X}")
            answer = input("确认仅修改此 RS 电机，并已停止其运动？输入 yes 发送一次: ")
            if answer.strip() != "yes":
                print("取消写入")
                return 1
            request = {
                "type": "request", "operation": "set_5m", "motor_id": motor_id,
                "bus": CAN_PORT, "master_id": MASTER_ID, "can_id": set_can_id,
                "data": SET_5M_DATA.hex(), "uid": uid,
            }
            append_record(record, **request)
            client.rx.drain(0.05)
            client.send(set_can_id, SET_5M_DATA)
            try:
                frames = client.rx.drain(client.timeout_s)
                replies = {
                    rs.decode_mcu_uid(frame["data"])
                    for frame in frames
                    if response_motor(frame, motor_id) == motor_id
                }
                if uid in replies and len(replies) == 1:
                    append_record(record, type="response", operation="set_5m",
                                  stage="set_baudrate", motor_id=motor_id,
                                  uid=uid, result="ACK_MATCHED")
                    save_can_id = (0x16 << 24) | (MASTER_ID << 8) | motor_id
                    print("Type-23 ACK matched; sending one type-22 save command")
                    append_record(record, type="request", operation="save_parameters",
                                  stage="save_after_set_5m", motor_id=motor_id,
                                  bus=CAN_PORT, master_id=MASTER_ID, can_id=save_can_id,
                                  data=SAVE_PARAMETERS_DATA.hex(), uid=uid)
                    try:
                        save_reply = client.save_parameters(motor_id)
                        if save_reply is None:
                            append_record(record, type="response", operation="save_parameters",
                                          motor_id=motor_id, uid=uid, result="TIMEOUT_UNKNOWN")
                            result, message = "SAVE_TIMEOUT_UNKNOWN", "类型22已发送但未收到匹配的类型2反馈；结果未知，不要重发"
                        else:
                            append_record(record, type="response", operation="save_parameters",
                                          motor_id=motor_id, uid=uid, result="ACK_MATCHED_TYPE2",
                                          response_can_id=save_reply["can_id"],
                                          response_data=save_reply["data"].hex())
                            result, message = "SAVE_ACK_MATCHED_POWER_CYCLE_REQUIRED", "类型23应答及类型22后的匹配类型2反馈均已收到；断电重启并恢复生产固件复检"
                    except (OSError, ConnectionError, TimeoutError) as exc:
                        append_record(record, type="response", operation="save_parameters",
                                      motor_id=motor_id, uid=uid, result="CONNECTION_LOST_UNKNOWN",
                                      message=str(exc))
                        result, message = "SAVE_CONNECTION_LOST_UNKNOWN", f"类型22发送后连接中断，结果未知，不要重发：{exc}"
                elif replies:
                    result, message = "UID_MISMATCH", "收到的 MCU UID 与写入前不一致；停止操作"
                else:
                    result, message = "TIMEOUT_UNKNOWN", "写入结果未知；不重发，先断电重启并查询"
            except (OSError, ConnectionError, TimeoutError) as exc:
                result, message = "CONNECTION_LOST_UNKNOWN", f"发送后连接中断，结果未知：{exc}"
            append_record(record, type="response", operation="set_5m", motor_id=motor_id,
                          uid=uid, result=result, message=message)
            print(f"{result}: {message}")
            return 0 if result == "SAVE_ACK_MATCHED_POWER_CYCLE_REQUIRED" else 2

        outcomes = []
        for attempt in range(1, 4):
            log_getid_request(record, motor_id, f"verify_{attempt}")
            try:
                uid = client.getid(motor_id)
                ok = uid == expected_uid
                outcomes.append({"attempt": attempt, "uid": uid, "result": "PASS" if ok else "UID_MISMATCH"})
                print(f"{attempt}/3 {'PASS' if ok else 'FAIL'} UID=0x{uid:016X}")
            except (OSError, ConnectionError, TimeoutError, RuntimeError) as exc:
                outcomes.append({"attempt": attempt, "result": "FAIL", "message": str(exc)})
                print(f"{attempt}/3 FAIL: {exc}")
        passed = len(outcomes) == 3 and all(item["result"] == "PASS" for item in outcomes)
        append_record(record, type="response", operation="verify_5m", motor_id=motor_id,
                      expected_uid=expected_uid, result="PASS" if passed else "FAIL", attempts=outcomes)
        print("PASS" if passed else "FAIL")
        return 0 if passed else 1
    except (OSError, ConnectionError, TimeoutError, RuntimeError, ValueError) as exc:
        append_record(record, type="error", operation="verify_5m" if args.verify_5m else
                      "set_5m" if args.set_5m else "query", motor_id=motor_id,
                      error=str(exc), result="UNKNOWN" if args.set_5m else "FAILED")
        print(f"FAILED: {exc}", file=sys.stderr)
        return 2 if args.set_5m else 1
    except (EOFError, KeyboardInterrupt):
        print("操作中断。若已发出速率写入命令，结果未知；不要自动重发。", file=sys.stderr)
        return 2
    except (EOFError, KeyboardInterrupt):
        print("操作中断。若已发出速率写入命令，结果未知；不要自动重发。", file=sys.stderr)
        return 2
    finally:
        if client is not None:
            client.close()


def self_test() -> None:
    import tempfile

    assert rs.rs_getid_tx(MASTER_ID, 0x7F) == 0x0000FD7F
    assert (0x17 << 24) | (MASTER_ID << 8) | 0x7F == 0x1700FD7F
    assert SET_5M_DATA == bytes((1, 2, 3, 4, 5, 6, 5, 0))
    assert (0x16 << 24) | (MASTER_ID << 8) | 0x7F == 0x1600FD7F
    assert SAVE_PARAMETERS_DATA == bytes((1, 2, 3, 4, 5, 6, 7, 8))
    save_reply = {"msg_id": 0x1F, "port": 0, "ide": rs.IDE_EXT, "dlc": 8,
                  "can_id": 0x02007FFD, "data": bytes(8)}
    assert response_saved_parameters(save_reply, 0x7F)
    for changed in ({"port": 1}, {"ide": 0}, {"dlc": 7}, {"msg_id": 0x10},
                    {"can_id": 0x02007FFE}):
        assert not response_saved_parameters({**save_reply, **changed}, 0x7F)
    with tempfile.TemporaryDirectory() as temp_dir:
        record_path = Path(temp_dir) / "migration.jsonl"
        set_request = {"type": "request", "operation": "set_5m",
                       "motor_id": 0x7F, "uid": 0x0102030405060708}
        record_path.write_text(json.dumps(set_request) + "\n", encoding="utf-8")
        try:
            load_set_record(record_path)
            raise AssertionError("verification must reject a record without type-22 save ACK")
        except ValueError:
            pass
        record_path.write_text("\n".join(json.dumps(item) for item in (
            set_request,
            {"type": "response", "operation": "set_5m", "stage": "set_baudrate",
             "motor_id": 0x7F, "uid": 0x0102030405060708, "result": "ACK_MATCHED"},
            {"type": "response", "operation": "save_parameters", "motor_id": 0x7F,
             "uid": 0x0102030405060708, "result": "ACK_MATCHED_TYPE2"},
        )) + "\n", encoding="utf-8")
        assert load_set_record(record_path) == (0x7F, 0x0102030405060708)
    wire = rs.serialize_frame(1, 0, CAN_PORT, 0x0000FD7F, GETID_DATA)
    assert len(wire) == rs.WIRE and rs.parse_frame(wire)["can_id"] == 0x0000FD7F
    reply = {"msg_id": 0x1F, "port": 0, "ide": rs.IDE_EXT, "dlc": 8,
             "can_id": 0x00007FFE, "data": bytes.fromhex("01 02 03 04 05 06 07 08")}
    assert response_motor(reply, 0x7F) == 0x7F
    for changed in ({"port": 1}, {"ide": 0}, {"dlc": 7}, {"msg_id": 0x10},
                    {"can_id": 0x00007FFD}):
        assert response_motor({**reply, **changed}, 0x7F) is None

    import threading

    local, peer = socket.socketpair()
    client = object.__new__(RsCanClient)
    client.sock = local
    client.rx = rs.RecvBuf(local)
    client.timeout_s = 0.1
    client.heartbeat = 0
    client.started = time.monotonic()

    def answer_getid() -> None:
        request = peer.recv(rs.WIRE)
        sent = rs.parse_frame(request)
        assert sent is not None and sent["can_id"] == 0x0000FD7F
        wrong_bus = bytearray(rs.serialize_frame(2, 1, 1, 0x00007FFE, reply["data"]))
        response = bytearray(rs.serialize_frame(3, 2, CAN_PORT, 0x00007FFE, reply["data"]))
        for wire in (wrong_bus, response):
            wire[5] = 0x1F
            wire[-2:] = rs.crc16_modbus(wire[:28]).to_bytes(2, "big")
        peer.sendall(wrong_bus + response[:13])
        peer.sendall(response[13:])

    responder = threading.Thread(target=answer_getid)
    responder.start()
    assert client.getid(0x7F) == 0x0102030405060708
    responder.join(timeout=1)
    local.close()
    peer.close()

    local, peer = socket.socketpair()
    client = object.__new__(RsCanClient)
    client.sock = local
    client.rx = rs.RecvBuf(local)
    client.timeout_s = 0.1
    client.heartbeat = 0
    client.started = time.monotonic()

    def answer_save() -> None:
        request = peer.recv(rs.WIRE)
        sent = rs.parse_frame(request)
        assert sent is not None
        assert sent["can_id"] == 0x1600FD7F and sent["data"] == SAVE_PARAMETERS_DATA
        response = bytearray(rs.serialize_frame(4, 3, CAN_PORT, 0x02007FFD, bytes(8)))
        response[5] = 0x1F
        response[-2:] = rs.crc16_modbus(response[:28]).to_bytes(2, "big")
        peer.sendall(response)

    responder = threading.Thread(target=answer_save)
    responder.start()
    assert client.save_parameters(0x7F) is not None
    responder.join(timeout=1)
    local.close()
    peer.close()

    local, peer = socket.socketpair()
    client = object.__new__(RsCanClient)
    client.sock = local
    client.rx = rs.RecvBuf(local)
    client.timeout_s = 0.02
    client.heartbeat = 0
    client.started = time.monotonic()
    assert client.save_parameters(0x7F) is None
    request = rs.parse_frame(peer.recv(rs.WIRE))
    assert request is not None and request["can_id"] == 0x1600FD7F
    local.close()
    peer.close()

    local, peer = socket.socketpair()
    client = object.__new__(RsCanClient)
    client.sock = local
    client.rx = rs.RecvBuf(local)
    client.timeout_s = 0.02
    client.heartbeat = 0
    client.started = time.monotonic()
    try:
        client.getid(0x7F)
        raise AssertionError("GETID without a response must time out")
    except TimeoutError:
        pass
    assert len(peer.recv(rs.WIRE)) == rs.WIRE
    local.close()
    peer.close()

    local, peer = socket.socketpair()
    peer.close()
    client = object.__new__(RsCanClient)
    client.sock = local
    client.rx = rs.RecvBuf(local)
    client.timeout_s = 0.02
    client.heartbeat = 0
    client.started = time.monotonic()
    try:
        client.getid(0x7F)
        raise AssertionError("a closed TCP peer must fail")
    except ConnectionError:
        pass
    local.close()
    print("RS CAN-FD baud tool self-test: PASS")


if __name__ == "__main__":
    if sys.argv[1:] == ["--self-test"]:
        self_test()
        raise SystemExit(0)
    raise SystemExit(main())
