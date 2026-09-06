#!/usr/bin/env python3
"""Сводка по бегу tools/bag_record: топики, частоты, объёмы, при желании — дамп записей.

    python3 scripts/bag_info.py drive.bag [--dump 10] [--topic slam/pose]

Конверт ZmqMessage разбирается без сгенерированных protobuf-модулей (нужны только timestamp и
topic). Для полного декодирования полезной нагрузки сгенерируйте модули:
    protoc -I proto --python_out=scripts/pb proto/*.proto
и передайте --decode: тогда каждая запись печатается через zmq_message_pb2.
"""

from __future__ import annotations

import argparse
import struct
import sys
from collections import defaultdict
from pathlib import Path

MAGIC = b"XRBAG1\n"


def read_records(path: str):
    with open(path, "rb") as f:
        if f.read(len(MAGIC)) != MAGIC:
            raise SystemExit(f"{path}: не бег (нет заголовка XRBAG1)")
        while True:
            head = f.read(12)
            if len(head) < 12:
                return
            recv_us, length = struct.unpack("<QI", head)
            data = f.read(length)
            if len(data) < length:
                return
            yield recv_us, data


def _varint(buf: bytes, i: int) -> tuple[int, int]:
    result = shift = 0
    while True:
        b = buf[i]
        i += 1
        result |= (b & 0x7F) << shift
        if not b & 0x80:
            return result, i
        shift += 7


def envelope(data: bytes) -> tuple[int, str, int]:
    """(timestamp робота, topic, номер поля payload) из ZmqMessage без protobuf-модулей."""
    ts, topic, payload = 0, "", 0
    i = 0
    while i < len(data):
        key, i = _varint(data, i)
        field, wire = key >> 3, key & 7
        if wire == 0:
            val, i = _varint(data, i)
            if field == 1:
                ts = val
        elif wire == 2:
            length, i = _varint(data, i)
            chunk = data[i:i + length]
            i += length
            if field == 2:
                topic = chunk.decode("utf-8", "replace")
            elif field >= 3:
                payload = field
        elif wire == 1:
            i += 8
        elif wire == 5:
            i += 4
        else:
            break
    return ts, topic, payload


PAYLOADS = {3: "OccupancyMap", 4: "CmdVel", 5: "Pose2D", 6: "RobotState", 7: "Trajectory",
            8: "Goal", 9: "PlannerState", 10: "MissionCommand", 11: "MissionState"}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--topic", default="", help="только этот топик")
    ap.add_argument("--dump", type=int, default=0, help="напечатать первые N записей")
    ap.add_argument("--decode", action="store_true", help="декодировать payload через scripts/pb/*_pb2")
    args = ap.parse_args()

    pb = None
    if args.decode:
        sys.path.insert(0, str(Path(__file__).resolve().parent / "pb"))
        import zmq_message_pb2 as pb  # noqa: E402

    count: dict[str, int] = defaultdict(int)
    size: dict[str, int] = defaultdict(int)
    first: dict[str, int] = {}
    last: dict[str, int] = {}
    kinds: dict[str, str] = {}
    t_first = t_last = None
    dumped = 0
    for recv_us, data in read_records(args.bag):
        ts, topic, payload = envelope(data)
        if args.topic and topic != args.topic:
            continue
        count[topic] += 1
        size[topic] += len(data)
        first.setdefault(topic, recv_us)
        last[topic] = recv_us
        kinds[topic] = PAYLOADS.get(payload, f"field{payload}")
        t_first = recv_us if t_first is None else min(t_first, recv_us)
        t_last = recv_us if t_last is None else max(t_last, recv_us)
        if dumped < args.dump:
            dumped += 1
            if pb is not None:
                msg = pb.ZmqMessage()
                msg.ParseFromString(data)
                print(f"--- {recv_us} {topic}\n{msg}")
            else:
                print(f"{recv_us} {topic:20} {kinds[topic]:14} robot_ts={ts} {len(data)} байт")

    if t_first is None:
        print("бег пуст")
        return 1
    total_s = (t_last - t_first) / 1e6
    print(f"{args.bag}: {sum(count.values())} сообщений за {total_s:.1f} с")
    print(f"{'топик':22} {'тип':14} {'шт':>7} {'Гц':>6} {'КиБ':>8}")
    for topic in sorted(count):
        span = (last[topic] - first[topic]) / 1e6
        hz = (count[topic] - 1) / span if span > 0 and count[topic] > 1 else 0.0
        print(f"{topic:22} {kinds[topic]:14} {count[topic]:7d} {hz:6.1f} {size[topic] / 1024:8.0f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
