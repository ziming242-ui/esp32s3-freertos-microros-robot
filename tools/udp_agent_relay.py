"""Bidirectional UDP relay for ESP32 hotspot traffic to a WSL micro-ROS Agent.

Front side: Windows hotspot address, UDP 8888.
Back side:  WSL mirrored loopback, UDP 8889.

The relay deliberately avoids per-packet logging so XRCE reliable-stream traffic
is not delayed by console or file I/O.
"""

from __future__ import annotations

import argparse
import select
import socket
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-address", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=8888)
    parser.add_argument("--agent-address", default="127.0.0.1")
    parser.add_argument("--agent-port", type=int, default=8889)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    front = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    front.bind((args.listen_address, args.listen_port))

    back = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    back.connect((args.agent_address, args.agent_port))

    client: tuple[str, int] | None = None
    client_packets = 0
    agent_packets = 0
    next_report = time.monotonic() + 5.0

    print(
        f"relay ready: {args.listen_address}:{args.listen_port} "
        f"<-> {args.agent_address}:{args.agent_port}",
        flush=True,
    )

    while True:
        readable, _, _ = select.select((front, back), (), (), 1.0)

        if front in readable:
            payload, remote = front.recvfrom(65535)
            if remote != client:
                client = remote
                print(f"client selected: {client[0]}:{client[1]}", flush=True)
            back.send(payload)
            client_packets += 1

        if back in readable:
            payload = back.recv(65535)
            if client is not None:
                front.sendto(payload, client)
                agent_packets += 1

        now = time.monotonic()
        if now >= next_report:
            print(
                f"packets: client->agent={client_packets} "
                f"agent->client={agent_packets}",
                flush=True,
            )
            next_report = now + 5.0


if __name__ == "__main__":
    main()
