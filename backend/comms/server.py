from __future__ import annotations

import asyncio
import json
from typing import Any, Callable

from .protocol import encode


class FleetCommServer:
    """TCP server on the fleet-manager side
    """

    def __init__(self) -> None:
        self._server: asyncio.AbstractServer | None = None
        self._writers: dict[str, asyncio.StreamWriter] = {}
        self.latest_states: dict[str, dict[str, Any]] = {}
        self.on_state: Callable[[dict[str, Any]], None] | None = None
        self.host: str = "127.0.0.1"
        self.port: int = 0

    async def start(self, host: str = "127.0.0.1", port: int = 0) -> None:
        self._server = await asyncio.start_server(self._handle_client, host, port)
        sockname = self._server.sockets[0].getsockname()
        self.host, self.port = sockname[0], sockname[1]

    async def stop(self) -> None:
        for writer in list(self._writers.values()):
            try:
                writer.close()
            except Exception:
                pass
        self._writers.clear()
        if self._server is not None:
            self._server.close()
            try:
                await self._server.wait_closed()
            except Exception:
                pass
            self._server = None

    def send(self, robot_id: str, message: dict[str, Any]) -> None:
        writer = self._writers.get(robot_id)
        if writer is None:
            print(f"no connection for {robot_id}, dropping {message.get('type')}")
            return
        writer.write(encode(message))

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        data = await reader.read(4096)
        if not data:
            writer.close()
            return
        try:
            hello = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError:
            writer.close()
            return
        if hello.get("type") != "register" or "robot_id" not in hello:
            writer.close()
            return

        robot_id = str(hello["robot_id"])
        self._writers[robot_id] = writer

        try:
            await self._recv_loop(robot_id, reader)
        finally:
            self._writers.pop(robot_id, None)
            try:
                writer.close()
            except Exception:
                pass

    async def _recv_loop(self, robot_id: str, reader: asyncio.StreamReader) -> None:
        while True:
            data = await reader.read(4096)
            if not data:
                break
            try:
                message = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                print(f"bad json chunk from {robot_id}")
                continue
            if message.get("type") == "state":
                key = message.get("robot_id", robot_id)
                self.latest_states[key] = message
                if self.on_state is not None:
                    self.on_state(message)
