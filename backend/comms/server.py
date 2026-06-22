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
        
        self._queues: dict[str, asyncio.Queue] = {}
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
        self._queues.clear()
        if self._server is not None:
            self._server.close()
            try:
                await self._server.wait_closed()
            except Exception:
                pass
            self._server = None

    def send(self, robot_id: str, message: dict[str, Any]) -> None:
        self._queue_for(robot_id).put_nowait(message)

    def _queue_for(self, robot_id: str) -> asyncio.Queue:
        queue = self._queues.get(robot_id)
        if queue is None:
            queue = asyncio.Queue()
            self._queues[robot_id] = queue
        return queue

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        line = await reader.readline()
        if not line:
            writer.close()
            return
        try:
            hello = json.loads(line.decode("utf-8"))
        except json.JSONDecodeError:
            writer.close()
            return
        if hello.get("type") != "register" or "robot_id" not in hello:
            writer.close()
            return

        robot_id = str(hello["robot_id"])
        self._writers[robot_id] = writer
        sender = asyncio.create_task(self._send_loop(robot_id, writer))

        try:
            await self._recv_loop(robot_id, reader)
        finally:
            sender.cancel()
            self._writers.pop(robot_id, None)
            try:
                writer.close()
            except Exception:
                pass

    async def _send_loop(self, robot_id: str, writer: asyncio.StreamWriter) -> None:
        queue = self._queue_for(robot_id)
        while True:
            message = await queue.get()
            writer.write(encode(message))
            await writer.drain()


    async def _recv_loop(self, robot_id: str, reader: asyncio.StreamReader) -> None:
        while True:
            line = await reader.readline()
            if not line:
                break
            try:
                message = json.loads(line.decode("utf-8"))
            except json.JSONDecodeError:
                continue
            if message.get("type") == "state":
                key = message.get("robot_id", robot_id)
                self.latest_states[key] = message
                if self.on_state is not None:
                    self.on_state(message)
