from __future__ import annotations

import asyncio
import json
from typing import Callable

from backend.robots.base import RobotBase

from .protocol import encode, register_message, state_message, waypoints_from_order


class RobotCommClient:
    """TCP client on the robot side.

    It connects to the fleet manager, registers, then waits for order messages
    and applies them to its robot.
    """

    def __init__(self, robot: RobotBase, on_order: Callable[[str, list], None]) -> None:
        self.robot = robot
        self.on_order = on_order
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._tasks: list[asyncio.Task] = []
        self._running = False

    async def start(self, host: str, port: int, state_period_s: float = 0.2) -> None:
        self._reader, self._writer = await asyncio.open_connection(host, port)
        self._writer.write(encode(register_message(self.robot.state.id)))
        await self._writer.drain()
        self._running = True
        self._tasks = [
            asyncio.create_task(self._recv_loop()),
            asyncio.create_task(self._state_loop(state_period_s)),
        ]

    async def stop(self) -> None:
        self._running = False
        for task in self._tasks:
            task.cancel()
        self._tasks = []
        if self._writer is not None:
            try:
                self._writer.close()
            except Exception:
                pass

    async def _recv_loop(self) -> None:
        assert self._reader is not None
        while self._running:
            line = await self._reader.readline()
            if not line:
                break
            try:
                message = json.loads(line.decode("utf-8"))
            except json.JSONDecodeError:
                continue
            if message.get("type") == "order":
                self.on_order(message.get("robot_id"), waypoints_from_order(message))

    async def _state_loop(self, period: float) -> None:
        assert self._writer is not None
        while self._running:
            try:
                self._writer.write(encode(state_message(self.robot)))
                await self._writer.drain()
            except Exception:
                break
            await asyncio.sleep(period)

