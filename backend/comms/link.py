from __future__ import annotations

from typing import Any, Callable, Iterable

from backend.robots.base import RobotBase

from .client import RobotCommClient
from .protocol import order_message
from .server import FleetCommServer


class FleetCommLink:
    """
    Communication link between the fleet-manager and the robots
    """

    def __init__(self) -> None:
        self.server = FleetCommServer()
        self.clients: list[RobotCommClient] = []
        self._started = False

    async def start(
        self,
        robots: Iterable[RobotBase],
        on_order: Callable[[str, list], None],
        host: str = "127.0.0.1",
        port: int = 0,
    ) -> None:
        await self.server.start(host, port)
        self.clients = []
        for robot in robots:
            client = RobotCommClient(robot, on_order)
            await client.start(self.server.host, self.server.port)
            self.clients.append(client)
        self._started = True

    async def stop(self) -> None:
        for client in self.clients:
            await client.stop()
        self.clients = []
        await self.server.stop()
        self._started = False

    def send_order(self, robot_id: str, order_id: str, waypoints: list) -> None:
        self.server.send(robot_id, order_message(robot_id, order_id, waypoints))

    @property
    def status(self) -> dict[str, Any]:
        return {
            "running": self._started,
            "host": self.server.host,
            "port": self.server.port,
            "robots_connected": sorted(self.server._writers.keys()),
        }
