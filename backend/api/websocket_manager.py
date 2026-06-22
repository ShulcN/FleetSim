from __future__ import annotations

import json

from fastapi import WebSocket


class WebSocketManager:
    def __init__(self):
        self._clients: set[WebSocket] = set()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self._clients.add(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        self._clients.discard(websocket)

    async def broadcast_json(self, data: dict) -> None:
        if not self._clients:
            return

        message = json.dumps(data)
        dead_clients = []

        for client in list(self._clients):
            try:
                await client.send_text(message)
            except Exception:
                dead_clients.append(client)

        for client in dead_clients:
            self.disconnect(client)
