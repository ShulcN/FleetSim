import asyncio

from .simulation_engine import SimulationEngine


async def run_simulation_loop(engine: SimulationEngine, broadcast_callback):
    """Run simulation forever and broadcast snapshots."""
    while True:
        await engine.step()
        await broadcast_callback(await engine.snapshot())
        dt = engine.settings.dt
        rtf = max(0.01, engine.settings.realtime_factor)
        await asyncio.sleep(dt / rtf)
