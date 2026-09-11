import asyncio
from time import monotonic


async def run_simulation_loop(engine, broadcast_callback):
    """Fixed physics dt, wall-clock accumulation, bounded batches and 10 Hz UI."""
    previous = monotonic()
    last_broadcast = previous - 1
    measured_at, measured_time = previous, engine.sim_time
    accumulator = 0.0
    last_mode = None
    while True:
        now = monotonic()
        mode = (engine.status, engine.settings.realtime_factor, engine.recording.id)
        if mode != last_mode:
            accumulator = 0.0
            measured_at, measured_time = now, engine.sim_time
            last_mode = mode
        if engine.status == "running":
            accumulator = min(
                accumulator + (now - previous) * engine.settings.realtime_factor,
                2 * engine.settings.realtime_factor,
            )
        else:
            accumulator = 0.0
            engine.actual_realtime_factor = 0.0
        previous = now
        batch_start = now
        steps = 0
        while (
            accumulator >= engine.settings.dt
            and engine.status == "running"
            and steps < 40
        ):
            await engine.step()
            accumulator -= engine.settings.dt
            steps += 1
            if monotonic() - batch_start > 0.025:
                break
        now = monotonic()
        if now - measured_at >= 0.5:
            engine.actual_realtime_factor = max(
                0, (engine.sim_time - measured_time) / (now - measured_at)
            )
            measured_at, measured_time = now, engine.sim_time
        if now - last_broadcast >= 0.1:
            await broadcast_callback(await engine.snapshot())
            last_broadcast = now
        await asyncio.sleep(0.001 if engine.status == "running" else 0.03)
