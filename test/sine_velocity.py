import asyncio
import moteus
import math
import time

async def main():
    # Khoi tao doi tuong dieu khien (tu phat hien controller qua fdcanusb)
    c = moteus.Controller()  
    
    # Dam bao dung dong co truoc khi bat dau
    await c.set_stop()

    print("Running sine speed profile... (Ctrl+C to stop)")

    # Thong so song sin
    A = 0.5    # bien do van toc (rad/s)
    f = 0.1      # tan so (Hz)
    dt = 0.001    # chu ky dieu khien (s) ~100Hz

    t0 = time.time()

    while True:
        t = time.time() - t0
        velocity = A * math.sin(2 * math.pi * f * t)
        # Dat che do: velocity control
        await c.set_position(position=math.nan, velocity=velocity)
        await asyncio.sleep(dt)

asyncio.run(main())