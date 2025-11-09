import asyncio
import moteus
import math
import time
import json
import zmq # Import ZeroMQ

async def main():
    c = moteus.Controller()
    await c.set_stop()
    print("Running sine speed profile... (Ctrl+C to stop)")

    # Khoi tao ZeroMQ publisher
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    # BIND toi mot cong tren container. Cong nay can duoc MAPPED ra ngoai may host.
    socket.bind("tcp://*:5556") # Chay tren cong 5556

    A = 0.5    # bien do van toc (rad/s)
    f = 0.1      # tan so (Hz)
    dt = 0.01    # chu ky dieu khien (s)

    t_simulated = 0.0     

    while True:
        loop_start_time = time.time() # Ghi lai thoi gian bat dau vong lap thuc te
        
        # Tang bien thoi gian "ly tuong" len dt
        t_simulated += dt 
        
        # Van toc lenh duoc tinh dua tren thoi gian "ly tuong" (de dam bao song sin muot)
        velocity_command = A * math.sin(2 * math.pi * f * t_simulated)
        
        # Gui lenh va nhan phan hoi tu dong co
        state = await c.set_position(
            position=math.nan, 
            velocity=velocity_command,
            query=True # Giu query=True de lay actual_velocity cho bieu do
        )
        
        # Lay van toc thuc te tu phan hoi cua dong co
        actual_velocity = state.values[moteus.Register.VELOCITY]
        actual_position = state.values[moteus.Register.POSITION]

        # Chuan bi du lieu de gui qua ZeroMQ
        data = {
            'time': t_simulated, # Gui t_simulated de bieu do van toc lenh duoc muot va dung thoi gian ly tuong
            'command_position': actual_position,
            'actual_position': actual_position,
            'command_velocity': velocity_command,
            'actual_velocity': actual_velocity
        }
        
        # Gui du lieu duoi dang JSON string
        socket.send_string(json.dumps(data))

        # Tinh toan thoi gian can ngu de duy tri chu ky dt
        time_spent_in_loop = time.time() - loop_start_time
        sleep_duration = dt - time_spent_in_loop

        if sleep_duration > 0:
            await asyncio.sleep(sleep_duration)
        else:
            await c.set_stop()
        pass 

if __name__ == '__main__':
    asyncio.run(main())