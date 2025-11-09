import asyncio
import moteus
import math
import time
import json
import zmq
import random

async def main():
    c = moteus.Controller()
    await c.set_stop()
    print("Running random position control with stops... (Ctrl+C to stop)")

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")

    # Parameters for position control
    position_tolerance = 0.05 # Sai so chap nhan duoc de coi la da den vi tri (rad)
    move_speed_limit = 2.0    # Gioi han van toc khi di chuyen (rad/s)

    # Parameters for stop behavior
    min_stop_duration = 3.0 # Thoi gian dung toi thieu (s)
    max_stop_duration = 7.0 # Thoi gian dung toi da (s)
    min_random_pos = -2 * math.pi # Vi tri ngau nhien toi thieu (rad)
    max_random_pos = 2 * math.pi  # Vi tri ngau nhien toi da (rad)

    dt = 0.02 # Chu ky dieu khien (s)

    t_simulated = 0.0
    current_mode = "MOVING_TO_RANDOM_POS" # Cac che do: MOVING_TO_RANDOM_POS, STOPPED
    target_position = 0.0
    stop_start_time = 0.0
    stop_duration = 0.0

    # Khoi tao vi tri muc tieu dau tien
    target_position = random.uniform(min_random_pos, max_random_pos)
    print(f"Bat dau di chuyen den vi tri ngau nhien dau tien: {target_position:.2f} rad")

    while True:
        loop_start_time = time.time()
        t_simulated += dt

        position_command = target_position
        velocity_command = 0.0 # Moteus se tu tinh van toc de den target_position

        state = await c.set_position(
            position=position_command,
            velocity_limit=move_speed_limit, # Gioi han van toc khi di chuyen
            query=True
        )

        actual_position = state.values[moteus.Register.POSITION]
        actual_velocity = state.values[moteus.Register.VELOCITY]

        if current_mode == "MOVING_TO_RANDOM_POS":
            # Kiem tra xem da den vi tri muc tieu chua
            if abs(actual_position - target_position) < position_tolerance:
                current_mode = "STOPPED"
                stop_start_time = t_simulated
                stop_duration = random.uniform(min_stop_duration, max_stop_duration)
                print(f"Da den vi tri {target_position:.2f} rad. Dung trong {stop_duration:.2f}s")

        elif current_mode == "STOPPED":
            # Cho het thoi gian dung
            if (t_simulated - stop_start_time) >= stop_duration:
                current_mode = "MOVING_TO_RANDOM_POS"
                target_position = random.uniform(min_random_pos, max_random_pos)
                print(f"Het thoi gian dung. Bat dau di chuyen den vi tri moi: {target_position:.2f} rad")

        data = {
            'time': t_simulated,
            'command_position': position_command,
            'actual_position': actual_position,
            'command_velocity': actual_velocity, #Khong dieu khien van toc -> actual
            'actual_velocity': actual_velocity,
            'mode': current_mode
        }

        socket.send_string(json.dumps(data))

        time_spent_in_loop = time.time() - loop_start_time
        sleep_duration = dt - time_spent_in_loop

        if sleep_duration > 0:
            await asyncio.sleep(sleep_duration)
        else:
            print(f"Warning: Loop time exceeded dt! ({time_spent_in_loop:.4f}s)")
            await c.set_stop()

if __name__ == '__main__':
    asyncio.run(main())