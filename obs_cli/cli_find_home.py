import time
import zmq
import numpy as np

def find_el_home(sys_manager):
    print("SET ELEVATION HOME ROUTINE")
    print("--------------------------")
    sys_manager.controller.set_mode("manual")
    input("Move to approx home and press enter")
    print("Now ensure Joystick value is zero")


    context = zmq.Context()

    pos_socket = context.socket(zmq.SUB)
    pos_socket.setsockopt(zmq.CONFLATE, 1)
    pos_socket.connect("tcp://scoti.local:60000")
    pos_socket.subscribe("H") # Position messages only (no home)

    speed_set_socket = context.socket(zmq.REQ)
    speed_set_socket.setsockopt(zmq.LINGER, 0) # Don't linger after socket closed
    speed_set_socket.connect("tcp://scoti.local:60004")

    def close_sockets():
        pos_socket.close()
        speed_set_socket.close()
        context.term()

    current_el = sys_manager.position_monitor.az_el[1]

    sys_manager.controller.set_mode("setpoint")
    sys_manager.controller.set_setpoint(0, current_el-10)

    while np.abs(sys_manager.position_monitor.az_el[1] - (current_el - 10)) > 0.01:
        time.sleep(0.05)

    print("Sweeping to find home position...")

    sys_manager.controller.set_mode("speed_set")
    sys_manager.controller.set_speed(0, 1)

    try:
        recv = None
        while recv is None:
            assert(sys_manager.position_monitor.az_el[1] < (current_el + 10))
            try:
                recv = pos_socket.recv_string(zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.02)
        home_pos = float(recv.split(",")[1][1:])
    except Exception as e:
        print("Timeout or error waiting for home switch")
        print(e)
        sys_manager.controller.set_speed(0, 0)
        sys_manager.controller.set_mode("setpoint")
        close_sockets()
        return

    print("Received home switch signal at ", home_pos)
    sys_manager.controller.set_mode("setpoint")
    offset = 1.8
    sys_manager.controller.set_setpoint(0, home_pos - offset)
    print("Moving to new home position...")
    
    while np.abs(sys_manager.position_monitor.az_el[1] - (home_pos - offset)) > 0.02:
        time.sleep(0.05)

    print("Arrived at new home position")

    speed_set_socket.send_string("H,E") # Set home
    time.sleep(0.5)
    try:
        assert(speed_set_socket.recv_string() == "OK")
    except:
        print("Timeout or error setting home position")
        close_sockets()
        return

    close_sockets()

    sys_manager.controller.set_setpoint(0, 0)
    print("Home position set sucessfully")



