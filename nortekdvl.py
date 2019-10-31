#!/usr/bin/env python3
"""
This script is used to integrate the nortek DVL1000 with ardupilot.
The DVL must be outputting messages $PNORBT6 to port 9000 of whoever runs this script.

There must also be a mavlink connection avaiable at 0.0.0.0:14551 so the script can get
ATTITUDE messages
"""

import asyncio
from pymavlink import mavutil
import math
import time
import socket
import sys

if sys.version_info[0] < 3 or sys.version_info[1] < 6:
    raise Exception("You must use Python >= 3.6 for async support!")

def cmd_set_home(lat, lon, alt):
    '''called when user selects "Set Home (with height)" on map'''
    print("Setting home to: ", lat, lon, alt)
    return master.mav.set_gps_global_origin_send(
        0,
        lat, # lat
        lon, # lon
        alt) # param7


# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14551', dialect="ardupilotmega")
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Set GPS origin ( This is the Blue Robotics parking lot)
cmd_set_home(338414762,-1183351514,1)

last_attitude = [0.0,0.0,0.0]
current_attitude = [0.0,0.0,0.0]
last_position = [0.0,0.0,0.0]
current_position = [0.0,0.0,0.0]

UDP_IP = "0.0.0.0"
UDP_PORT = 9000
sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)
last_time = None

def send_vision(x, y, z, rotation=[0,0,0], confidence=100):
    "Sends message VISION_POSITION_DELTA to flight controller"
    master.mav.vision_position_delta_send(
        0, # time
        125000, # delta time (us)
        rotation, # angle delta 
        [x, y, z], # position delta
        confidence) # confidence (0-100%)


def read_dvl_deltas():
    "Received dvl messages. they must have fields dx, dy, dz, and time"
    global last_time
    try:
        datagram, _ = sock.recvfrom(1024) # buffer size is 1024 bytes
    except:
        return None
    decoded = datagram.decode("UTF-8")
    parts = decoded.strip().split(",")
    data = dict()
    for item in parts[1:]:
        key, val = item.split("=")
        try:
            val = float(val)
        except ValueError:
            pass
        data[key.lower()] = val

    if data["fom"] > 0.5 or abs(data['vz']) > 10 or abs(data['vy']) > 10:
        #print("Inaccurate, rejecting")
        return None

    vx, vy, vz = data["vx"], data["vy"], data["vz"]
    current_time = data['time']
    if last_time is not None:
        dt = current_time - last_time
    else:
        dt = 0
    last_time = data["time"]

    print(dt)
    return dt*vx, dt*vy, dt*vz

async def receive_attitude():
    "Receive attitude messages from flight controller"
    global current_attitude
    while True:
        await asyncio.sleep(0.001)
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'ATTITUDE':
            msg = msg.to_dict()
            roll = msg['roll']
            pitch = msg['pitch']
            yaw = msg['yaw']
            current_attitude = [roll, pitch, yaw]


async def write_dvl():
    """every time a new dvl reading is available, calculate new angle deltas
    and sends to the flight controller
    """
    global last_attitude
    global current_attitude
    while True: 

        dvlData = read_dvl_deltas()
        if dvlData is None:
            await asyncio.sleep(0.01)   
            continue
        dx, dy, dz = dvlData
        angles = list(map(float.__sub__, current_attitude, last_attitude))
        angles[2] = angles[2] % (math.pi*2)
        send_vision(dx, dy, dz, list(angles))
        last_attitude = current_attitude
        await asyncio.sleep(0.01)

async def main():
    "Runs attitude and dvl tasks"
    task1 = asyncio.create_task(receive_attitude())
    task2 = asyncio.create_task(write_dvl())

    await asyncio.gather(task1, task2)

asyncio.run(main())
