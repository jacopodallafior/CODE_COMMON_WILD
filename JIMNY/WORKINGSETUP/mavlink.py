#!/usr/bin/python3

"""
mavlink.py
Author: Matthijs Steyerberg, Axel Barbelanne
Date: 22-10-2024

Script for serial communication with CubePilot, receiving MAVLink messages and storing telemetry data.
The CubeSerial class establishes a MAVLink connection, sets message frequencies, and retrieves data for
specific message types to provide real-time information on vehicle status, GPS, attitude, and control signals.

Functions:
    - println(statement): Prints a statement with module context if called externally.
    - probably_vehicle_heartbeat(msg): Determines if a message likely represents a vehicle heartbeat.

Classes:
    - CubeSerial: Manages serial communication with CubePilot and retrieves MAVLink messages for telemetry.
"""

from pymavlink import mavutil
import time
import serial
import serial.tools.list_ports as port_list
from cuberos_py.mavlink_classes import HEARTBEAT, ATTITUDE, GPS_RAW_INT, GPS_RTK, VFR_HUD, GLOBAL_POSITION_INT, HOME_POSITION, SCALED_IMU, RC_CHANNELS_RAW


def println(statement):
    """Prints a statement, with module context if called externally."""
    if __name__ == "__main__":
        print(statement)
    else:
        print(f"[{__name__}]: {statement}")


def probably_vehicle_heartbeat(msg):
    """Determines if a message likely represents a vehicle heartbeat."""
    if msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_GIMBAL:
        return False
    if msg.type in (mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_TYPE_GIMBAL,
                    mavutil.mavlink.MAV_TYPE_ADSB,
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER):
        return False
    if msg.autopilot in frozenset([
            mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ]):
        return False
    return True


class CubeSerial:
    """Communicates with CubePilot to retrieve information via the MAVLink interface."""
    def __init__(self, serial_port, baud_rate=115200):
        println("Initializing Orange Cube connection")
        println("Starting MAVLink connection...")
        try:
            self.cube = mavutil.mavlink_connection(serial_port, baud_rate)
        except serial.SerialException as e:
            println(f"Serial port: {serial_port} not available...")
            println(e)
            exit()
        
        self.cube.wait_heartbeat()
        self.rc_freq = 50
        self.set_message_interval(65, self.rc_freq)  # RC_CHANNELS message ID is 65
        println("Received first MAVlink heartbeat")

        # Initialize message storage variables
        self.heartbeat = None
        self.attitude = None
        self.gps_raw_int = None
        self.gps_status = None
        self.gps_rtk = None
        self.gps_raw_int_freq = 10  # Hz
        self.vfr_hud = None
        self.global_position_int = None
        self.global_position_int_freq = 10  # Hz
        self.rc = None
        #self.rc_freq = 20  # Hz
        self.highress_imu = None
        self.imu = None
        self.imu_freq = 20  # Hz

        self.required_satellites = 4
        self.t = time.time()

    def set_message_interval(self, message_id, frequency_hz):
        """Sets the interval at which a specific message type is requested from the CubePilot."""
        interval_us = int(1e6 / frequency_hz)
        self.cube.mav.command_long_send(
            self.cube.target_system,
            self.cube.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0, 0, 0, 0, 0
        )
        println(f"Set message interval for message ID {message_id} to {frequency_hz} Hz")

    def get_messages(self):
        """Retrieves messages from the CubePilot and stores them in corresponding class attributes."""
        msg = self.cube.recv_match(type=["ATTITUDE", "GPS_RAW_INT", "GPS_STATUS", "GPS_RTK", "HOME_POSITION",
                                         "LOCAL_POSITION_NED", "SCALED_IMU3", "GLOBAL_POSITION_INT",
                                         "RC_CHANNELS", "HEARTBEAT"], blocking=False)
        if msg is None:
            println("No message received")
            return

        try:
            message_actions = {
                "HEARTBEAT": lambda msg: setattr(self, 'heartbeat', HEARTBEAT(msg)) if probably_vehicle_heartbeat(msg) else None,
                "ATTITUDE": lambda msg: setattr(self, 'attitude', ATTITUDE(msg)),
                "GPS_RAW_INT": lambda msg: (setattr(self, 'gps_raw_int', GPS_RAW_INT(msg)),
                                            println(f"-- Failed to find enough satellites ({msg.satellites_visible}/{self.required_satellites})")
                                            if (msg.satellites_visible < self.required_satellites) else None),
                "GPS_RTK": lambda msg: setattr(self, 'gps_rtk', GPS_RTK(msg)),
                "VFR_HUD": lambda msg: setattr(self, 'vfr_hud', VFR_HUD(msg)),
                "GLOBAL_POSITION_INT": lambda msg: setattr(self, 'global_position_int', GLOBAL_POSITION_INT(msg)),
                "HOME_POSITION": lambda msg: setattr(self, 'home_position', HOME_POSITION(msg)),
                "SCALED_IMU3": lambda msg: setattr(self, 'imu', SCALED_IMU(msg)),
                "RC_CHANNELS": lambda msg: setattr(self, 'rc', RC_CHANNELS_RAW(msg))
            }

            msg_type = msg.get_type()
            if msg_type == "GPS_INPUT":
                println(msg_type)
            if msg_type in message_actions:
                message_actions[msg_type](msg)

        except AttributeError as error:
            println(f"Attribute error: {error}")


if __name__ == "__main__":
    print("COM ports available:")
    ports = list(port_list.comports())
    for p in ports:
        print(p)

    try:
        cube_port = "/dev/ttyACM0"
        cube = CubeSerial(cube_port, 115200)
    except serial.SerialException as e:
        print(f"Serial port: {cube_port} unavailable...")
        print(e)
        exit()

    # Init a first round
    cube.get_messages()

    t = time.time()

    while True:
        dt = time.time() - t
        cube.get_messages()


        
        if dt > 1.:
            print(f'Mode: {cube.heartbeat.mode}') #print(f'Mode: {cube.mode}') 

            if cube.gps_raw_int is None:
                print("Failed to receive GPS...")
            else:
                print(f'Lat: {cube.gps_raw_int.lat}')
                print(f'Lon: {cube.gps_raw_int.lon}')
                print(f'Alt: {cube.gps_raw_int.alt}')

            if cube.attitude is None:
                print("Failed to receive orientation...")
            else:
                print(f'Yaw: {cube.attitude.yaw}')

            if cube.rc is None:
                print("Failed to receive RC")
            else:
                print(f'Chan1: {cube.rc.chan1}')
                print(f'Chan2: {cube.rc.chan2}')
                print(f'Chan3: {cube.rc.chan3}')
                print(f'Chan4: {cube.rc.chan4}')
                print(f'Chan5: {cube.rc.chan5}')

            t = time.time()
