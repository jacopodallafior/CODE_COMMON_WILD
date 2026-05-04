#!/usr/bin/python3

"""
mavlink_classes.py
Author: Matthijs Steyerberg, Axel Barbelanne
Date: 22-10-2024

Script to process and store incoming MAVLink messages in helper classes representing relevant telemetry data.
These classes interpret various MAVLink message types to provide organized data for further processing and analysis.
Classes mirror MAVLink message structures for consistency and ease of use.

Functions:
    - println(statement): Prints a statement with context based on the script's execution mode.
    - calc_yaw(q): Calculates and returns the yaw (heading) angle from a quaternion.

Classes:
    - HEARTBEAT: Parses MAVLink heartbeat messages, storing vehicle mode, type, and arm status.
    - ATTITUDE: Stores attitude information (roll, pitch, yaw, rates) from MAVLink attitude messages.
    - GPS_RAW_INT: Extracts and converts raw GPS data, including coordinates, altitude, and accuracy values.
    - GPS_RTK: (Testing required) Stores GPS RTK data, including rate, satellite count, and base accuracy.
    - VFR_HUD: Parses VFR data for ground speed, heading, and climb rate from MAVLink HUD messages.
    - GLOBAL_POSITION_INT: Retrieves global position and velocity data from MAVLink messages.
    - HOME_POSITION: Records home position coordinates and altitude for navigation purposes.
    - SCALED_IMU: Converts and stores scaled IMU data, including acceleration, gyro, and magnetic field values.
    - RC_CHANNELS_RAW: Processes raw RC channel inputs and signal strength indicators.
    - RC_CHANNELS_SCALED: (Currently unused) Scales RC channel outputs to represent control inputs.
    - HIGHRES_IMU: (Currently unused) Stores high-resolution IMU data, including pressure and temperature readings.
    - UTM_GLOBAL_POSITION: (Currently unused) Parses UTM coordinates and position details.
    - GPS_STATUS: (Currently unavailable) Processes GPS satellite data for visibility and signal strength.
    - ODOMETRY: (Currently unused) Stores odometry data, including position, velocity, and orientation.
    - LOCAL_POSITION_NED: (Currently unused) Retrieves local position data in NED coordinates.
"""

import math
import numpy as np
from pymavlink import mavutil


def println(statement):
    """Prints a statement, with module context if called externally."""
    if __name__ == "__main__":
        print(statement)
    else:
        print(f"[{__name__}]: {statement}")


def calc_yaw(q):
    """Calculates the yaw angle from a quaternion."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


class HEARTBEAT:
    """Parses MAVLink heartbeat messages and stores vehicle mode, type, and arm status."""
    def __init__(self, msg):
        println("Received MAVlink heartbeat")
        self.mode = mavutil.mode_string_v10(msg)
        self.mav_type = msg.type
        self.base_mode = msg.base_mode
        self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        self.mav_autopilot = msg.autopilot


class ATTITUDE:
    """Stores attitude information including roll, pitch, yaw, and angular rates."""
    def __init__(self, msg):
        self.time_boot_ms = msg.time_boot_ms
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.dip = math.atan2(math.sin(msg.pitch), math.cos(msg.roll))
        self.yaw = msg.yaw
        self.rollspeed = msg.rollspeed
        self.pitchspeed = msg.pitchspeed
        self.yawspeed = msg.yawspeed


class GPS_RAW_INT:
    """Extracts and converts raw GPS data, including coordinates, altitude, and accuracy values."""
    def __init__(self, msg):
        self.time_usec = msg.time_usec
        self.fix_type = msg.fix_type
        conversion_map = {0: -1, 1: -1, 2: 0, 3: 0, 4: 18, 5: 20, 6: 19}
        self.status = conversion_map.get(self.fix_type)
        self.lat = msg.lat / 1e7
        self.lon = msg.lon / 1e7
        self.alt = msg.alt / 1e3
        self.eph = float(msg.eph / 100)
        self.epv = float(msg.epv / 100)
        self.vel = float(msg.vel / 100)
        self.cog = float(msg.cog / 100)
        self.satellites_visible = msg.satellites_visible
        self.h_acc = float(msg.h_acc / 1000) if hasattr(msg, 'h_acc') else 0.
        self.v_acc = float(msg.v_acc / 1000) if hasattr(msg, 'v_acc') else 0.
        self.vel_acc = float(msg.vel_acc / 1000) if hasattr(msg, 'vel_acc') else 0.
        self.hdg_acc = float(msg.hdg_acc / 10000) if hasattr(msg, 'hdg_acc') else 0.
        self.yaw = float(msg.yaw / 100) if hasattr(msg, 'yaw') else 0.


class GPS_RTK:
    """Stores GPS RTK data, including rate, satellite count, and base accuracy."""
    def __init__(self, msg):
        self.rtk_rate = msg.rtk_rate
        self.nsats = msg.nsats
        self.base_acc = msg.accuracy


class VFR_HUD:
    """Parses VFR data for ground speed, heading, and climb rate."""
    def __init__(self, msg):
        self.groundspeed = msg.groundspeed
        self.heading = msg.heading
        self.climb = msg.climb


class GLOBAL_POSITION_INT:
    """Retrieves global position and velocity data from MAVLink messages."""
    def __init__(self, msg):
        self.time_boot_ms = msg.time_boot_ms
        self.lat = msg.lat / 1e7
        self.lon = msg.lon / 1e7
        self.alt = msg.alt / 1e3
        self.relative_alt = msg.relative_alt
        self.vx = msg.vx / 100
        self.vy = msg.vy / 100
        self.vz = msg.vz / 100
        self.hdg = msg.hdg / 100


class HOME_POSITION:
    """Records home position coordinates and altitude for navigation purposes."""
    def __init__(self, msg):
        self.lat = msg.latitude / 1e7
        self.lon = msg.longitude / 1e7
        self.alt = msg.altitude / 1e3
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.q = msg.q


class SCALED_IMU:
    """Converts and stores scaled IMU data, including acceleration, gyro, and magnetic field values."""
    def __init__(self, msg):
        self.xacc = msg.xacc / 1000 * 9.8066
        self.yacc = msg.yacc / 1000 * 9.8066
        self.zacc = msg.zacc / 1000 * 9.8066
        self.xgyro = msg.xgyro
        self.ygyro = msg.ygyro
        self.zgyro = msg.zgyro
        self.xmag = msg.xmag
        self.ymag = msg.ymag
        self.zmag = msg.zmag


class RC_CHANNELS_RAW:
    """Processes raw RC channel inputs and signal strength indicators."""
    def __init__(self, msg):
        self.time_boot_ms = msg.time_boot_ms
        self.chan1 = msg.chan1_raw
        self.chan2 = msg.chan2_raw
        self.chan3 = msg.chan3_raw
        self.chan4 = msg.chan4_raw
        self.chan5 = msg.chan5_raw
        self.rssi = msg.rssi
        self.dbm = (self.rssi / 1.9) - 127


class RC_CHANNELS_SCALED:
    """Scales RC channel outputs between [-10000,10000], where 0 indicates no control input."""
    def __init__(self, msg):
        self.time_boot_ms = msg.time_boot_ms
        self.chan1 = msg.chan1_scaled
        self.chan2 = msg.chan2_scaled
        self.chan3 = msg.chan3_scaled
        self.chan4 = msg.chan4_scaled
        self.chan5 = msg.chan5_scaled
        self.rssi = msg.rssi
        self.dbm = (self.rssi / 1.9) - 127


class HIGHRES_IMU:
    """Stores high-resolution IMU data, including pressure and temperature readings."""
    def __init__(self, msg):
        self.time_usec = msg.time_usec
        self.xacc = msg.xacc
        self.yacc = msg.yacc
        self.zacc = msg.zacc
        self.xgyro = msg.xgyro
        self.ygyro = msg.ygyro
        self.zgyro = msg.zgyro
        self.xmag = msg.xmag
        self.ymag = msg.ymag
        self.zmag = msg.zmag
        self.abs_pressure = msg.abs_pressure
        self.diff_pressure = msg.diff_pressure
        self.temperature = msg.temperature


class UTM_GLOBAL_POSITION:
    """Parses UTM coordinates and position details."""
    def __init__(self, msg):
        self.time = msg.time
        self.uas_id = msg.uas_id
        self.lat = msg.lat / 1e7
        self.lon = msg.lon / 1e7
        self.alt = msg.alt / 1e3
        self.relative_alt = msg.relative_alt
        self.vx = msg.vx / 100
        self.vy = msg.vy / 100
        self.vz = msg.vz / 100
        self.hdg = msg.hdg / 100
        self.h_acc = msg.h_acc
        self.v_acc = msg.v_acc
        self.vel_acc = msg.vel_acc
        self.update_rate = msg.update_rate
        self.flags = msg.flags


class GPS_STATUS:
    """Processes GPS satellite data for visibility and signal strength."""
    def __init__(self, msg):
        self.satellites_visible = msg.satellites_visible
        self.satellite_prn = msg.satellite_prn
        self.satellite_used = msg.satellite_used
        self.satellite_elevation = msg.satellite_elevation
        self.satellite_azimuth = msg.satellite_azimuth
        self.satellite_snr = msg.satellite_snr


class ODOMETRY:
    """Stores odometry data, including position, velocity, and orientation."""
    def __init__(self, msg):
        self.time_usec = msg.time_usec
        self.frame_id = msg.frame_id
        self.child_frame_id = msg.child_frame_id
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.q = msg.q
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz
        self.rollspeed = msg.rollspeed
        self.pitchspeed = msg.pitchspeed
        self.yawspeed = msg.yawspeed
        self.pose_covariance = msg.pose_covariance
        self.velocity_covariance = msg.velocity_covariance
        self.reset_counter = msg.reset_counter
        self.estimator_type = msg.estimator_type
        self.quality = msg.quality

    def get_yaw(self):
        """Returns the yaw angle derived from the quaternion."""
        return calc_yaw(self.q)

    def get_abs_vel(self):
        """Calculates and returns the absolute velocity."""
        return np.sqrt(self.vx ** 2 + self.vy ** 2 + self.vz ** 2)


class LOCAL_POSITION_NED:
    """Retrieves local position data in NED coordinates."""
    def __init__(self, msg):
        self.time_boot_ms = msg.time_boot_ms
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz
