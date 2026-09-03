#!/usr/bin/python3

"""
data_capture.py
Author: Matthijs Steyerberg, Axel Barbelanne
Date: 22-10-2024

Modified for sterfBoard CSV debugging.
"""

import csv
from datetime import datetime
import threading
import time

import can
import rclpy
import serial
from ackermann_msgs.msg import AckermannDrive
from diagnostic_msgs.msg import KeyValue
from gps_msgs.msg import GPSFix
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class AckermannToVehicleNode(Node):
    def __init__(self, send_to_serial=True, is_esp32=False, serial_port='/dev/ttyACM2', baud_rate=115200):
        super().__init__('ackermann_to_vehicle_node')

        self.send_to_serial = send_to_serial
        self.GPS_speed = 0.0
        self.is_esp32 = is_esp32
        self.ack_signal_flag = 0
        self._last_cmd_print_time = 0.0
        self.serial_lock = threading.Lock()

        if send_to_serial:
            try:
                self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=0.02)

                if is_esp32:
                    self.serial_connection.dtr = False
                    self.serial_connection.rts = False
                    time.sleep(0.1)

                    self.serial_connection.dtr = True
                    self.serial_connection.rts = True
                    time.sleep(2)

                    self.serial_connection.dtr = False
                    self.serial_connection.rts = False

                    self.get_logger().info(f'Connected to ESP32 on {serial_port}')
                else:
                    self.get_logger().info(f'Connected to Arduino on {serial_port}')

            except serial.SerialException as e:
                self.get_logger().error(f'Failed to open serial port: {e}')
                rclpy.shutdown()
                return
        else:
            self.bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

        self.ackermann_subscriber = self.create_subscription(
            AckermannDrive, 'ackermann_cmd', self.ackermann_callback, 10
        )
        self.error_signal_subscriber = self.create_subscription(
            KeyValue, 'signal_error', self.signal_flag_callback, 10
        )
        self.gps_subscriber = self.create_subscription(
            GPSFix, 'gps_raw', self.gps_callback, 10
        )

        self.serial_data_publisher = self.create_publisher(Float32MultiArray, 'feedback_data', 10)

        if self.send_to_serial:
            self.create_timer(0.02, self.read_from_serial_port)   # 50 Hz
            self.heartbeat_interval = 0.2
            self.heartbeat_timer = self.create_timer(self.heartbeat_interval, self.send_heartbeat)
        else:
            self.create_timer(0.1, self.read_from_serial_port)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.sterf_dbg_file = open(f'sterf_debug_{ts}.csv', 'w', newline='')
        self.sterf_dbg_writer = csv.writer(self.sterf_dbg_file)
        self.sterf_dbg_writer.writerow([
            'pc_time',
            't_ms',
            'hbSignal',
            'checkedMode',
            'serialHbOk',
            'noAck',
            'ackAgeMs',
            'steerCmdRx',
            'steerRateRx',
            'targetJump',
            'steerCmdCan',
            'steerRateCan',
            'steerFb',
            'steerRateFb',
            'axelAlive',
            'steeringAlive',
            'brakeAlive',
            'serialCmdCount',
            'serialHbCount',
            'canSteerTxCount'
        ])
        self.sterf_dbg_file.flush()

    def ackermann_callback(self, msg):
        steering_angle = msg.steering_angle
        steering_angle_velocity = msg.steering_angle_velocity
        velocity = msg.speed
        acceleration = msg.acceleration
        jerk = msg.jerk
        gps_speed = self.GPS_speed

        now = time.time()
        if (now - self._last_cmd_print_time) >= 0.10:
            self._last_cmd_print_time = now
            self.get_logger().info(
                f'CMD steer={steering_angle:.2f} deg, '
                f'steer_rate={steering_angle_velocity:.2f} deg/s, '
                f'vel={velocity:.2f} m/s, '
                f'acc={acceleration:.2f}, '
                f'gps={gps_speed:.2f}'
            )

        if self.send_to_serial:
            message = f"{steering_angle},{steering_angle_velocity},{velocity},{acceleration},{jerk},{gps_speed}"
            self.send_to_serial_port(message)
        else:
            self.send_to_can_bus(steering_angle, steering_angle_velocity, velocity, acceleration, jerk)

    def gps_callback(self, msg):
        self.GPS_speed = msg.speed

    def signal_flag_callback(self, msg):
        assert (msg.key == "0" or msg.key == "1" or msg.key == "2"), \
            f"Received unexpected key in error message: {msg.key}"
        self.ack_signal_flag = int(msg.key)

    def send_to_can_bus(self, steering_angle, steering_angle_velocity, velocity, acceleration, jerk):
        steering_angle_data = int(steering_angle * 1000)
        steering_angle_velocity_data = int(steering_angle_velocity * 1000)
        velocity_data = int(velocity * 1000)
        acceleration_data = int(acceleration * 1000)
        jerk_data = int(jerk * 1000)

        can_id = 0x123
        can_data = [
            (steering_angle_data >> 8) & 0xFF,
            steering_angle_data & 0xFF,
            (steering_angle_velocity_data >> 8) & 0xFF,
            steering_angle_velocity_data & 0xFF,
            (velocity_data >> 8) & 0xFF,
            velocity_data & 0xFF,
            (acceleration_data >> 8) & 0xFF,
            acceleration_data & 0xFF,
            (jerk_data >> 8) & 0xFF,
            jerk_data & 0xFF
        ]

        message = can.Message(arbitration_id=can_id, data=can_data[:8], is_extended_id=False)
        try:
            self.bus.send(message)
            self.get_logger().info(
                f'Sent CAN message: Steering={steering_angle}, Velocity={velocity}, Acc={acceleration}, Jerk={jerk}'
            )
        except can.CanError:
            self.get_logger().error('Failed to send CAN message')

    def send_to_serial_port(self, message):
        message = "<" + message + ">\n"
        try:
            with self.serial_lock:
                self.serial_connection.write(message.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send serial message: {e}')

    def read_from_serial_port(self):
        if not self.send_to_serial:
            return

        try:
            while self.serial_connection.in_waiting > 0:
                data = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                if not data:
                    continue

                if data.startswith("DBG_HEADER,"):
                    continue

                if data.startswith("DBG,"):
                    parts = data.split(',')
                    if len(parts) == 20:
                        self.sterf_dbg_writer.writerow([time.time()] + parts[1:])
                        self.sterf_dbg_file.flush()
                    continue

                if not data.startswith("<"):
                    continue

                parsed_data = self.parse_serial_data(data)

                if parsed_data:
                    msg = Float32MultiArray()
                    msg.data = [
                        parsed_data['velocity_ctrl'],
                        parsed_data['velocity_fb'],
                        parsed_data['steering_rate_ctrl'],
                        parsed_data['steering_angle_fb']
                    ]
                    self.serial_data_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error reading from serial port: {e}')

    def parse_serial_data(self, data):
        try:
            parts = data.strip('<>').split(',')
            if len(parts) != 4:
                return None

            velocity_ctrl = float(parts[0])
            velocity_fb = float(parts[1])
            steering_rate_ctrl = float(parts[2])
            steering_angle_fb = float(parts[3])

            return {
                'velocity_ctrl': velocity_ctrl,
                'velocity_fb': velocity_fb,
                'steering_rate_ctrl': steering_rate_ctrl,
                'steering_angle_fb': steering_angle_fb,
            }
        except ValueError:
            return None

    def send_heartbeat(self):
        heartbeat_message = f"H,{int(self.ack_signal_flag)}"
        self.send_to_serial_port(heartbeat_message)

    def destroy_node(self):
        try:
            if hasattr(self, 'sterf_dbg_file'):
                self.sterf_dbg_file.flush()
                self.sterf_dbg_file.close()
        except Exception:
            pass

        try:
            if self.send_to_serial and hasattr(self, 'serial_connection'):
                self.serial_connection.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    try:
        rclpy.init(args=args)

        send_to_serial = '--serial' in args if args else False
        send_to_serial = True
        serial_port = '/dev/ttyUSB0'   # must be sterfBoard port
        baud_rate = 115200

        node = AckermannToVehicleNode(
            send_to_serial=send_to_serial,
            serial_port=serial_port,
            baud_rate=baud_rate
        )
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Shutting down.')
        if node.send_to_serial:
            node.send_to_serial_port('E')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)