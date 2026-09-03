#!/usr/bin/python3

import csv
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32MultiArray
import serial


class AckermannToSteeringSerial(Node):
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=115200):
        super().__init__('ackermann_to_steering_serial')

        self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=0.02)
        time.sleep(2.0)

        self.last_target = None
        self.last_send_time = 0.0
        self._last_debug_print_time = 0.0
        self.min_send_period = 0.02
        self.min_target_step = 0.2

        # latest commanded values
        self.last_cmd_target = 0.0
        self.last_cmd_rate = 0.0
        self.last_cmd_speed = 0.0
        self.last_cmd_acc = 0.0
        self.last_cmd_jerk = 0.0

        # latest RC-side debug values
        self.rc_steer_raw = 0.0
        self.rc_speed_raw = 0.0
        self.rc_target_rate = 0.0
        self.rc_manual_target = 0.0

        # latest board telemetry
        self.ino_t_ms = 0
        self.ino_target_deg = 0.0
        self.ino_target_slew_deg = 0.0
        self.ino_angle_deg = 0.0
        self.ino_error_deg = 0.0
        self.ino_delta_v = 0.0
        self.ino_va_out = 0.0
        self.ino_vb_out = 0.0
        self.ino_codeA = 0
        self.ino_codeB = 0
        self.ino_count = 0
        self.ino_pid_on = 0
        self.ino_p_term = 0.0
        self.ino_i_term = 0.0
        self.ino_d_term = 0.0
        self.ino_isr = 0
        self.ino_invalid = 0

        self.ackermann_subscriber = self.create_subscription(
            AckermannDrive, 'ackermann_cmd', self.ackermann_callback, 10
        )

        self.rc_debug_subscriber = self.create_subscription(
            Float32MultiArray, 'rc_steer_debug', self.rc_debug_callback, 10
        )

        self.feedback_publisher = self.create_publisher(
            Float32MultiArray, 'feedback_data', 10
        )

        self.create_timer(0.01, self.read_serial)   # 100 Hz serial polling
        self.create_timer(0.02, self.log_row)       # 50 Hz logging

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = open(f'steering_test_log_{ts}.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'pc_time',
            'rc_steer_raw',
            'rc_speed_raw',
            'rc_target_rate_deg_s',
            'rc_manual_target_deg',
            'cmd_target_deg',
            'cmd_target_rate_deg_s',
            'cmd_speed_m_s',
            'cmd_acc_m_s2',
            'cmd_jerk',
            'ino_t_ms',
            'ino_target_deg',
            'ino_target_slew_deg',
            'ino_angle_deg',
            'ino_error_deg',
            'ino_delta_v',
            'ino_va_out',
            'ino_vb_out',
            'ino_codeA',
            'ino_codeB',
            'ino_count',
            'ino_pid_on',
            'ino_p_term',
            'ino_i_term',
            'ino_d_term',
            'ino_isr',
            'ino_invalid',
        ])
        self.csv_file.flush()

        self.get_logger().info(f'Connected to steering board on {serial_port}')

        self.send_line("d")
        time.sleep(0.1)
        self.send_line("e")

    def send_line(self, line: str):
        msg = line.strip() + "\n"
        self.serial_connection.write(msg.encode('ascii', errors='ignore'))

    def ackermann_callback(self, msg: AckermannDrive):
        target = float(msg.steering_angle)
        target = max(-420.0, min(420.0, target))

        self.last_cmd_target = target
        self.last_cmd_rate = float(msg.steering_angle_velocity)
        self.last_cmd_speed = float(msg.speed)
        self.last_cmd_acc = float(msg.acceleration)
        self.last_cmd_jerk = float(msg.jerk)

        now = time.time()
        should_send = False

        if self.last_target is None:
            should_send = True
        elif abs(target - self.last_target) >= self.min_target_step:
            should_send = True
        elif (now - self.last_send_time) >= self.min_send_period:
            should_send = True

        if should_send:
            self.send_line(f"s{target:.2f}")

            if (now - self._last_debug_print_time) >= 0.10:
                self._last_debug_print_time = now
                self.get_logger().info(
                    f'Sent steering target: {target:.2f} deg, rate={self.last_cmd_rate:.2f} deg/s'
                )

            self.last_target = target
            self.last_send_time = now

    def rc_debug_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.rc_steer_raw = float(msg.data[0])
            self.rc_speed_raw = float(msg.data[1])
            self.rc_target_rate = float(msg.data[2])
            self.rc_manual_target = float(msg.data[3])

    def read_serial(self):
        try:
            while self.serial_connection.in_waiting > 0:
                line = self.serial_connection.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith("DATA,"):
                    parts = line.split(',')

                    if len(parts) >= 18:
                        try:
                            self.ino_t_ms = int(parts[1])
                            self.ino_target_deg = float(parts[2])
                            self.ino_target_slew_deg = float(parts[3])
                            self.ino_angle_deg = float(parts[4])
                            self.ino_error_deg = float(parts[5])
                            self.ino_delta_v = float(parts[6])
                            self.ino_va_out = float(parts[7])
                            self.ino_vb_out = float(parts[8])
                            self.ino_codeA = int(parts[9])
                            self.ino_codeB = int(parts[10])
                            self.ino_count = int(parts[11])
                            self.ino_pid_on = int(parts[12])
                            self.ino_p_term = float(parts[13])
                            self.ino_i_term = float(parts[14])
                            self.ino_d_term = float(parts[15])
                            self.ino_isr = int(parts[16])
                            self.ino_invalid = int(parts[17])

                            fb = Float32MultiArray()
                            fb.data = [
                                0.0,
                                0.0,
                                self.last_cmd_rate,
                                self.ino_angle_deg
                            ]
                            self.feedback_publisher.publish(fb)

                        except ValueError:
                            pass

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def log_row(self):
        self.csv_writer.writerow([
            time.time(),
            self.rc_steer_raw,
            self.rc_speed_raw,
            self.rc_target_rate,
            self.rc_manual_target,
            self.last_cmd_target,
            self.last_cmd_rate,
            self.last_cmd_speed,
            self.last_cmd_acc,
            self.last_cmd_jerk,
            self.ino_t_ms,
            self.ino_target_deg,
            self.ino_target_slew_deg,
            self.ino_angle_deg,
            self.ino_error_deg,
            self.ino_delta_v,
            self.ino_va_out,
            self.ino_vb_out,
            self.ino_codeA,
            self.ino_codeB,
            self.ino_count,
            self.ino_pid_on,
            self.ino_p_term,
            self.ino_i_term,
            self.ino_d_term,
            self.ino_isr,
            self.ino_invalid,
        ])
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.send_line("d")
            self.csv_file.flush()
            self.csv_file.close()
            self.serial_connection.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AckermannToSteeringSerial(serial_port='/dev/ttyUSB0', baud_rate=115200)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()