"""
data_capture.py
Author: Matthijs Steyerberg, Axel Barbelanne
Date: 22-10-2024

data_capture.py
Author: Matthijs Steyerberg, Axel Barbelanne
Date: 22-10-2024

Node to send and receive required data to the vehicle. 
This message contains an Ackermann_msg and GPS speed:
- steering_wheel_angle
- steering_wheel_rate
- velocity
- acceleration
- jerk
- GPS_speed
The vehicle is requisted to send back the current:
- steering angle
- throttle percentage
- brake percentage
- drive mode (forwards/backwards)
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import can
import serial
import time
from gps_msgs.msg import GPSFix
import threading
from diagnostic_msgs.msg import KeyValue


class AckermannToVehicleNode(Node):
    def __init__(self, send_to_serial=True, is_esp32 = False, serial_port='/dev/ttyACM2', baud_rate=115200):
        super().__init__('ackermann_to_vehicle_node')

        # Flag to determine if we're sending via CAN bus or Serial port
        self.send_to_serial = send_to_serial
        self.GPS_speed = 0
        self.is_esp32 = is_esp32

        if send_to_serial:
            # Initialize serial communication with Arduino/ESP32 to send commands
            try:
                self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1)
                
                if is_esp32:
                    # Reset ESP32 by toggling DTR and RTS
                    self.serial_connection.dtr = False  # Pull DTR low
                    self.serial_connection.rts = False  # Pull RTS low
                    time.sleep(0.1)  # Short delay to ensure reset

                    self.serial_connection.dtr = True  # Pull DTR high (release reset)
                    self.serial_connection.rts = True  # Pull RTS high (release reset)
                    time.sleep(2)  # Wait for ESP32 to reboot

                    # Set DTR and RTS low again to finish reset process
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
            # Initialize the CAN bus (using virtual interface 'vcan0' for testing, use your actual interface)
            self.bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

        # Subscriber to AckermannDrive message
        self.ackermann_subscriber = self.create_subscription(AckermannDrive, 'ackermann_cmd', self.ackermann_callback, 10)

        self.error_signal_subscriber = self.create_subscription(KeyValue, 'signal_error', self.signal_flag_callback, 10)

        # Subscriber to GPS data
        self.gps_subscriber = self.create_subscription(GPSFix, 'gps_raw', self.gps_callback, 10)

        self.serial_data_publisher = self.create_publisher(Float32MultiArray, 'feedback_data', 10)

        # Timer to read serial data back from the Arduino
        if self.send_to_serial:
            self.create_timer(0.2, self.read_from_serial_port) 
            # Start a timer to send heartbeat messages
            self.heartbeat_interval = 0.2  # seconds
            self.heartbeat_timer = threading.Timer(self.heartbeat_interval, self.send_heartbeat)
            self.heartbeat_timer.start()

        else:
            self.create_timer(0.1, self.read_from_serial_port)  # Change this to use CAN instead if support is added

        # Flag to determine if the validity of the ackermann signal (0: disarmed, 1: armed, valid, 2: armed and invalid)
        # Start as disarmed
        self.ack_signal_flag = 0


    def ackermann_callback(self, msg):
        # Extract the required data
        steering_angle = msg.steering_angle
        steering_angle_velocity = msg.steering_angle_velocity
        velocity = msg.speed
        acceleration = msg.acceleration
        jerk = msg.jerk
        gps_speed = self.GPS_speed

        if self.send_to_serial:
            # Construct the message
            message = f"{steering_angle},{steering_angle_velocity},{velocity},{acceleration},{jerk},{gps_speed}"
            self.send_to_serial_port(message)
        else:
            self.send_to_can_bus(steering_angle, steering_angle_velocity, velocity, acceleration, jerk)

    def gps_callback(self, msg):
        print(f"received GPS, speed: {msg.speed}")
        self.GPS_speed = msg.speed

    def signal_flag_callback(self, msg):
        """
        Callback to update the ackermann signal validity based on the key (1: error, 0: ok).
        """
        assert (msg.key == "0" or msg.key == "1" or msg.key == "2"), f"Received unexpected key in error message: {msg.key}"
        self.ack_signal_flag = int(msg.key)


    def send_to_can_bus(self, steering_angle, steering_angle_velocity, velocity, acceleration, jerk):
        """
        Send the data to the CAN bus
        """
        # Pack the data into a CAN message (example uses 16-bit representation for each value)
        steering_angle_data = int(steering_angle * 1000)              # Scale to fit in CAN message
        steering_angle_velocity_data = int(steering_angle_velocity * 1000)  # Scale to fit in CAN message
        velocity_data = int(velocity * 1000)                          # Scale to fit in CAN message
        acceleration_data = int(acceleration * 1000)                  # Scale to fit in CAN message
        jerk_data = int(jerk * 1000)                                  # Scale to fit in CAN message

        # Construct the CAN message (you may split data into multiple messages based on your CAN protocol)
        can_id = 0x123  # Example CAN ID, modify as needed
        can_data = [
            (steering_angle_data >> 8) & 0xFF,          # Steering angle high byte
            steering_angle_data & 0xFF,                 # Steering angle low byte
            (steering_angle_velocity_data >> 8) & 0xFF, # Steering angle velocity high byte
            steering_angle_velocity_data & 0xFF,        # Steering angle velocity low byte
            (velocity_data >> 8) & 0xFF,                # Velocity high byte
            velocity_data & 0xFF,                       # Velocity low byte
            (acceleration_data >> 8) & 0xFF,            # Acceleration high byte
            acceleration_data & 0xFF,                   # Acceleration low byte
            (jerk_data >> 8) & 0xFF,                    # Jerk high byte
            jerk_data & 0xFF                            # Jerk low byte
        ]

        # Create a CAN message
        message = can.Message(arbitration_id=can_id, data=can_data[:8], is_extended_id=False)  # Send first 8 bytes
        try:
            self.bus.send(message)
            self.get_logger().info(f'Sent CAN message: Steering={steering_angle}, Velocity={velocity}, Acc={acceleration}, Jerk={jerk}')
        except can.CanError:
            self.get_logger().error('Failed to send CAN message')

    def send_to_serial_port(self, message):
        message = "<" + message + ">\n"
        # Format: <steering_angle,steering_angle_velocity,velocity,acceleration,jerk,GPS_speed>
        try:
            self.serial_connection.write(message.encode())
            self.get_logger().info(f'Sent Serial message: {message.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send serial message: {e}')

    def read_from_serial_port(self):
        if self.serial_connection.in_waiting > 0:
            try:
                data = self.serial_connection.readline().decode('utf-8').strip()  # Read a line and decode it
                self.get_logger().info(f' Serial message received: {data}')
                parsed_data = self.parse_serial_data(data)

                if parsed_data:
                    # Publish the parsed data
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
            # Parse the data into three floats
            parts = data.strip('<>').split(',')
            if len(parts) != 4:
                self.get_logger().warn(f'Unexpected data format: {data}')
                return None
            
            # Convert parts to floats
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
        except ValueError as e:
            self.get_logger().warn(f'Error parsing serial data: {data}, {e}')
            return None

    def send_heartbeat(self):
        # Send a heartbeat message
        heartbeat_message = f"H,{int(self.ack_signal_flag)}"
        self.send_to_serial_port(heartbeat_message)
        # Restart the timer
        self.heartbeat_timer = threading.Timer(self.heartbeat_interval, self.send_heartbeat)
        self.heartbeat_timer.start()

    def destroy_node(self):
        # Cancel heartbeat timer
        if hasattr(self, 'heartbeat_timer'):
            self.heartbeat_timer.cancel()
        super().destroy_node()


def main(args=None):
    try:
        rclpy.init(args=args)

        # Determine whether to send to CAN or Serial based on input arguments
        send_to_serial = '--serial' in args if args else False
        send_to_serial = True
        print(send_to_serial)
        serial_port = '/dev/ttyUSB0'   #/dev/ttyTHS1 # Modify with actual Arduino port, /dev/ttyTHS1 for Jimny; /dev/ttyUSB0 for rover
        baud_rate = 115200

        node = AckermannToVehicleNode(send_to_serial=send_to_serial, serial_port=serial_port, baud_rate=baud_rate)
        rclpy.spin(node)
        # node = AckermannToVehicleNode(send_to_serial=send_to_serial, serial_port=serial_port, baud_rate=baud_rate)
        # rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Shutting down.')
        if node.send_to_serial:
            node.send_to_serial_port('E')  # Send an emergency stop command

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)
