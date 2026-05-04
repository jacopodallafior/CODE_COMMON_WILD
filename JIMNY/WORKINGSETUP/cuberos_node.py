#!/usr/bin/python3
import rclpy
import math
import numpy as np
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Imu
from gps_msgs.msg import GPSFix
from cuberos.msg import RCIn, Heartbeat
from cuberos_py.mavlink import CubeSerial

DANCE = False   # Special feature for Clovis

def println(statement):
    if __name__ == "__main__":
        print(statement)
    else:
        if DANCE:
            print(f"[{__name__}]: {statement}")


class CubeROSNode(Node):
    """ROS Node to bridge CubePilot data to ROS topics using Mavlink"""
    def __init__(self):
        super().__init__('cube_ros_node')
        # Publishers for the data from the CubePilot
        self.publisher_gps = self.create_publisher(GPSFix, 'gps', 10)
        self.publisher_gps_raw = self.create_publisher(GPSFix, 'gps_raw', 10)
        self.publisher_rc = self.create_publisher(RCIn, 'rc_inputs', 10)
        self.publisher_imu = self.create_publisher(Imu, 'imu_attitude', 10)
        self.publisher_heartbeat = self.create_publisher(Heartbeat, 'heartbeat', 1)

        # Cube object to communicate with the CubePilot using Mavlink to receive data
        #self.cube = CubeSerial('/dev/ttyACM0', 115200)  # Adjust the port based on the system
        self.cube = CubeSerial('/dev/ttyACM0', 115200)  # Adjust the port based on the system
        # Create a timer to call get_messages() frequently

        self.timer_get_messages = self.create_timer(0.0001, self.get_messages)  # Adjust frequency as needed

        # Publish on the topics with the same frequency as the data is requested from the Cube
        self.timer_gps = self.create_timer(1/self.cube.global_position_int_freq, self.publish_gps_data)
        self.timer_gps_raw = self.create_timer(1/self.cube.gps_raw_int_freq, self.publish_gps_raw_data)
        self.timer_rc = self.create_timer(1/self.cube.rc_freq, self.publish_rc_data)
        self.timer_imu = self.create_timer(1/self.cube.imu_freq, self.publish_imu_data)
        self.timer_heartbeat = self.create_timer(1, self.publish_heartbeat_data)


    def get_messages(self):
        """Retrieve messages from the Cube"""
        self.cube.get_messages()  # Call to get messages as often as possible

    def publish_gps_data(self):
        """Publish GPS data with accelerometer motion compensation"""
        println("Publishing GPS data")
        if self.cube.global_position_int:
            gps_msg = GPSFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "global"

            gps_msg.latitude = self.cube.global_position_int.lat
            gps_msg.longitude = self.cube.global_position_int.lon
            gps_msg.altitude = self.cube.global_position_int.alt
            gps_msg.track = self.cube.global_position_int.hdg       # Compass heading
            self.publisher_gps.publish(gps_msg)

    def publish_gps_raw_data(self):
        """Publish raw GPS data"""
        println("Publishing raw GPS data")
        gps_msg = GPSFix()
        publish = False
        if self.cube.gps_raw_int:
            publish = True
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "global"
            gps_msg.status.header.stamp = self.get_clock().now().to_msg()
            gps_msg.status.header.frame_id = "global"
            gps_msg.status.status = self.cube.gps_raw_int.status

            gps_msg.latitude = self.cube.gps_raw_int.lat
            gps_msg.longitude = self.cube.gps_raw_int.lon
            gps_msg.altitude = self.cube.gps_raw_int.alt
            
            gps_msg.status.satellites_visible = self.cube.gps_raw_int.satellites_visible

            gps_msg.hdop = self.cube.gps_raw_int.eph  # Horizontal dilution of precision
            gps_msg.vdop = self.cube.gps_raw_int.epv

            gps_msg.speed = self.cube.gps_raw_int.vel  # Ground speed (meters/second)
            gps_msg.track = self.cube.gps_raw_int.cog  # Course over ground (NOT heading, but direction of movement) in degrees

            gps_msg.err_horz = self.cube.gps_raw_int.h_acc  # Horizontal position uncertainty (meters) [eph]
            gps_msg.err_vert = self.cube.gps_raw_int.v_acc  # Vertical position uncertainty (meters) [epv]
            gps_msg.err_track = self.cube.gps_raw_int.hdg_acc  # Track uncertainty (degrees) [epd]
            gps_msg.err_speed = self.cube.gps_raw_int.vel_acc  # Ground speed uncertainty (meters/second) [eps]

        if self.cube.vfr_hud:
            gps_msg.climb = self.cube.vfr_hud.climb
        if self.cube.gps_status:
            gps_msg.status.satellites_used = self.cube.gps_status.satellites_used
        if self.cube.attitude:
            gps_msg.pitch = math.degrees(self.cube.attitude.pitch)
            gps_msg.roll = math.degrees(self.cube.attitude.roll)
            gps_msg.dip = math.degrees(self.cube.attitude.dip)

        if publish:
            self.publisher_gps_raw.publish(gps_msg)

    def publish_rc_data(self):
        """Publish scaled RC channels"""
        println("Publishing RC channels")
        if self.cube.rc:
            rc_msg = RCIn()
            rc_msg.header.stamp = self.get_clock().now().to_msg()
            rc_msg.rssi = self.cube.rc.rssi
            rc_msg.channels = np.interp(
                [
                    self.cube.rc.chan1, 
                    self.cube.rc.chan2, 
                    self.cube.rc.chan3, 
                    self.cube.rc.chan4, 
                    self.cube.rc.chan5
                ],
                [1102, 1928],
                [-1000, 1000]
            ).astype(int).tolist()
            self.publisher_rc.publish(rc_msg)

    def publish_imu_data(self):
        """Publish IMU/ATTITUDE data"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "Jimny"

        if self.cube.imu:
            # Populate IMU message with data from `cube`
            imu_msg.linear_acceleration.x = self.cube.imu.xacc
            imu_msg.linear_acceleration.y = self.cube.imu.yacc
            imu_msg.linear_acceleration.z = self.cube.imu.zacc

        if self.cube.attitude:
            # Set orientation (convert roll/pitch/yaw to quaternion if needed)
            q = quaternion_from_euler(self.cube.attitude.roll, 
                                      self.cube.attitude.pitch, 
                                      self.cube.attitude.yaw)
            imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            # Set angular velocity
            imu_msg.angular_velocity.x = self.cube.attitude.rollspeed
            imu_msg.angular_velocity.y = self.cube.attitude.pitchspeed
            imu_msg.angular_velocity.z = self.cube.attitude.yawspeed

        # Publish the message
        self.publisher_imu.publish(imu_msg)

    def publish_heartbeat_data(self):
        """Publish flight mode of the Cube"""
        println("Publishing mode")
        if self.cube.heartbeat:
            mode_msg = Heartbeat()
            mode_msg.header.stamp = self.get_clock().now().to_msg()

            mode_msg.mode = self.cube.heartbeat.mode
            mode_msg.mav_type = self.cube.heartbeat.mav_type
            mode_msg.base_mode = self.cube.heartbeat.base_mode
            mode_msg.armed = self.cube.heartbeat.armed
            mode_msg.mav_autopilot = self.cube.heartbeat.mav_autopilot
            self.publisher_heartbeat.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)

    cube_ros_node = CubeROSNode()

    try:
        rclpy.spin(cube_ros_node)  # Keep the node running
    except KeyboardInterrupt:
        pass

    cube_ros_node.destroy_node()  # Clean up the node
    rclpy.shutdown()


if __name__ == "__main__":
    main()
