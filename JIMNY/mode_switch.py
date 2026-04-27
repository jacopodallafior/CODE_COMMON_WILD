#!/usr/bin/python3

"""
mode_switch_node.py
Author: Matthijs Steyerberg, Axel Barbelanne change by jacopo and nicholas
Date: 22-10-2024
"""

import rclpy
import numpy as np
import math
import time
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_msgs.msg import Float32MultiArray
from cuberos.msg import RCIn, Heartbeat
from ackermann_msgs.msg import AckermannDrive
from diagnostic_msgs.msg import KeyValue


def println(statement):
    if __name__ == "__main__":
        print(statement)
    else:
        print(f"[{__name__}]: {statement}")


class JIMNY:
    def __init__(self):
        self.steering_angle = None
        self.des_steering_angle = None
        self.max_steering_angle = 30  # degrees
        self.steer_ratio = 14
        self.steer_return_to_zero = 0.8
        self.des_steering_rate = None
        self.max_steering_rate = 30   # degrees/s
        self.throttle = None
        self.velocity = None

        # NOW IN m/s
        self.max_velocity = 15.0 / 3.6   # ~4.17 m/s
        self.min_velocity = -10.0 / 3.6  # ~-2.78 m/s

        self.des_velocity = None
        self.acceleration = None
        self.max_acceleration = 1.5  # m/s^2
        self.acc_return_to_zero = 0.95
        self.des_acceleration = None
        self.time = time.time()

    def reset_des(self):
        self.des_steering_angle = 0
        self.des_steering_rate = 0
        self.des_velocity = 0
        self.des_acceleration = 0
        self.time = time.time()

    def update_des(self):
        dt = time.time() - self.time
        self.set_des_velocity(dt)
        self.set_des_steering_angle(dt)
        self.time = time.time()

    def set_des_velocity(self, dt):
        self.des_velocity = self.des_velocity + (self.des_acceleration * dt)

        if self.des_velocity != 0:
            self.des_velocity = self.des_velocity * self.acc_return_to_zero

        if self.des_velocity > self.max_velocity:
            self.des_velocity = self.max_velocity
        elif self.des_velocity < self.min_velocity:
            self.des_velocity = self.min_velocity

    def set_des_steering_angle(self, dt):
        self.des_steering_angle = float(
            math.floor(self.des_steering_angle + self.des_steering_rate * dt)
        )
        if self.des_steering_angle != 0:
            self.des_steering_angle = float(
                math.floor(self.des_steering_angle * self.steer_return_to_zero)
            )

        if self.des_steering_angle > self.max_steering_angle:
            self.des_steering_angle = self.max_steering_angle
        elif self.des_steering_angle < -self.max_steering_angle:
            self.des_steering_angle = -self.max_steering_angle


class ModeSwitchNode(Node):
    def __init__(self):
        super().__init__('mode_switch_node')

        self.mode_subscriber = self.create_subscription(
            Heartbeat, 'heartbeat', self._heartbeat_callback, 10)

        self.feedback_subscriber = self.create_subscription(
            Float32MultiArray, 'feedback_data', self._feedback_callback, 10)

        self.rc_subscriber = self.create_subscription(
            RCIn, 'rc_inputs', self._rc_callback, 10)

        self.mpc_subscriber = self.create_subscription(
            AckermannDrive, 'mpc_commands', self._mpc_callback, 10)

        self.mpc_error_subscriber = self.create_subscription(
            Bool, 'mpc_error_state', self._error_state_callback, 10)

        self.excitation_subscriber = self.create_subscription(
            AckermannDrive, 'excitation_cmd', self._excitation_callback, 10)

        self.ackermann_publisher = self.create_publisher(AckermannDrive, 'ackermann_cmd', 10)
        self.mode_publisher = self.create_publisher(String, 'mode', 10)
        self.signal_error_publisher = self.create_publisher(KeyValue, 'signal_error', 10)

        self.heartbeat_timer = self.create_timer(0.2, self._check_heartbeat)
        self.disarmed_pub_timer = self.create_timer(0.2, self._pub_disarmed)
        self.error_publisher_timer = self.create_timer(0.1, self._pub_ack_signal_flag)

        self.last_heartbeat_time = time.time()
        self.heartbeat_timeout = 2

        self.mode = None
        self.armed = False
        self.jimny = JIMNY()

        self.MPC_VEL_TOLERANCE = 0.1  # m/s
        self.MPC_STEER_TOLERANCE = 5  # degrees
        self.can_start_mpc = False
        self.reset_mpc = True
        self.mpc_started = False

        self.signal_error = None
        self.signal_error_msg = None

        self.MAX_VEL = 12       # m/s
        self.MAX_STEER = 630

        self.signal_error = True
        self.signal_error_msg = "Initial setup"

        # --- RC manual steering: python-like incremental target ---
        # Stick center => target stays where it is
        # Stick deflection => changes target at a rate proportional to stick
        self.RC_RATE_SCALE = 180.0
        self.RC_STICK_DEADBAND = 70
        self.RC_TARGET_LIMIT = self.jimny.max_steering_angle * self.jimny.steer_ratio  # ~420 deg

        self._manual_target_angle = 0.0
        self._last_rc_time = time.time()
        self._steering_angle_fb = 0.0
        # ----------------------------------------------------------

    def _heartbeat_callback(self, msg):
        self.last_heartbeat_time = time.time()

        if self.mode != msg.mode:
            self.mpc_started = False

            if msg.mode not in ('MANUAL', 'ACRO', 'LOITER'):
                self.get_logger().warn(f"{msg.mode} mode not supported")
                self.mode = None
                self.signal_error = True
                self.signal_error_msg = f"Invalid mode: {msg.mode}"
            else:
                println(f"Mode changed to: {msg.mode}")
                self.mode = msg.mode
                self.signal_error = True
                self.signal_error_msg = "Temporary pause after mode change"

            self.jimny.reset_des()

            # Align manual target to actual steering feedback on mode change
            self._manual_target_angle = self._steering_angle_fb
            self._last_rc_time = time.time()

        mode_msg = String()
        mode_msg.data = msg.mode
        self.mode_publisher.publish(mode_msg)

        if self.armed != msg.armed:
            self.mpc_started = False
            println(f"Armed status changed to: {msg.armed}")
            self.armed = msg.armed

            # Align manual target to actual steering feedback on arm/disarm
            self._manual_target_angle = self._steering_angle_fb
            self._last_rc_time = time.time()

    def _check_heartbeat(self):
        if time.time() - self.last_heartbeat_time > self.heartbeat_timeout:
            self.get_logger().warn("Lost cube connection! No heartbeat received")
            self.signal_error = True
            self.signal_error_msg = "Lost cube connection! No heartbeat received"

    def _pub_disarmed(self):
        if not self.armed and self.mode is not None:
            self.publish_ackermann_msg()
            self.signal_error = False

    def _feedback_callback(self, msg):
        velocity_fb = msg.data[1]
        steering_angle_fb = msg.data[3]

        self._steering_angle_fb = steering_angle_fb

        self.can_start_mpc = (
            abs(steering_angle_fb) < self.MPC_STEER_TOLERANCE and
            abs(velocity_fb) < self.MPC_VEL_TOLERANCE
        )

        if abs(velocity_fb) >= self.MAX_VEL or abs(steering_angle_fb) >= self.MAX_STEER:
            self.get_logger().warn("Feedback signal out of bounds")
            self.signal_error = True
            self.signal_error_msg = "Feedback signal out of bounds"

    def _error_state_callback(self, msg):
        if msg.data:
            self.get_logger().warn("MPC did not find a valid solution")
            self.signal_error = True
            self.signal_error_msg = "MPC did not find a valid solution"
        if self.reset_mpc:
            self.reset_mpc = False
            if not msg.data:
                self.signal_error = False

    def _rc_callback(self, rc_msg):
        if self.mode == 'MANUAL' and self.armed:
            self.reset_mpc = True

            now = time.time()
            dt = now - self._last_rc_time
            dt = max(0.0, min(dt, 0.05))
            self._last_rc_time = now

            stick = rc_msg.channels[3]

            if abs(stick) < self.RC_STICK_DEADBAND:
                stick = 0.0

            # Stick controls target rate, not absolute target angle
            target_rate = float(np.interp(
                stick,
                [-1000, 1000],
                [-self.RC_RATE_SCALE, self.RC_RATE_SCALE]
            ))

            # Integrate rate into target angle
            self._manual_target_angle += target_rate * dt
            self._manual_target_angle = float(np.clip(
                self._manual_target_angle,
                -self.RC_TARGET_LIMIT,
                self.RC_TARGET_LIMIT
            ))

            self.jimny.des_steering_angle = self._manual_target_angle
            self.jimny.des_steering_rate = target_rate

            max_velocity = self.jimny.max_velocity
            min_velocity = self.jimny.min_velocity
            vel = round(
                np.interp(rc_msg.channels[1], [-1000, 1000], [min_velocity, max_velocity]), 2
            )
            self.jimny.des_velocity = vel

            self.publish_ackermann_msg(
                vel=vel,
                steering_angle=self._manual_target_angle,
                steering_angle_rate=target_rate
            )
            self.signal_error = False

        elif not self.armed:
            self._manual_target_angle = self._steering_angle_fb
            self._last_rc_time = time.time()
            println("Vehicle is not armed")

    def _mpc_callback(self, msg):
        if self.mode == 'ACRO' and self.armed:
            if not self.can_start_mpc and not self.mpc_started:
                self.get_logger().info("Cannot start MPC. Stop the vehicle and set the steering to neutral.")
                self.signal_error = True
                self.signal_error_msg = "Cannot start MPC. Vehicle is not set to neutral steering or not stopped."
                return

            self.mpc_started = True
            self.jimny.des_steering_angle = msg.steering_angle
            self.jimny.des_steering_rate = msg.steering_angle_velocity
            self.jimny.des_velocity = msg.speed
            self.jimny.des_acceleration = msg.acceleration

            self.publish_ackermann_msg(
                steering_angle=self.jimny.des_steering_angle,
                steering_angle_rate=self.jimny.des_steering_rate,
                vel=self.jimny.des_velocity,
                acc=self.jimny.des_acceleration
            )
            self.signal_error = False

    def _excitation_callback(self, msg):
        if self.mode == 'LOITER' and self.armed:
            println("Received excitation command")
            self.publish_ackermann_msg(
                steering_angle=msg.steering_angle,
                steering_angle_rate=msg.steering_angle_velocity,
                vel=msg.speed,
                acc=msg.acceleration,
                jerk=msg.jerk
            )
            self.signal_error = False
        else:
            return

    def _pub_ack_signal_flag(self):
        if self.signal_error is None:
            return

        msg = KeyValue()

        if not self.armed:
            msg.key = "0"
            msg.value = "Disarmed"
        elif not self.signal_error:
            msg.key = "1"
            msg.value = "Armed, signal ok"
        else:
            msg.key = "2"
            msg.value = self.signal_error_msg

        self.signal_error_publisher.publish(msg)

    def publish_ackermann_msg(self, steering_angle=0., steering_angle_rate=0., vel=0., acc=0., jerk=0.):
        ackermann_msg = AckermannDrive()
        ackermann_msg.steering_angle = steering_angle
        ackermann_msg.steering_angle_velocity = steering_angle_rate
        ackermann_msg.speed = vel
        ackermann_msg.acceleration = acc
        ackermann_msg.jerk = jerk

        self.ackermann_publisher.publish(ackermann_msg)
        println(
            f'Published AckermannDrive: Steering={ackermann_msg.steering_angle}, '
            f'Steer rate={ackermann_msg.steering_angle_velocity}, '
            f'Velocity={ackermann_msg.speed}, '
            f'Acceleration={ackermann_msg.acceleration}, '
            f'Jerk={ackermann_msg.jerk}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitchNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
