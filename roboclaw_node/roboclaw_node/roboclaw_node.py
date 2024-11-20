#!/usr/bin/env python

from math import pi, cos, sin
import numpy as np
import diagnostic_msgs
import diagnostic_updater

from roboclaw_driver.roboclaw_driver import Roboclaw
import rclpy
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import Vector3, Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

import transforms3d
import time
import sys

from rclpy.time import Duration

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"


class OdomEncoder:
    def __init__(self, ticks_per_meter, base_width, _node):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.node = _node
        self.odom_pub = self.node.create_publisher(Odometry, '/odom', 10)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0  # M1=left
        self.last_enc_right = 0  # M2=right
        self.last_enc_time = self.node.get_clock().now()
        self.broadcaster = TransformBroadcaster(self.node)

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """
        Calculate quaternion from euler angles.
        """
        roll /= 2.0
        pitch /= 2.0
        yaw /= 2.0
        cos_roll = cos(roll)
        sin_roll = sin(roll)
        cos_pitch = cos(pitch)
        sin_pitch = sin(pitch)
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        cos_roll_x_cos_yaw = cos_roll * cos_yaw
        cos_roll_x_sin_yaw = cos_roll * sin_yaw
        sin_roll_x_cos_yaw = sin_roll * cos_yaw
        sin_roll_x_sin_yaw = sin_roll * sin_yaw

        q = np.empty((4,))
        q[0] = cos_pitch * sin_roll_x_cos_yaw - sin_pitch * cos_roll_x_sin_yaw
        q[1] = cos_pitch * sin_roll_x_sin_yaw + sin_pitch * cos_roll_x_cos_yaw
        q[2] = cos_pitch * cos_roll_x_sin_yaw - sin_pitch * sin_roll_x_cos_yaw
        q[3] = cos_pitch * cos_roll_x_cos_yaw + sin_pitch * sin_roll_x_sin_yaw

        return q

    @staticmethod
    def normalize_angle(angle):
        """
        Normalize angle to be in the range of -pi to pi.
        """
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        """
        Calculate current linear and angular velocity from encoder ticks.
        """
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0
        current_time = self.node.get_clock().now()
        d_time = (current_time.nanoseconds - self.last_enc_time.nanoseconds) / 1000000000
        self.last_enc_time = current_time

        encoder_deviation_threshold = 10
        if abs(left_ticks - right_ticks) < encoder_deviation_threshold:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        """
        Update odometry and publish. Ignore encoder jumps.
        """
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        if abs(enc_left - self.last_enc_left) > 20000:
            self.node.get_logger().info("Ignoring left encoder jump: cur %d, last %d" % (enc_left, self.last_enc_left))
        elif abs(enc_right - self.last_enc_right) > 20000:
            self.node.get_logger().info("Ignoring right encoder jump: cur %d, last %d" % (enc_right, self.last_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        """
        Publish odometry and base_footprint message.
        """
        #broadcaster = TransformBroadcaster(self.node)

        current_time = self.node.get_clock().now()
        z_rot_quat = self.quaternion_from_euler(0, 0, yaw=cur_theta)

        base_footprint_transform = TransformStamped()
        base_footprint_transform.header.stamp = current_time.to_msg()
        base_footprint_transform.header.frame_id = "odom"
        base_footprint_transform._child_frame_id = "base_footprint"

        base_footprint_transform.transform.translation = Vector3(x=cur_x, y=cur_y, z=0.0)
        base_footprint_transform.transform.rotation = Quaternion(x=z_rot_quat[0],
                                                                 y=z_rot_quat[1],
                                                                 z=z_rot_quat[2],
                                                                 w=z_rot_quat[3])

        self.broadcaster.sendTransform(base_footprint_transform)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=z_rot_quat[0],
                                                y=z_rot_quat[1],
                                                z=z_rot_quat[2],
                                                w=z_rot_quat[3])

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_footprint'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance
        self.odom_pub.publish(odom)


class Roboclaw_node(Node):

    def __init__(self):
        super().__init__('roboclaw_node')

        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        self.declare_parameter("dev", "/dev/ttyACM0")
        self.declare_parameter("baud", 38400)
        self.declare_parameter("address", 128)
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("ticks_per_meter", 2495.0)
        self.declare_parameter("base_width", 0.421)
        self.declare_parameter("invert_motor_direction", False)
        self.declare_parameter("flip_left_and_right_motors", False)

        # Set default values
        self.get_logger().info("Roboclaw: set default values.")

        dev_name = str(self.get_parameter("dev").value)
        baud_rate = int(self.get_parameter("baud").value)
        self.ADDRESS = int(self.get_parameter("address").value)
        self.MAX_SPEED = float(self.get_parameter("max_speed").value)
        self.TICKS_PER_METER = float(self.get_parameter("ticks_per_meter").value)
        self.BASE_WIDTH = float(self.get_parameter("base_width").value)
        self.INVERT_MOTOR_DIRECTION = self.get_parameter("invert_motor_direction").value
        self.FLIP_LEFT_AND_RIGHT_MOTORS = self.get_parameter("flip_left_and_right_motors").value

        if self.ADDRESS > 0x87 or self.ADDRESS < 0x80:
            self.get_logger().logfatal("Address out of range")
            rclpy.shutdown()

        # Connect to Roboclaw
        self.roboclaw = Roboclaw(dev_name, baud_rate)
        self.get_logger().info('Connecting to roboclaw')
        if self.roboclaw.Open() == 1:
            self.get_logger().info('Roboclaw: Connection established')
        else:
            self.get_logger().error("Roboclaw: Couldn't open port. Is 'ls /dev/ | grep %s' available?" % dev_name)
            self.get_logger().info("Shutting down.")
            rclpy.shutdown()

        # Check version
        self.get_logger().info("Roboclaw: fetching version")
        (status, version) = self.roboclaw.ReadVersion(self.ADDRESS)
        if status == 1:
            self.get_logger().info("Roboclaw: version is %s" % version)
        else:
            self.get_logger().warn("Roboclaw: Problem getting version with status {} version {}. Continue."
                                   .format(status, version))

        # Set up diagnostics
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.FunctionDiagnosticTask("Vitals", self.check_vitals))
        self.updater.update()

        # Reset motors
        self.get_logger().info("Roboclaw: stop motors.")
        self.roboclaw.SpeedM1M2(self.ADDRESS, 0, 0)
        self.roboclaw.ResetEncoders(self.ADDRESS)

        # Set clock
        self.manual_spin_rate = 10 # hz
        self.last_set_speed_time = self.get_clock().now()

        # Set up subscriber
        self.subscription = self.create_subscription(Twist, "base/cmd_vel", self.cmd_vel_callback, 10)

        # Set up odometry encoder
        self.odom_encoder = OdomEncoder(self.TICKS_PER_METER, self.BASE_WIDTH, self)

        # Publish defaults
        self.get_logger().info("dev %s" % dev_name)
        self.get_logger().info("baud %d" % baud_rate)
        self.get_logger().info("address %d" % self.ADDRESS)
        self.get_logger().info("max_speed %f" % self.MAX_SPEED)
        self.get_logger().info("ticks_per_meter %f" % self.TICKS_PER_METER)
        self.get_logger().info("base_width %f" % self.BASE_WIDTH)
        self.get_logger().info("invert_motor_direction %s" % self.INVERT_MOTOR_DIRECTION)
        self.get_logger().info("flip_left_and_right_motors %s" % self.FLIP_LEFT_AND_RIGHT_MOTORS)

    def run(self):
        """
        Main loop for roboclaw node, including spin for cmd_vel and odom updates
        """
        self.get_logger().info('Roboclaw: Starting motor drive')
        rate = 100  # run at 100 hz to compensate cmd_vel topic input rate
        while rclpy.ok():
            rclpy.spin_once(self)

            # stop motors if no command received for 1 second
            if (self.get_clock().now() - self.last_set_speed_time) > Duration(seconds=1.0):
                try:
                    self.roboclaw.ForwardM1(self.ADDRESS, 0)
                    self.roboclaw.ForwardM2(self.ADDRESS, 0)
                except OSError as e:
                    self.get_logger().error("Could not stop")
                    self.get_logger().info(e)

            # reset encoders
            status_left, enc_left, crc_left = None, None, None
            status_right, enc_right, crc_right = None, None, None

            # read left encoders
            try:
                status_left, enc_left, crc_left = self.roboclaw.ReadEncM1(self.ADDRESS)
            except ValueError:
                pass
            except OSError as e:
                self.get_logger().warn("ReadEncM1 OSError: %d", e.errno)
                self.get_logger().info(e)

            # read right encoders
            try:
                status_right, enc_right, crc_right = self.roboclaw.ReadEncM2(self.ADDRESS)
            except ValueError:
                pass
            except OSError as e:
                self.get_logger().warn("ReadEncM2 OSError: %d", e.errno)
                self.get_logger().info(e)

            # invert encoders if necessary
            if self.INVERT_MOTOR_DIRECTION == 1:
                enc_left = -enc_left
                enc_right = -enc_right

            # flip direction if necessary
            if self.FLIP_LEFT_AND_RIGHT_MOTORS == 1:
                enc_left, enc_right = enc_right, enc_left

            # send encoder data to odometry calculation
            try:
                self.odom_encoder.update_publish(enc_left, enc_right)  # update_publish expects enc_left enc_right
                self.updater.update()
            except Exception as e:
                self.get_logger().info("Issue publishing to odom" + str(e))

            # sleep
            time.sleep(1.0/rate)

    def cmd_vel_callback(self, twist):
        """
            Callback for velocity commands. Send commands to roboclaw.
        """
        self.last_set_speed_time = self.get_clock().now()
        linear_x = twist.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        vel_right = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
        vel_left = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0

        if self.INVERT_MOTOR_DIRECTION:
            vel_right = -vel_right
            vel_left = -vel_left

        if self.FLIP_LEFT_AND_RIGHT_MOTORS:
            vel_left, vel_right = vel_right, vel_left

        left_ticks = int(vel_left * self.TICKS_PER_METER)
        right_ticks = int(vel_right * self.TICKS_PER_METER)  # ticks/s

        if left_ticks == 0 and right_ticks == 0:
            self.roboclaw.ForwardM1(self.ADDRESS, 0)
            self.roboclaw.ForwardM2(self.ADDRESS, 0)
        else:
            self.roboclaw.SpeedM1M2(self.ADDRESS, left_ticks, right_ticks)

    def shutdown(self):
        """
        Stop motors and shut down.
        """
        self.get_logger().info('Shutting down')
        try:
            self.roboclaw.ForwardM1(self.ADDRESS, 0)
            self.roboclaw.ForwardM2(self.ADDRESS, 0)
        except OSError:
            self.get_logger().error("Shutdown did not work trying again")
            try:
                self.roboclaw.ForwardM1(self.ADDRESS, 0)
                self.roboclaw.ForwardM2(self.ADDRESS, 0)
            except OSError as e:
                self.get_logger().error("Could not shutdown motors!!!!")
                self.get_logger().info(e)

    def check_vitals(self, stat):
        """
        Diagnostics stuff that needs more understanding.
        """
        try:
            status = self.roboclaw.ReadError(self.ADDRESS)[1]
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: %d", e.errno)
            self.get_logger().info(e)  # rclpy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", str((self.roboclaw.ReadMainBatteryVoltage(self.ADDRESS)[1] / 10)))
            stat.add("Logic Batt V:", str(float(self.roboclaw.ReadLogicBatteryVoltage(self.ADDRESS)[1] / 10)))
            stat.add("Temp1 C:", str(float(self.roboclaw.ReadTemp(self.ADDRESS)[1] / 10)))
            stat.add("Temp2 C:", str(float(self.roboclaw.ReadTemp2(self.ADDRESS)[1] / 10)))
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: %d", e.errno)
            self.get_logger().info(e)
        return stat


def main(args=None):
    rclpy.init(args=args)
    roboclaw_node = Roboclaw_node()
    try:
        roboclaw_node.run()
    except KeyboardInterrupt:
        sys.exit(1)
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        roboclaw_node.shutdown()
        roboclaw_node.get_logger().info('Exiting')
        roboclaw_node.destroy_node()


if __name__ == '__main__':
    main()
