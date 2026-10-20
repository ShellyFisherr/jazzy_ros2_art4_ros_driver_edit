#
# @file: udp_receiver
# @brief: A script for receiving UDP messages and publishing them to ROS2
#
# It receives tracker data over UDP and publishes the latest tracker
# position and velocity to the respective ROS2 topics.
#
# --------------------------------------------
# Subscribed Topics:
#
# None
#
# --------------------------------------------
# Published Topics:
#
# /tracker/pose
# /tracker/velocity
#
# --------------------------------------------
# @author: Daan Krijnen
# @date: October 07 - 2026
#

import socket
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


class TrackerUdpBridge(Node):

    def __init__(self):

        super().__init__('tracker_udp_bridge')

        self.udp_ip = '0.0.0.0'
        self.udp_port = 5005

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        self.sock.bind(
            (self.udp_ip, self.udp_port)
        )

        self.sock.settimeout(0.1)

        self.latest_data = None
        self.data_lock = threading.Lock()


        self.running = True

        self.last_receive_time = None
        self.last_sequence = None


        self.update_rate = 100.0  # Hz
        self.max_data_age = 0.1  # seconds


        self.received_packets = 0
        self.published_packets = 0
        self.timer_calls = 0
        self.sequence_jumps = 0

        self.last_publish_time = None

        self.max_publish_interval = 0.0

        self.last_diagnostic_time = time.monotonic()


        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/tracker/pose',
            1
        )

        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            '/tracker/velocity',
            1
        )


        self.publish_timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_latest_data
        )


        self.receiver_thread = threading.Thread(
            target=self.receive_udp_data,
            daemon=True
        )

        self.receiver_thread.start()


        self.get_logger().info(
            f'Listening for UDP packets on '
            f'{self.udp_ip}:{self.udp_port}'
        )

        self.get_logger().info(
            'UDP receiver running in separate thread'
        )

        self.get_logger().info(
            f'ROS2 publishing rate: {self.update_rate:.1f} Hz'
        )


    def receive_udp_data(self):

        while self.running:

            try:

                data, address = self.sock.recvfrom(4096)

            except socket.timeout:

                continue

            except OSError:

                break

            try:

                message = data.decode('utf-8').strip()

                values = message.split(',')

                if len(values) != 15:
                    continue

                sequence = int(values[0])
                timestamp = float(values[1])

                x = float(values[2])
                y = float(values[3])
                z = float(values[4])

                qx = float(values[5])
                qy = float(values[6])
                qz = float(values[7])
                qw = float(values[8])

                vx = float(values[9])
                vy = float(values[10])
                vz = float(values[11])

                wx = float(values[12])
                wy = float(values[13])
                wz = float(values[14])

            except (ValueError, UnicodeDecodeError):

                continue


            receive_time = time.monotonic()


            if self.last_sequence is not None:

                sequence_difference = (
                    sequence - self.last_sequence
                )

                if sequence_difference <= 0

                if sequence_difference != 1:

                    self.sequence_jumps += 1

            self.last_sequence = sequence

            packet = {

                'sequence': sequence,
                'timestamp': timestamp,

                'x': x,
                'y': y,
                'z': z,

                'qx': qx,
                'qy': qy,
                'qz': qz,
                'qw': qw,

                'vx': vx,
                'vy': vy,
                'vz': vz,

                'wx': wx,
                'wy': wy,
                'wz': wz,

                'receive_time': receive_time,
            }


            with self.data_lock:

                self.latest_data = packet
                self.last_receive_time = receive_time

            self.received_packets += 1


    def publish_latest_data(self):


        current_publish_time = time.monotonic()

        self.timer_calls += 1

        if self.last_publish_time is not None:

            publish_interval = (
                current_publish_time
                - self.last_publish_time
            )

            if publish_interval > self.max_publish_interval:

                self.max_publish_interval = publish_interval

        self.last_publish_time = current_publish_time


        with self.data_lock:

            if self.latest_data is None:

                self.print_diagnostics()

                return

            data = self.latest_data.copy()

            last_receive_time = self.last_receive_time


        current_time = time.monotonic()

        data_age = (
            current_time
            - last_receive_time
        )

        if data_age > self.max_data_age:

            self.publish_zero_velocity()

            self.print_diagnostics()

            return

        pose_msg = PoseStamped()

        pose_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose_msg.header.frame_id = 'steamvr_world'

        pose_msg.pose.position.x = data['x']
        pose_msg.pose.position.y = data['y']
        pose_msg.pose.position.z = data['z']

        pose_msg.pose.orientation.x = data['qx']
        pose_msg.pose.orientation.y = data['qy']
        pose_msg.pose.orientation.z = data['qz']
        pose_msg.pose.orientation.w = data['qw']

        self.pose_publisher.publish(
            pose_msg
        )


        velocity_msg = TwistStamped()

        velocity_msg.header.stamp = (
            pose_msg.header.stamp
        )

        velocity_msg.header.frame_id = 'steamvr_world'

        velocity_msg.twist.linear.x = data['vx']
        velocity_msg.twist.linear.y = data['vy']
        velocity_msg.twist.linear.z = data['vz']

        velocity_msg.twist.angular.x = data['wx']
        velocity_msg.twist.angular.y = data['wy']
        velocity_msg.twist.angular.z = data['wz']

        self.velocity_publisher.publish(
            velocity_msg
        )

        self.published_packets += 1

        self.print_diagnostics()

    def publish_zero_velocity(self):

        velocity_msg = TwistStamped()

        velocity_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        velocity_msg.header.frame_id = 'steamvr_world'

        velocity_msg.twist.linear.x = 0.0
        velocity_msg.twist.linear.y = 0.0
        velocity_msg.twist.linear.z = 0.0

        velocity_msg.twist.angular.x = 0.0
        velocity_msg.twist.angular.y = 0.0
        velocity_msg.twist.angular.z = 0.0

        self.velocity_publisher.publish(
            velocity_msg
        )

    def print_diagnostics(self):

        current_time = time.monotonic()

        # Only print diagnostics once per second.
        if (
            current_time - self.last_diagnostic_time
            < 1.0
        ):
            return

        self.last_diagnostic_time = current_time

        self.get_logger().info(

            f'RX: {self.received_packets:6d} | '

            f'PUB: {self.published_packets:6d} | '

            f'TIMER: {self.timer_calls:6d} | '

            f'SEQ_JUMPS: {self.sequence_jumps:4d} | '

            f'LATEST_SEQ: {self.last_sequence} | '

            f'MAX_PUB_DT: '
            f'{self.max_publish_interval * 1000:.2f} ms'
        )

    def destroy_node(self):

        self.running = False

        try:

            self.sock.close()

        except OSError:

            pass

        if self.receiver_thread.is_alive():

            self.receiver_thread.join(
                timeout=1.0
            )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = TrackerUdpBridge()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()