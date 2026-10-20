import socket

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


class TrackerUdpBridge(Node):
    
  def __init__(self):
      super().__init__('tracker_udp_bridge')

      # udp config
      self.udp_ip = '0.0.0.0'
      self.udp_port = 5005

      self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

      self.sock.bind((self.udp_ip, self.udp_port))

      #make socket non-blocking
      self.sock.setblocking(False)

      self.pose_publisher = self.create_publisher(PoseStamped, '/tracker/pose', 10)

      self.velocity_publisher = self.create_publisher(TwistStamped, '/tracker/velocity', 10)


      #check UDP socket at 100 Hz
      self.timer = self.create_timer(0.01, self.receive_udp_data)

      self.get_logger().info(f'Listening for UDP packets on {self.udp_ip}:{self.udp_port}')


  def receive_udp_data(self):

    try:
     data, adress = self.sock.recvfrom(4096)

    except BlockingIOError:
      return

    try:
      message = data.decode('utf-8').strip()

      values = message.split(',')

      if len(values) !=15:
        self.get_logger().warning(f'Expected 15 values, but got {len(values)}')
        return

      #-------------------------------
      # Parse packet data
      #
      # 0 sequence number
      # 1 timestamp
      # 2 x - position
      # 3 y - position
      # 4 z - position
      # 5 qx - quaternion
      # 6 qy - quaternion
      # 7 qz - quaternion
      # 8 qw - quaternion
      # 9 vx - velocity
      # 10 vy - velocity
      # 11 vz - velocity
      # 12 wx - angular velocity
      # 13 wy - angular velocity
      # 14 wz - angular velocity
      #-------------------------------

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

    except (ValueError, UnicodeDecodeError) as error:
      self.get_logger().warning(f'Error parsing UDP packet: {error}')
      return

    #publish
    pose_msg = PoseStamped()
    pose_msg.header.stamp = self.get_clock().now().to_msg()

    pose_msg.header.frame_id = 'steamvr_world'
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z

    pose_msg.pose.orientation.x = qx
    pose_msg.pose.orientation.y = qy
    pose_msg.pose.orientation.z = qz
    pose_msg.pose.orientation.w = qw

    self.pose_publisher.publish(pose_msg)


    velocity_msg = TwistStamped()
    velocity_msg.header.stamp = pose_msg.header.stamp
    velocity_msg.header.frame_id = 'steamvr_world'

    velocity_msg.twist.linear.x = vx
    velocity_msg.twist.linear.y = vy
    velocity_msg.twist.linear.z = vz

    velocity_msg.twist.angular.x = wx
    velocity_msg.twist.angular.y = wy
    velocity_msg.twist.angular.z = wz

    self.velocity_publisher.publish(velocity_msg)

  def destroy_node(self):
    self.sock.close()
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