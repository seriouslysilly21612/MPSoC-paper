#!/usr/bin/env python3
"""The board side of the clock of the hardware-in-the-loop camera (tools/sim/hil_camera.py).

Answers every ping of /hil/clock_ping [seq, t_pc_sent] with /hil/clock_pong [seq, t_pc_sent,
t_board], t_board the ROS clock of this machine: the clock of the image stamps, of release_s in
decisions.csv and of the status of the control task. hil_camera.py takes the offset of the two
clocks from the pings with the shortest round trip.

  source /opt/ros/humble/setup.bash && python3 clock_echo.py      (on the board, Ctrl-C ends)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray


class ClockEcho(Node):
    def __init__(self):
        super().__init__('hil_clock_echo')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Float64MultiArray, '/hil/clock_pong', qos)
        self.create_subscription(Float64MultiArray, '/hil/clock_ping', self.ping, qos)
        self.n = 0

    def ping(self, msg):
        now = 1e-9 * self.get_clock().now().nanoseconds
        out = Float64MultiArray()
        out.data = [float(msg.data[0]), float(msg.data[1]), now]
        self.pub.publish(out)
        self.n += 1
        if self.n in (1, 100) or self.n % 1000 == 0:
            self.get_logger().info(f'{self.n} pings answered')


def main():
    rclpy.init()
    node = ClockEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
