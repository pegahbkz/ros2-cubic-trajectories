import rclpy
from rclpy.node import Node
from ar_interface.msg import CubicTrajParams
import random
import time

class PointsGenerator(Node):
    def __init__(self):
        # initialize node
        super().__init__('points_generator')
        # create publisher
        self.publisher_ = self.create_publisher(CubicTrajParams, 'cubic_traj_params', 10)
        # set timer to publish every 10 seconds
        self.timer = self.create_timer(10.0, self.publish_random_params)

    def publish_random_params(self):
        # create message
        msg = CubicTrajParams()
        # initial position
        msg.p0 = random.uniform(-10, 10)
        # final position
        msg.pf = random.uniform(-10, 10)
        # initial velocity
        msg.v0 = random.uniform(-10, 10)
        # random final velocity
        msg.vf = random.uniform(-10, 10)
        # initial time
        msg.t0 = 0.0
        # final time
        msg.tf = msg.t0 + random.uniform(4, 8)
        # publish message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: p0={msg.p0}, pf={msg.pf}, v0={msg.v0}, vf={msg.vf}, t0={msg.t0}, tf={msg.tf}')

def main(args=None):
    rclpy.init(args=args)
    node = PointsGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
