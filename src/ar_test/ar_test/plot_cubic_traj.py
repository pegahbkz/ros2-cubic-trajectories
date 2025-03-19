import rclpy
from rclpy.node import Node
from ar_interface.msg import CubicTrajCoeffs
from std_msgs.msg import Float64
import numpy as np

class PublishCubicTraj(Node):
    def __init__(self):
        super().__init__('publish_cubic_traj')

        # Subscription to trajectory coefficients
        self.subscription = self.create_subscription(
            CubicTrajCoeffs, 'cubic_traj_coeffs', self.compute_trajectory, 10)

        # Publishers for position, velocity, and acceleration
        self.position_publisher = self.create_publisher(Float64, 'cubic_trajectory/position', 10)
        self.velocity_publisher = self.create_publisher(Float64, 'cubic_trajectory/velocity', 10)
        self.acceleration_publisher = self.create_publisher(Float64, 'cubic_trajectory/acceleration', 10)

        # Timer to publish at 0.1-second intervals
        self.timer = self.create_timer(0.1, self.publish_next)
        self.index = 0
        self.t = []
        self.p = []
        self.v = []
        self.a = []
        self.trajectory_ready = False

    def compute_trajectory(self, msg):
        self.get_logger().info(f'Received cubic trajectory coefficients: a0={msg.a0}, a1={msg.a1}, a2={msg.a2}, a3={msg.a3}, t0={msg.t0}, tf={msg.tf}')

        # Generate time values at 0.1s intervals
        self.t = np.arange(msg.t0, msg.tf, 0.1)

        # Compute position, velocity, and acceleration
        self.p = msg.a0 + msg.a1 * self.t + msg.a2 * self.t**2 + msg.a3 * self.t**3
        self.v = msg.a1 + 2 * msg.a2 * self.t + 3 * msg.a3 * self.t**2
        self.a = 2 * msg.a2 + 6 * msg.a3 * self.t

        self.index = 0  # Reset index for new trajectory
        self.trajectory_ready = True  # Set flag to start publishing
        self.get_logger().info("Trajectory computed. Publishing data in real-time.")

    def publish_next(self):
        if self.trajectory_ready and self.index < len(self.t):
            # Create and publish individual Float64 messages
            pos_msg = Float64()
            vel_msg = Float64()
            acc_msg = Float64()

            pos_msg.data = self.p[self.index]
            vel_msg.data = self.v[self.index]
            acc_msg.data = self.a[self.index]

            self.position_publisher.publish(pos_msg)
            self.velocity_publisher.publish(vel_msg)
            self.acceleration_publisher.publish(acc_msg)

            self.get_logger().info(f"Published: t={self.t[self.index]:.1f}, p={self.p[self.index]:.3f}, v={self.v[self.index]:.3f}, a={self.a[self.index]:.3f}")

            self.index += 1  # Move to the next value

        if self.index >= len(self.t):  
            self.trajectory_ready = False  # Stop publishing after last value

def main(args=None):
    rclpy.init(args=args)
    node = PublishCubicTraj()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
