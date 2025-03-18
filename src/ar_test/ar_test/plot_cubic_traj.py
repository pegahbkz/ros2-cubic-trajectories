import rclpy
from rclpy.node import Node
from ar_interface.msg import CubicTrajCoeffs
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime

class TrajectoryProcessor(Node):
    def __init__(self):
        # initialize node
        super().__init__('trajectory_processor')
        
        # create subscription
        self.subscription = self.create_subscription(
            CubicTrajCoeffs, 'cubic_traj_coeffs', self.process_trajectory, 10)
        
        # publishers for position, velocity, and acceleration
        self.position_publisher = self.create_publisher(Float64MultiArray, 'cubic_trajectory/position', 10)
        self.velocity_publisher = self.create_publisher(Float64MultiArray, 'cubic_trajectory/velocity', 10)
        self.acceleration_publisher = self.create_publisher(Float64MultiArray, 'cubic_trajectory/acceleration', 10)

        # set up the figure for plotting
        self.fig, self.ax = plt.subplots()

        # directory where plots will be saved
        self.save_dir = 'src/ar_test/ar_test/trajectory_plots'  # Change this as needed
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)  # Create the directory if it doesn't exist

    def process_trajectory(self, msg):
        self.get_logger().info(f'Received cubic trajectory coefficients: a0={msg.a0}, a1={msg.a1}, a2={msg.a2}, a3={msg.a3}, t0={msg.t0}, tf={msg.tf}')
        
        if msg.t0 >= msg.tf:
            self.get_logger().error(f"Invalid time range: t0={msg.t0}, tf={msg.tf}. t0 should be less than tf.")
            return

        # generate time values
        t = np.linspace(msg.t0, msg.tf, 100)

        # compute position, velocity, and acceleration
        p = msg.a0 + msg.a1 * t + msg.a2 * t**2 + msg.a3 * t**3
        v = msg.a1 + 2 * msg.a2 * t + 3 * msg.a3 * t**2
        a = 2 * msg.a2 + 6 * msg.a3 * t

        self.get_logger().info(f"First position values: {p[:5]}")
        self.get_logger().info(f"First velocity values: {v[:5]}")
        self.get_logger().info(f"First acceleration values: {a[:5]}")

        # publish messages
        position_msg = Float64MultiArray(data=p.tolist())
        velocity_msg = Float64MultiArray(data=v.tolist())
        acceleration_msg = Float64MultiArray(data=a.tolist())

        self.position_publisher.publish(position_msg)
        self.velocity_publisher.publish(velocity_msg)
        self.acceleration_publisher.publish(acceleration_msg)

        self.get_logger().info("Published position, velocity, and acceleration.")

        # plot and save the trajectory
        self.save_plot(t, p, v, a)

    def save_plot(self, t, p, v, a):
        self.ax.clear()
        
        # plot the position, velocity, and acceleration
        self.ax.plot(t, p, label='Position', color='b')
        self.ax.plot(t, v, label='Velocity', color='g')
        self.ax.plot(t, a, label='Acceleration', color='r')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Magnitude')
        self.ax.set_title('Cubic Trajectory')
        self.ax.legend()

        # create plot file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'trajectory_plot_{timestamp}.png'
        filepath = os.path.join(self.save_dir, filename)
        
        # save plot
        plt.savefig(filepath)
        self.get_logger().info(f"Plot saved to {filepath}")
        self.fig.canvas.draw()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
