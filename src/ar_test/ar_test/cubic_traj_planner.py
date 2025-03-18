import rclpy
from rclpy.node import Node
from ar_interface.msg import CubicTrajParams, CubicTrajCoeffs
from ar_interface.srv import ComputeCubicTraj

class CubicTrajPlanner(Node):
    def __init__(self):
        # initialize node
        super().__init__('cubic_traj_planner')
        # define subscriber
        self.subscription = self.create_subscription(
            CubicTrajParams,
            'cubic_traj_params',
            self.compute_coeffs,
            10)
        # define publisher
        self.publisher = self.create_publisher(CubicTrajCoeffs, 'cubic_traj_coeffs', 10)
        # define client
        self.client = self.create_client(ComputeCubicTraj, 'compute_cubic_traj')

    def compute_coeffs(self, msg):
        self.get_logger().info(f'Received cubic traj params: p0={msg.p0}, pf={msg.pf}, v0={msg.v0}, vf={msg.vf}, t0={msg.t0}, tf={msg.tf}')
        # create service request
        req = ComputeCubicTraj.Request()
        req.p0, req.pf, req.v0, req.vf, req.t0, req.tf = msg.p0, msg.pf, msg.v0, msg.vf, msg.t0, msg.tf

        # wait for service to be available
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available')
            return

        future = self.client.call_async(req)

        # handle the future response directly
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            # check if the service call was successful
            response = future.result()
            if response:
                coeffs_msg = CubicTrajCoeffs(a0=response.a0, a1=response.a1, a2=response.a2, a3=response.a3, t0=response.t0, tf=response.tf)
                # publish the coefficients
                self.publisher.publish(coeffs_msg)
                self.get_logger().info(f'Published cubic coefficients: {coeffs_msg}')
            else:
                self.get_logger().error('Service response was empty')
        except Exception as e:
            self.get_logger().error(f'Service call failed with error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CubicTrajPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
