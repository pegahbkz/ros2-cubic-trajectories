import rclpy
from rclpy.node import Node
from ar_interface.srv import ComputeCubicTraj

class ComputeCubicCoeffs(Node):
    def __init__(self):
        # initialize node
        super().__init__('compute_cubic_coeffs')
        # create service
        self.srv = self.create_service(ComputeCubicTraj, 'compute_cubic_traj', self.compute_callback)

    def compute_callback(self, request, response):
        self.get_logger().info(f'Received request: p0={request.p0}, pf={request.pf}, v0={request.v0}, vf={request.vf}, t0={request.t0}, tf={request.tf}')
        # get time parameters
        t0, tf = request.t0, request.tf
        # set cubic coefficients
        A = [[1, t0, t0**2, t0**3],
             [0, 1, 2*t0, 3*t0**2],
             [1, tf, tf**2, tf**3],
             [0, 1, 2*tf, 3*tf**2]]
        # set boundary conditions
        B = [request.p0, request.v0, request.pf, request.vf]

        import numpy as np
        # solve cubic coefficients
        coeffs = np.linalg.solve(A, B)
        # create response
        self.get_logger().info(f'Solved coefficients: a0={coeffs[0]}, a1={coeffs[1]}, a2={coeffs[2]}, a3={coeffs[3]}')
        response.a0, response.a1, response.a2, response.a3 = coeffs
        response.t0 = t0
        response.tf = tf
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ComputeCubicCoeffs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
