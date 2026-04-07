import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class BarnController(Node):
    def __init__(self):
        super().__init__('barn_controller')

        # ── PARAMETERS FROM LAUNCH FILE ──
        self.declare_parameter('goal_x', 10.0)
        self.declare_parameter('goal_y', 0.0)
        self.goal = np.array([
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value
        ])
        self.get_logger().info(f'Goal set to: {self.goal}')

        # ── YOUR MATLAB PARAMETERS ──
        self.alpha   = 2.0
        self.beta    = 0.05
        self.k_sym   = 0.05
        self.k2      = 0.01
        self.k3      = 10.0
        self.epsilon = 1e-16
        self.dt      = 0.05

        theta        = np.pi / 3
        self.R_perp  = np.array([[np.cos(theta), -np.sin(theta)],
                                  [np.sin(theta),  np.cos(theta)]])

        # ── STABILITY CONSTANTS (same as MATLAB) ──
        a22 = (np.sqrt((self.alpha - self.beta)**2 + 4*self.alpha**2)
               - (self.alpha - self.beta)) / 2
        self.dmax = (np.exp((self.beta - a22) / (self.alpha - a22)) *
                     (2*(self.alpha - self.beta)*a22 / (self.alpha - a22)**2))

        k11 = self.k2 + self.k_sym * np.cos(theta)
        k12 = -self.k_sym * np.sin(theta)
        self.k1 = 10*(2 * (self.dmax * (k11/2 + 1.5*np.sqrt(k11**2 + k12**2))
                   + np.exp(self.beta/self.alpha) * self.dmax
                   * np.sqrt(k11**2 + k12**2)
                   - self.dmax/2 * k11) / (self.k3 - 1))

        # ── BUMP FUNCTIONS (same as MATLAB) ──
        self.phi_avoid = lambda d: (
            (abs(d) < np.sqrt(self.alpha)) *
            np.exp((self.beta - d**2) /
                   np.maximum(self.epsilon, self.alpha - d**2))
        )
        self.phi_sym = lambda d: (
            (abs(d) < 2*np.sqrt(self.alpha)) *
            np.exp((self.beta - d**2) /
                   np.maximum(self.epsilon, 4*self.alpha - d**2))
        )

        # ── ROBOT STATE ──
        self.x         = np.array([0.0, 0.0])
        self.yaw       = 0.0
        self.obstacles = np.empty((0, 2))

        # ── ROS SUBSCRIBERS ──
        self.create_subscription(LaserScan, '/front/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry,  '/odom',       self.odom_callback,  10)

        # ── ROS PUBLISHER ──
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── CONTROL LOOP TIMER ──
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('BARN Controller started!')

    def lidar_callback(self, msg):
        """Convert LiDAR scan to obstacle positions — your x_j"""
        obstacles = []
        for k, r in enumerate(msg.ranges):
            if np.isinf(r) or np.isnan(r) or r < msg.range_min:
                continue
            angle = msg.angle_min + k * msg.angle_increment
            # convert to world frame
            ox = self.x[0] + r * np.cos(self.yaw + angle)
            oy = self.x[1] + r * np.sin(self.yaw + angle)
            obstacles.append([ox, oy])
        self.obstacles = np.array(obstacles) if obstacles else np.empty((0, 2))

    def odom_callback(self, msg):
        """Get robot current position and heading"""
        self.x[0] = msg.pose.pose.position.x
        self.x[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self.yaw = np.arctan2(siny, cosy)

    def control_loop(self):
        """Your MATLAB control law — runs every dt seconds"""

        if len(self.obstacles) == 0:
            return

        x_i    = self.x
        x_goal = self.goal
        x_j    = self.obstacles
        error  = x_i - x_goal

        # ── AVOIDANCE TERM (L1 + D adapted for single robot vs obstacles) ──
        avoid_sum = np.zeros(2)
        D_val     = 1.0

        for j in range(len(x_j)):
            diff = x_i - x_j[j]
            d    = np.linalg.norm(diff)
            if d > 0:
                phi        = self.phi_avoid(d)
                avoid_sum += phi * diff / d
                D_val     += self.k3 * np.sign(-phi)

        # ── SYMMETRY TERM (u_sym) ──
        sym_sum = np.zeros(2)
        for j in range(len(x_j)):
            diff = x_i - x_j[j]
            d    = np.linalg.norm(diff)
            if d > 0:
                sym_sum += self.phi_sym(d) * (self.R_perp @ diff)
        u_sym = self.k_sym * sym_sum

        # ── FULL CONTROL LAW: u = U_trac + U_dess ──
        U_trac = -self.k1 * D_val * error
        U_dess = self.k2 * avoid_sum + u_sym
        u      = U_trac + U_dess      # u = [u_x, u_y]

        # ── HOLONOMIC → DIFFERENTIAL DRIVE ──
        linear_x  = float(np.clip(np.linalg.norm(u), 0.0, 2.0))
        angular_z = float(np.arctan2(u[1], u[0]) - self.yaw)
        angular_z = float(np.arctan2(np.sin(angular_z), np.cos(angular_z)))

        # ── SEND TO JACKAL ──
        cmd           = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = BarnController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
