#!/usr/bin/env python3
"""
BARN Challenge Controller + Live LiDAR Visualizer
==================================================
BASE: document 21 EXACT — all parameters unchanged

TWO ADDITIONS ONLY:
    1. Back half rotation safety
       if back half (90°-270°) has obstacle within body+0.03m
       → stop rotation only
       → linear_x unchanged

    2. Cluster wall detection (angle based only)
       if left_blocked AND right_blocked AND center_blocked
       → wall between robot and goal
       → remove goal bias from gap scoring
       → commit time = 3.0s (go around properly)
       → normal behavior resumes when front clears
"""
import asyncio
import signal
import math
import threading
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose


class BarnController(Node):
    def __init__(self):
        super().__init__('barn_controller')

        self.cb_group = ReentrantCallbackGroup()

        # ── GOAL ──
        self.goal           = None
        self.goal_reached   = False
        self.goal_tolerance = 0.25

        # ── ROBOT STATE ──
        self.x          = np.array([0.0, 0.0])
        self.yaw        = 0.0
        self.odom_ready = False

        # ── LIDAR ──
        self.ranges     = None
        self.angle_min  = -math.pi
        self.angle_inc  = 0.00436
        self.range_min  = 0.0
        self.range_max  = 30.0
        self.scan_ready = False

        # ── STUCK / RECOVERY (doc 21 exact) ──
        self.last_pos        = np.array([0.0, 0.0])
        self.stuck_timer     = 0.0
        self.stuck_threshold = 1.5
        self.stuck_move_dist = 0.05
        self.recovery_mode   = False
        self.recovery_timer  = 0.0
        self.recovery_phase  = 0
        self.recovery_backup = 0.5
        self.recovery_spin   = 1.0

        # ── PARAMETERS (doc 21 exact) ──
        self.THRESHOLD   = 1
        self.ALPHA       = 1
        self.BETA        = 0.001
        self.k1          = 1.6
        self.k2          = 1.5
        self.k3          = 3.0
        self.FRONT_ANGLE = math.pi / 4

        # ── D_VAL (doc 21 exact) ──
        self.D_val_smooth = 1.0
        self.D_alpha      = 0.3

        # ── SIDE CHECK (doc 21 exact) ──
        self.SIDE_SAFE_DIST = 0.5
        self.SIDE_ANGLE_IN  = math.radians(20)
        self.SIDE_ANGLE_OUT = math.radians(90)

        # ── GAP PARAMETERS (doc 21 exact) ──
        self.GAP_RANGE_THRESH  = 1
        self.MIN_GAP_WIDTH_DEG = 20.0
        self.MIN_GAP_DEPTH     = 1.3

        # ── GAP COMMIT (doc 21 exact) ──
        self.current_gap_angle = None
        self.gap_commit_timer  = 0.0
        self.GAP_COMMIT_TIME   = 0.7

        # ── VELOCITY (doc 21 exact) ──
        self.VX_FAR     = 2.0
        self.VX_MID     = 1.75
        self.VX_CLOSE   = 1.25
        self.DIST_FAR   = 2.0
        self.DIST_CLOSE = 1.0

        self.wz_max = 1 #1.9

        # ── PASSAGE DETECTION (doc 21 exact) ──
        self.PASSAGE_DIST = 1.2
        self.WZ_GAIN_SLOW = 0.75 * self.wz_max / math.pi
        self.WZ_GAIN_FAST = 2.0 * self.wz_max / math.pi

        # ── ADDITION 1: back half rotation safety ──
        self.ROT_MARGIN = 0.03

        # ── ADDITION 2: cluster wall detection ──
        # angle based only — no distance parameters
        self.WALL_CHECK_ANGLE = math.radians(60)  # check ±60° left and right
        self.WALL_GAP_COMMIT  = 3.0               # longer commit when wall detected

        # ── VIZ STATE ──
        self.viz_lock          = threading.Lock()
        self.viz_gaps          = []
        self.viz_escape_rf     = 0.0
        self.viz_goal_rf       = 0.0
        self.viz_goal_odom_deg = 0.0
        self.viz_d_val         = 1.0
        self.viz_g             = 0.0
        self.viz_front_dist    = 9.9
        self.viz_left_dist     = 9.9
        self.viz_right_dist    = 9.9
        self.viz_in_passage    = False
        self.viz_mode          = 'WAITING'

        self._action_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.execute_goal_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.create_subscription(Odometry, '/platform/odom/filtered',
            self.odom_callback, 10, callback_group=self.cb_group)
        self.create_subscription(LaserScan, '/front/scan',
            self.lidar_callback, 10, callback_group=self.cb_group)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_timer(0.05, self._run_control, callback_group=self.cb_group)
        self.create_timer(5.0, self.keepalive_callback, callback_group=self.cb_group)
        self.get_logger().info('✅ barn_controller ready!')

    def _is_sector_blocked(self, angle_start, angle_end):
        """Check if a sector has NO gap (all beams < GAP_RANGE_THRESH)."""
        n       = len(self.ranges)
        has_gap = False
        for k in range(n):
            a = self.angle_min + k * self.angle_inc
            a = math.atan2(math.sin(a), math.cos(a))
            if angle_start <= a <= angle_end:
                if self.ranges[k] > self.GAP_RANGE_THRESH:
                    has_gap = True
                    break
        return not has_gap

    def _detect_cluster_wall(self):
        """
        Detect cluster wall by angle only.
        Check if left sector, right sector AND center are all blocked.
        No distance parameters used.
        """
        center_blocked = self._is_sector_blocked(
            -math.radians(20), math.radians(20))
        left_blocked   = self._is_sector_blocked(
            0, self.WALL_CHECK_ANGLE)
        right_blocked  = self._is_sector_blocked(
            -self.WALL_CHECK_ANGLE, 0)
        return center_blocked and left_blocked and right_blocked

    def gamma(self, d):
        norm2 = d * d
        if norm2 < self.ALPHA:
            return math.exp((self.BETA - norm2) / (self.ALPHA - norm2))
        return 0.0

    def _get_vx_max(self, front_dist):
        if front_dist > self.DIST_FAR:
            return self.VX_FAR
        elif front_dist > self.DIST_CLOSE:
            t = ((front_dist - self.DIST_CLOSE)
                 / (self.DIST_FAR - self.DIST_CLOSE))
            return self.VX_CLOSE + t * (self.VX_MID - self.VX_CLOSE)
        else:
            return self.VX_CLOSE

    def goal_callback(self, goal_request):
        self.get_logger().info('📨 Goal received!')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_goal_callback(self, goal_handle):
        # ── FIX 1: Read goal directly - NO transformation ──
        pose = goal_handle.request.pose.pose
        self.goal = np.array([pose.position.x, pose.position.y])
        
        self.get_logger().info(
            f'🎯 Goal received: x={self.goal[0]:.2f}, y={self.goal[1]:.2f}')
        
        self.goal_reached      = False
        self.recovery_mode     = False
        self.stuck_timer       = 0.0
        self.last_pos          = self.x.copy()
        self.D_val_smooth      = 1.0
        self.current_gap_angle = None
        self.gap_commit_timer  = 0.0

        feedback_msg = NavigateToPose.Feedback()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.goal = None
                self._stop_robot()
                goal_handle.canceled()
                return NavigateToPose.Result()
            dist = float(np.linalg.norm(self.x - self.goal))
            feedback_msg.distance_remaining = dist
            goal_handle.publish_feedback(feedback_msg)
            if self.goal_reached:
                # ── FIX 2: Clear goal before succeeding ──
                self.goal = None
                goal_handle.succeed()
                return NavigateToPose.Result()
            # ── FIX 3: Use proper asyncio.sleep ──
            await asyncio.sleep(0.1)
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _run_control(self):
        if not self.odom_ready or not self.scan_ready:
            return
        if self.goal is None:
            return
        if self.goal_reached:
            self._stop_robot()
            return

        dist_to_goal = float(np.linalg.norm(self.x - self.goal))
        if dist_to_goal < self.goal_tolerance:
            self.goal_reached = True
            self._stop_robot()
            self.get_logger().info('🏁 GOAL REACHED!')
            return

        dist_moved = float(np.linalg.norm(self.x - self.last_pos))
        if dist_moved > self.stuck_move_dist:
            self.stuck_timer = 0.0
            self.last_pos    = self.x.copy()
        else:
            self.stuck_timer += 0.05

        if self.stuck_timer > self.stuck_threshold and not self.recovery_mode:
            self.recovery_mode     = True
            self.recovery_timer    = 0.0
            self.recovery_phase    = 0
            self.stuck_timer       = 0.0
            self.current_gap_angle = None
            self.gap_commit_timer  = 0.0
            self.get_logger().info('⚠️  STUCK!')

        if self.recovery_mode:
            self._do_recovery()
            with self.viz_lock:
                self.viz_mode = 'RECOVERY'
            return

        # ── ERROR (doc 21 exact) ──
        e_x = float(self.goal[0] - self.x[0])
        e_y = float(self.goal[1] - self.x[1])

        goal_angle_odom = math.atan2(e_y, e_x)
        goal_angle_rf   = goal_angle_odom - self.yaw
        goal_angle_rf   = math.atan2(
            math.sin(goal_angle_rf), math.cos(goal_angle_rf)
        )

        # ── DISTANCES (doc 21 exact) ──
        front_dist = self._sector_min(-self.FRONT_ANGLE, self.FRONT_ANGLE)
        left_dist  = self._sector_min(self.SIDE_ANGLE_IN, self.SIDE_ANGLE_OUT)
        right_dist = self._sector_min(-self.SIDE_ANGLE_OUT, -self.SIDE_ANGLE_IN)

        g      = self.gamma(front_dist)
        vx_max = self._get_vx_max(front_dist)

        # ── PASSAGE DETECTION (doc 21 exact) ──
        in_passage = (left_dist < self.PASSAGE_DIST and
                      right_dist < self.PASSAGE_DIST)

        if g > 0.15:
            wz_gain = self.WZ_GAIN_FAST * 5.0
        elif in_passage:
            wz_gain = self.WZ_GAIN_SLOW
        else:
            wz_gain = self.WZ_GAIN_FAST

        # ── D_VAL (doc 21 exact) ──
        D_val_target      = 1.0 - self.k3 * g
        self.D_val_smooth = (self.D_alpha * D_val_target
                             + (1.0 - self.D_alpha) * self.D_val_smooth)
        D_val = self.D_val_smooth

        # ── ADDITION 2: cluster wall detection ──
        cluster_wall = self._detect_cluster_wall()
        if cluster_wall:
            self.get_logger().info(
                '🧱 cluster wall detected → no goal bias',
                throttle_duration_sec=2.0)

        # ── GAP ESCAPE (doc 21 exact + wall mode) ──
        escape_rf, gaps = self._find_gap_escape(
            goal_angle_odom, cluster_wall)

        if escape_rf > 0 and left_dist < self.SIDE_SAFE_DIST:
            escape_rf = -abs(escape_rf)
        elif escape_rf < 0 and right_dist < self.SIDE_SAFE_DIST:
            escape_rf = abs(escape_rf)

        escape_odom = self.yaw + escape_rf

        with self.viz_lock:
            self.viz_gaps          = gaps
            self.viz_escape_rf     = escape_rf
            self.viz_goal_rf       = goal_angle_rf
            self.viz_goal_odom_deg = math.degrees(goal_angle_odom)
            self.viz_d_val         = D_val
            self.viz_g             = g
            self.viz_front_dist    = front_dist
            self.viz_left_dist     = left_dist
            self.viz_right_dist    = right_dist
            self.viz_in_passage    = in_passage
            self.viz_mode          = 'WALL' if cluster_wall else (
                'PASSAGE' if in_passage else 'NORMAL')

        # ── CONTROL LAW (doc 21 exact) ──
        dx_sum = g * math.cos(escape_odom)
        dy_sum = g * math.sin(escape_odom)

        if g > 0.15:
            u_x = self.k2 * dx_sum
            u_y = self.k2 * dy_sum
        else:
            u_x = self.k1 * D_val * e_x + self.k2 * dx_sum
            u_y = self.k1 * D_val * e_y + self.k2 * dy_sum

        theta_desired = math.atan2(u_y, u_x)
        theta_error   = theta_desired - self.yaw
        theta_error   = math.atan2(
            math.sin(theta_error), math.cos(theta_error)
        )
        speed = math.hypot(u_x, u_y)

        if g == 0:
            linear_x = float(np.clip(
                speed * max(0.0, math.cos(theta_error)),
                0.0, vx_max
            ))
        else:
            linear_x = float(np.clip(
                speed * max(0.0, math.cos(theta_error)) * (1.0 - g),
                0.0, vx_max
            ))

        angular_z = float(np.clip(
            wz_gain * theta_error, -self.wz_max, self.wz_max
        ))

        # ── ADDITION 1: back half rotation safety ──
        if abs(angular_z) > 0.1 and not in_passage:
            n = len(self.ranges)
            back_unsafe = False
            for k in range(n):
                r = self.ranges[k]
                if r < 0.05:
                    continue
                a  = self.angle_min + k * self.angle_inc
                a  = math.atan2(math.sin(a), math.cos(a))
                if abs(a) < math.pi / 2:
                    continue
                ca = abs(math.cos(a))
                sa = abs(math.sin(a))
                if ca < 1e-9:
                    body = 0.215
                elif sa < 1e-9:
                    body = 0.254
                else:
                    body = min(0.254 / ca, 0.215 / sa)
                if r > body and r < body + self.ROT_MARGIN:
                    back_unsafe = True
                    break
            if back_unsafe:
                angular_z = 0.0
                self.get_logger().info(
                    '🛑 back half close → stop rotation',
                    throttle_duration_sec=0.5)

        # ── EMERGENCY STOP (doc 21 exact) ──
        front_e = self._sector_min(-self.FRONT_ANGLE, self.FRONT_ANGLE)
        rear_e  = self._get_rear_dist()
        if front_e < 0.15 and rear_e < 0.15:
            self.get_logger().info('🚨 EMERGENCY STOP!',
                                   throttle_duration_sec=0.5)
            self._publish_cmd(0.0, 0.0)
            return

        self.get_logger().info(
            f'[{"WALL" if cluster_wall else "PASS" if in_passage else "OPEN"}] '
            f'D={D_val:.2f} g={g:.3f} '
            f'front={front_dist:.2f}m '
            f'L={left_dist:.2f}m R={right_dist:.2f}m '
            f'esc={math.degrees(escape_rf):.0f}° '
            f'vx={linear_x:.2f} wz={angular_z:.2f}',
            throttle_duration_sec=1.0
        )
        self._publish_cmd(linear_x, angular_z)

    def _find_gap_escape(self, goal_angle_odom, cluster_wall=False):
        """
        Find best gap escape angle.
        cluster_wall=True → no goal bias (widest/deepest gap)
        cluster_wall=False → normal goal-biased scoring
        commit time increases when wall detected
        """
        commit_time = self.WALL_GAP_COMMIT if cluster_wall else self.GAP_COMMIT_TIME

        if self.current_gap_angle is not None and self.gap_commit_timer > 0:
            self.gap_commit_timer -= 0.05
            return self.current_gap_angle, []

        gaps = self._scan_gaps(full_circle=False)
        if not gaps:
            gaps = self._scan_gaps(full_circle=True)

        if not gaps:
            escape = goal_angle_odom - self.yaw
            escape = math.atan2(math.sin(escape), math.cos(escape))
            self.current_gap_angle = None
            return escape, []

        n = len(self.ranges)
        for gap in gaps:
            center_rf   = gap['center']
            center_odom = self.yaw + center_rf
            center_odom = math.atan2(
                math.sin(center_odom), math.cos(center_odom)
            )

            start_k = int((gap['start'] - self.angle_min) / self.angle_inc)
            end_k   = int((gap['end']   - self.angle_min) / self.angle_inc)
            start_k = max(0, min(n-1, start_k))
            end_k   = max(0, min(n-1, end_k))
            if start_k > end_k:
                start_k, end_k = end_k, start_k
            segment   = self.ranges[start_k:end_k+1]
            depth     = float(np.mean(segment))
            clearance = float(np.min(segment))

            dev = math.atan2(
                math.sin(center_odom - goal_angle_odom),
                math.cos(center_odom - goal_angle_odom)
            )

            if cluster_wall:
                # no goal bias → find widest deepest gap to go around
                gap['score'] = gap['width'] * (depth**2) * clearance
            else:
                # normal: prefer gap toward goal
                gap['score'] = gap['width'] * (depth**2) * math.cos(dev) * clearance

        best      = max(gaps, key=lambda x: x['score'])
        escape_rf = best['center']

        self.current_gap_angle = escape_rf
        self.gap_commit_timer  = commit_time

        return escape_rf, gaps

    def _scan_gaps(self, full_circle=False):
        n    = len(self.ranges)
        gaps = []
        in_gap          = False
        gap_start_angle = 0.0

        for k in range(n):
            angle = self.angle_min + k * self.angle_inc
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if not full_circle and abs(angle) > math.pi / 2:
                if in_gap:
                    end = self.angle_min + (k-1) * self.angle_inc
                    self._close_gap(gaps, gap_start_angle, end)
                    in_gap = False
                continue

            r = self.ranges[k]
            if r > self.GAP_RANGE_THRESH:
                if not in_gap:
                    in_gap          = True
                    gap_start_angle = angle
            else:
                if in_gap:
                    end = self.angle_min + (k-1) * self.angle_inc
                    self._close_gap(gaps, gap_start_angle, end)
                    in_gap = False

        if in_gap:
            end = self.angle_min + (n-1) * self.angle_inc
            self._close_gap(gaps, gap_start_angle, end)

        return gaps

    def _close_gap(self, gaps, start_raw, end_raw):
        start     = math.atan2(math.sin(start_raw), math.cos(start_raw))
        end       = math.atan2(math.sin(end_raw),   math.cos(end_raw))
        diff      = math.atan2(math.sin(end - start), math.cos(end - start))
        width_deg = abs(math.degrees(diff))

        if width_deg < self.MIN_GAP_WIDTH_DEG:
            return

        center = math.atan2(
            math.sin(start) + math.sin(end),
            math.cos(start) + math.cos(end)
        )

        n      = len(self.ranges)
        beam_k = int((center - self.angle_min) / self.angle_inc)
        beam_k = max(0, min(n - 1, beam_k))
        if self.ranges[beam_k] < self.MIN_GAP_DEPTH:
            return

        gaps.append({
            'center': center,
            'start':  start,
            'end':    end,
            'width':  width_deg,
            'score':  0.0
        })

    def _sector_min(self, angle_start, angle_end):
        n        = len(self.ranges)
        min_dist = self.range_max
        for k in range(n):
            a = self.angle_min + k * self.angle_inc
            a = math.atan2(math.sin(a), math.cos(a))
            if angle_start <= a <= angle_end:
                r = self.ranges[k]
                if self.range_min < r < min_dist:
                    min_dist = r
        return min_dist

    def _get_rear_dist(self):
        n         = len(self.ranges)
        rear_dist = self.range_max
        for k in range(n):
            a = self.angle_min + k * self.angle_inc
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) > math.pi - self.FRONT_ANGLE:
                r = self.ranges[k]
                if self.range_min < r < rear_dist:
                    rear_dist = r
        return rear_dist

    def _do_recovery(self):
        self.recovery_timer += 0.05
        e_x     = float(self.goal[0] - self.x[0])
        e_y     = float(self.goal[1] - self.x[1])
        goal_rf = math.atan2(e_y, e_x) - self.yaw
        goal_rf = math.atan2(math.sin(goal_rf), math.cos(goal_rf))

        if self.recovery_phase == 0:
            rear_dist = self._get_rear_dist()
            if rear_dist > 0.4:
                self._publish_cmd(-0.2, 0.0)
                self.get_logger().info(
                    f'[RECOVERY] backup {self.recovery_timer:.1f}s',
                    throttle_duration_sec=0.5)
            else:
                self.recovery_phase = 1
                self.recovery_timer = 0.0
                return
            if self.recovery_timer > self.recovery_backup:
                self.recovery_phase = 1
                self.recovery_timer = 0.0

        elif self.recovery_phase == 1:
            spin_dir = math.copysign(1.0, goal_rf)
            self._publish_cmd(0.0, spin_dir * self.wz_max)
            self.get_logger().info(
                f'[RECOVERY] spin {self.recovery_timer:.1f}s',
                throttle_duration_sec=0.5)
            if self.recovery_timer > self.recovery_spin:
                self.recovery_mode     = False
                self.recovery_timer    = 0.0
                self.recovery_phase    = 0
                self.stuck_timer       = 0.0
                self.last_pos          = self.x.copy()
                self.D_val_smooth      = 1.0
                self.current_gap_angle = None
                self.gap_commit_timer  = 0.0
                self.get_logger().info('✅ Recovery done!')

    def odom_callback(self, msg):
        self.x[0] = msg.pose.pose.position.x
        self.x[1] = msg.pose.pose.position.y
        q         = msg.pose.pose.orientation
        siny      = 2.0 * (q.w * q.z + q.x * q.y)
        cosy      = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self.yaw  = math.atan2(siny, cosy)
        if not self.odom_ready:
            self.get_logger().info(
                f'📍 pos=({self.x[0]:.2f},{self.x[1]:.2f})')
        self.odom_ready = True

    def lidar_callback(self, msg):
        ranges          = np.array(msg.ranges)
        ranges          = np.where(np.isfinite(ranges), ranges, msg.range_max)
        self.ranges     = ranges
        self.angle_min  = msg.angle_min
        self.angle_inc  = msg.angle_increment
        self.range_min  = msg.range_min
        self.range_max  = msg.range_max
        self.scan_ready = True

    def keepalive_callback(self):
        if self.goal is None:
            self.get_logger().info('⏳ Waiting for goal...')
        elif not self.goal_reached:
            dist = np.linalg.norm(self.x - self.goal)
            mode = 'RECOVERY' if self.recovery_mode else 'NORMAL'
            self.get_logger().info(
                f'[{mode}] dist={dist:.2f}m stuck={self.stuck_timer:.1f}s')

    def _publish_cmd(self, linear_x, angular_z):
        cmd                 = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x  = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def _stop_robot(self):
        self._publish_cmd(0.0, 0.0)


def run_visualizer(node):
    plt.ion()
    fig = plt.figure(figsize=(8, 8))
    ax  = fig.add_subplot(111, projection='polar')

    while rclpy.ok():
        if node.ranges is None:
            plt.pause(0.1)
            continue
        try:
            ax.clear()
            with node.viz_lock:
                ranges        = node.ranges.copy()
                angle_min     = node.angle_min
                angle_inc     = node.angle_inc
                gaps          = list(node.viz_gaps)
                escape_rf     = node.viz_escape_rf
                goal_rf       = node.viz_goal_rf
                goal_odom_deg = node.viz_goal_odom_deg
                d_val         = node.viz_d_val
                g_val         = node.viz_g
                front_dist    = node.viz_front_dist
                left_dist     = node.viz_left_dist
                right_dist    = node.viz_right_dist
                in_passage    = node.viz_in_passage
                mode          = node.viz_mode

            n = len(ranges)
            for k in range(0, n, 4):
                a = angle_min + k * angle_inc
                a = math.atan2(math.sin(a), math.cos(a))
                r = min(float(ranges[k]), 6.0)
                if abs(a) > math.pi / 2:
                    col, alp = 'lightgray', 0.15
                elif ranges[k] > node.GAP_RANGE_THRESH:
                    col, alp = 'limegreen', 0.5
                else:
                    col, alp = 'red', 0.6
                ax.plot([a, a], [0, r], color=col, alpha=alp, lw=0.8)

            for gap in gaps:
                start  = gap['start']
                end    = gap['end']
                center = gap['center']
                score  = gap.get('score', 0.0)
                beam_k = int((center - angle_min) / angle_inc)
                beam_k = max(0, min(n-1, beam_k))
                depth  = min(float(ranges[beam_k]), 6.0)
                arc    = np.linspace(start, end, 30)
                ax.fill_between(arc, 0, depth, alpha=0.3, color='lime')
                ax.plot([center, center], [0, depth], color='darkgreen', lw=2.5)
                ax.text(center, depth+0.3,
                        f'{math.degrees(center):.0f}°\ns={score:.0f}',
                        ha='center', fontsize=7,
                        color='darkgreen', fontweight='bold')

            ax.annotate('', xy=(goal_rf, 5.0), xytext=(goal_rf, 0.3),
                        arrowprops=dict(arrowstyle='->', color='blue', lw=2.5))
            ax.text(goal_rf, 5.5, 'GOAL', color='blue',
                    ha='center', fontsize=9, fontweight='bold')

            esc_col = 'magenta' if in_passage else ('orange' if mode == 'WALL' else 'red')
            ax.annotate('', xy=(escape_rf, 4.2), xytext=(escape_rf, 0.3),
                        arrowprops=dict(arrowstyle='->', color=esc_col, lw=2.5))
            ax.text(escape_rf, 4.8,
                    f'ESC {math.degrees(escape_rf):.0f}°',
                    color=esc_col, ha='center', fontsize=9, fontweight='bold')

            sa_in  = node.SIDE_ANGLE_IN
            sa_out = node.SIDE_ANGLE_OUT
            l_col  = 'red' if left_dist < node.SIDE_SAFE_DIST else 'cyan'
            r_col  = 'red' if right_dist < node.SIDE_SAFE_DIST else 'cyan'
            ax.fill_between(np.linspace(sa_in, sa_out, 10),
                            0, min(left_dist, 6.0), alpha=0.15, color=l_col)
            ax.fill_between(np.linspace(-sa_out, -sa_in, 10),
                            0, min(right_dist, 6.0), alpha=0.15, color=r_col)

            fa = node.FRONT_ANGLE
            for ang in [fa, -fa]:
                ax.axvline(x=ang, color='orange', lw=1.5, alpha=0.7, linestyle='--')

            tc = np.linspace(-math.pi, math.pi, 360)
            ax.plot(tc, [node.THRESHOLD]*360, 'orange', lw=1.0, alpha=0.4, linestyle=':')
            ax.plot(tc, [node.PASSAGE_DIST]*360, 'magenta', lw=1.0, alpha=0.3, linestyle='-.')

            title_color = 'orange' if mode == 'WALL' else (
                'magenta' if in_passage else (
                'red' if mode == 'RECOVERY' else 'black'))
            ax.set_title(
                f'[{mode}]  '
                f'D={d_val:.2f}  g={g_val:.3f}  goal={goal_odom_deg:.0f}°\n'
                f'front={front_dist:.2f}m  L={left_dist:.2f}m  R={right_dist:.2f}m',
                color=title_color, fontsize=9, pad=15
            )

            patches = [
                mpatches.Patch(color='limegreen', label='free'),
                mpatches.Patch(color='red',       label='blocked'),
                mpatches.Patch(color='lime',      label='gap'),
                mpatches.Patch(color='blue',      label='goal'),
                mpatches.Patch(color='red',       label='escape [open]'),
                mpatches.Patch(color='magenta',   label='escape [passage]'),
                mpatches.Patch(color='orange',    label='escape [wall]'),
            ]
            ax.legend(handles=patches, loc='upper right',
                      bbox_to_anchor=(1.4, 1.0), fontsize=8)

            ax.set_ylim(0, 6.5)
            ax.set_theta_zero_location('N')
            ax.set_theta_direction(-1)
            ax.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.pause(0.1)
        except Exception:
            plt.pause(0.1)


def main():
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    rclpy.init()
    node = BarnController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    try:
        run_visualizer(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
