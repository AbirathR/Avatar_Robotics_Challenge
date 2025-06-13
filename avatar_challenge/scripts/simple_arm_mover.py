#!/usr/bin/env python3
import json
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetCartesianPath
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import time

from scipy import interpolate
from scipy.interpolate import splprep, splev

from visualization_msgs.msg import Marker



class ShapeExecutor(Node):
    def __init__(self, json_path: str):
        super().__init__('shape_executor')
        self.shapes = self._load_json(json_path)

        # Set up the Cartesian‐path service client
        self.cart_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        while not self.cart_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_cartesian_path...')
        # Set up the trajectory execution action client
        self.exec_client = ActionClient(self, FollowJointTrajectory,
                                        '/xarm7_traj_controller/follow_joint_trajectory')
        while not self.exec_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for trajectory executor...')

        self.marker_pub = self.create_publisher(Marker, 'shape_marker', 10)


    def _load_json(self, path: str):
        """Load and validate the JSON shape definitions."""
        with open(path, 'r') as f:
            data = json.load(f)
 
        return data['shapes']

    # @staticmethod
    # def _euler_to_rot_z(yaw: float) -> np.ndarray:
    #     """Rotation matrix for a yaw about Z."""
    #     c, s = math.cos(yaw), math.sin(yaw)
    #     return np.array([[ c, -s, 0],
    #                      [ s,  c, 0],
    #                      [ 0,  0, 1]])
    
    def _publish_shape_marker(self, waypoints):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'draw_shape'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.002   # line width in meters
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 1.0

        # Convert PoseStamped → points
        for wp in waypoints:
            m.points.append(wp.pose.position)

        self.marker_pub.publish(m)

    @staticmethod
    def _euler_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Create a rotation matrix from roll (X), pitch (Y), yaw (Z)
        using the ZYX convention (i.e. R = Rz * Ry * Rx).
        """
        # Precompute sines/cosines
        cr, sr = math.cos(roll),  math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw),   math.sin(yaw)

        # Rotation about X (roll)
        R_x = np.array([[1,  0,   0],
                        [0, cr, -sr],
                        [0, sr,  cr]])
        # Rotation about Y (pitch)
        R_y = np.array([[ cp, 0, sp],
                        [  0, 1,  0],
                        [-sp, 0, cp]])
        # Rotation about Z (yaw)
        R_z = np.array([[cy, -sy, 0],
                        [sy,  cy, 0],
                        [ 0,   0, 1]])
        # Combined
        return R_z @ R_y @ R_x


    @staticmethod
    def _quaternion_from_euler(roll: float, pitch: float, yaw: float):
        """
        Convert roll/pitch/yaw to quaternion (x,y,z,w).
        Using the standard formula.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w
    
    def _compute_bspline_pts(self,
        ctrl_pts: np.ndarray,
        degree: int = 3,
        num_samples: int = 50,
        closed: bool = False) -> np.ndarray:
        """
        Fit a degree-k B-spline through ctrl_pts (Nx2) and return
        num_samples points along it. closed=True makes it periodic.
        """
        pts = np.asarray(ctrl_pts, dtype=float)
        n_pts = pts.shape[0]
        # Must have at least k+1 points
        if n_pts < 2:
            raise ValueError("Need at least 2 points to spline")
        k = min(degree, n_pts-1)

        # Setup periodic flag (no manual duplication)
        per = bool(closed)

        try:
            # splprep wants a list of coordinate arrays
            tck, _ = splprep([pts[:,0], pts[:,1]],
                            k=k,
                            s=0,
                            per=per)
        except Exception as e:
            self.get_logger().error(f"BSpline fit failed: {e}. Falling back to straight segments.")
            # fallback: just return the original vertices uniformly sampled
            return pts if num_samples == len(pts) else \
                np.linspace(pts[0], pts[-1], num_samples)

        # Sample uniformly in parameter space
        u_fine = np.linspace(0, 1, num_samples)
        x_fine, y_fine = splev(u_fine, tck)
        return np.vstack([x_fine, y_fine]).T

    def _compute_blend_pts(self,
            keypoints: np.ndarray,
            num_samples: int = 50) -> np.ndarray:
        """
        Quadratic Bézier through P0→P2 with P1 as control:
          B(t) = (1-t)^2 P0 + 2(1-t)t P1 + t^2 P2,  t∈[0,1]
        """
        pts = np.asarray(keypoints, dtype=float)
        if pts.shape != (3,2):
            raise ValueError("Blend requires exactly 3 keypoints")
        P0, P1, P2 = pts
        t = np.linspace(0, 1, num_samples)[:,None]  # column vector
        B = (1 - t)**2 * P0 + 2*(1-t)*t * P1 + t**2 * P2
        return B  # shape (num_samples,2)

    
    def _build_waypoints(self, shape: dict) -> list[PoseStamped]:
        pos = np.array(shape['start_pose']['position'])
        roll, pitch, yaw = shape['start_pose']['orientation']
        R       = self._euler_to_rot(roll, pitch, yaw)
        qx,qy,qz,qw = self._quaternion_from_euler(roll, pitch, yaw)

        pts_2d = []
        typ = shape.get('type', 'polygon')

        if typ == 'arc':
            arc = shape['arc']
            cx, cy   = arc['center']
            r        = arc['radius']
            th0      = arc['start_angle']
            th1      = arc['end_angle']
            N        = arc.get('samples', 50)
            thetas   = np.linspace(th0, th1, N)
            pts_2d   = [(cx + r*math.cos(t), cy + r*math.sin(t)) for t in thetas]

        elif typ == 'bspline':
            bs = shape['bspline']
            ctrl = np.array(bs['control_points'])
            deg  = bs.get('degree', 3)
            N    = bs.get('samples', 50)
            closed = bs.get('closed', False)
            pts_2d = self._compute_bspline_pts(ctrl, degree=deg,
                                               num_samples=N,
                                               closed=closed).tolist()

        elif typ == 'blend':
            bl = shape['blend']
            ctrl = np.array(bl['keypoints'])
            N    = bl.get('samples', 50)
            # compute 2D blend points
            pts_2d = self._compute_blend_pts(ctrl, num_samples=N).tolist()

        else:  # polygon
            pts_2d = shape['vertices']

        # lift into 3D waypoints
        waypoints = []
        for x2, y2 in pts_2d:
            local = np.array([x2, y2, 0.0])
            gp    = pos + R.dot(local)
            p = PoseStamped()
            p.header.frame_id = 'world'
            p.pose.position.x = float(gp[0])
            p.pose.position.y = float(gp[1])
            p.pose.position.z = float(gp[2])
            p.pose.orientation.x = qx
            p.pose.orientation.y = qy
            p.pose.orientation.z = qz
            p.pose.orientation.w = qw
            waypoints.append(p)

        return waypoints
    
    def _publish_shape_marker(self, waypoints):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'draw_shape'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.002   # line width in meters
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 1.0

        # Convert PoseStamped → points
        for wp in waypoints:
            m.points.append(wp.pose.position)

        self.marker_pub.publish(m)


    def _execute_cartesian(self, waypoints: list[PoseStamped]):
        """Call the Cartesian service and then send the result to the trajectory executor."""
        # 1) prepare service request
        req = GetCartesianPath.Request()
        req.group_name      = 'xarm7'
        req.link_name       = 'link_eef'
        req.header.frame_id = 'world'
        req.max_step        = 0.005   # finer interpolation
        req.jump_threshold  = 0.0
        req.avoid_collisions = True
        req.waypoints       = [wp.pose for wp in waypoints]

        # 2) call service
        future = self.cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp.fraction < 1.0:
            self.get_logger().warn(f"Only {resp.fraction*100:.1f}% planned")

        # 3) send as a FollowJointTrajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = resp.solution.joint_trajectory
        send_fut = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle.accepted:
            self.get_logger().error("Trajectory rejected")
            return
        result_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut)
        result = result_fut.result().result
        if result.error_code == FollowJointTrajectory.Result().SUCCESSFUL:
            self.get_logger().info("Shape drawn successfully.")
        else:
            self.get_logger().error(f"Execution failed: {result.error_code}")

    def run(self):
        """Process and execute each shape in the JSON."""
        for idx, shape in enumerate(self.shapes):
            self.get_logger().info(f"Drawing shape {idx+1}/{len(self.shapes)}")
            wpts = self._build_waypoints(shape)
            self._publish_shape_marker(wpts)
            self._execute_cartesian(wpts)
            time.sleep(2.0)


def main():
    rclpy.init()
    executor = ShapeExecutor('/home/dev/dev_ws/src/avatar_challenge/scripts/shapes.json')
    executor.run()
    executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
