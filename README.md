# Avatar_Robotics_Challenge

Input shape definitions and how to pass new shapes:

New shapes can be added to the.json file present in avatar_challenge/scripts/shapes.json.
The types of shapes that can be added are: polygons, arc, bspline and blend.

## Shape definitions:
# Polygon (default)
type: omit or "polygon"
vertices: list of 2D points [[x0,y0], [x1,y1], ...]. The end-effector draws straight lines between consecutive vertices.
Start_pose:
position: [x, y, z] in meters
orientation: [roll, pitch, yaw] in radians

# Arc
type: "arc"
arc:
center: [cx, cy] center of circle in local 2D frame
radius: r in meters
start_angle: θ0 (radians)
end_angle: θ1 (radians)
samples: number of straight-line segments (integer)
start_pose: same as polygon

# B-spline
type: "bspline"
bspline:
control_points: list of 2D points
degree: spline degree (e.g 3)
samples: number of points to sample along the curve
closed: true or false for looped spline
start_pose: same as polygon

# Blend
type: "blend"
blend:
keypoints: exactly three 2D points [P0, P1, P2]
samples: number of interpolated points
start_pose: same as polygon


# Description of my approach

In the higher level loop I load the json that contains a list of different shapes to traverse. Each of the shapes is first past through a waypoint generation which gives the plan. The plan is then executed. After which we move on to the next shape. 

To identify and get an idea of the topics, I used the RQT_GRAPH, as well the list of topics, services and actions. I was able to narrow down "/xarm7_traj_controller/follow_joint_trajectory" action to be useful for controlling the robot. Although this takes in direct joint angles not the end effector pose. To calculate the inverse kinematics I used the "/compute_cartesian_path" service to calculate the IK. This is also specifically useful since I need straight lines between my waypoints. I tried other methods the caused the robot to move in curves towards corresponding waypoints which was not desirable.


Plan/Waypoint generation
Polygons
Each polygon is broken down into lines. We just need the endpoints of the line to traverse. Since the points are given in relative 2D position with respect to the start point and orientation. From the start orientation we can get the rotation matrix R. The formula to convert the 2D point to 3D coordinate is as follows:

Pos_3d = start_pos + R*2D_pose

Arc
Each arc is defined by its center, radius, and the angle range (start to end, in radians). I sample a bunch of points (default 50) along this arc using basic trigonometry to get 2D coordinates relative to the center.
These 2D points are still in the shape's local frame. So just like in the polygon case, we rotate them to 3d. The equation for different points in the arc is:
(x,y)=(cx+r⋅cos(θ), cy+r⋅sin(θ))
Bspline


This one uses control points and a degree. I chose to treat bspline curves as a way to specify smooth, flowing paths without sharp corners. The curve doesn’t necessarily pass through the control points, but they guide the general shape.
I allowed both open and closed splines (closed=True/False) and exposed the degree and sampling rate to the user. This makes it more flexible if someone wants to define loops or higher-order curves.


Blend:
To demonstrate the blend I took 3 points to form adjacent line segments. The idea is to move towards the 2nd point and switch to the third point before reaching the second point. This is done smoothly to give an arc trajectory. 

