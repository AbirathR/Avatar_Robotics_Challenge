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
