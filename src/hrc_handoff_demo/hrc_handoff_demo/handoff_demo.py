"""MoveIt-driven scripted handoff demo.

State machine through a fixed sequence (INIT → LEFT pick → handover →
RIGHT drop → reset → loop). Each ARM step requests a plan from
MoveGroup via the /move_action action server, then replays the
returned RobotTrajectory by publishing /joint_states. Gripper steps
are direct linear interpolation (single joint, no IK risk).

Why MoveIt for arms but not grippers:
  - Arms have self-collision risk during interp; OMPL + the SRDF
    disabled_collisions table handles that correctly, hand-tuned
    poses do not.
  - Grippers are passive joints in MoveIt's planning view (no IK,
    not part of any chain group) — animating them directly is the
    standard pattern.
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import random
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import (
    Constraints, JointConstraint, MotionPlanRequest, RobotState,
    PlanningOptions, PositionIKRequest, PositionConstraint,
    OrientationConstraint, BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Quaternion


# ---------------------- joint names + poses -------------------------
ARM_JOINT_SUFFIXES = ['shoulder_pan_joint', 'shoulder_lift_joint',
                      'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                      'wrist_3_joint']
LEFT_ARM_JOINTS  = [f'left_{s}'  for s in ARM_JOINT_SUFFIXES]
RIGHT_ARM_JOINTS = [f'right_{s}' for s in ARM_JOINT_SUFFIXES]
ALL_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + ['left_finger_joint',
                                                    'right_finger_joint']

# Reference joint pose: home/ready stance for either arm.
READY = (0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0)

# ============== Cartesian targets (runtime IK via /compute_ik) ==========
# Each target is the WORLD-frame Pose for the named arm's tool0. At
# runtime, /compute_ik is called with the target + a seed joint state;
# the resulting joint values are then planned to via the standard
# JointConstraint path. Seeded IK is essential — KDL returns wrap-around
# / elbow-flipped solutions without a sensible seed.

# Cube on the (new flush) table: top at z=0.78, cube edge 0.05 → cube
# center z = 0.805. The 2F-85 mounts TCP at +0.135 m along tool0's
# z-axis, so for a z-down approach tool0 sits 0.135 m above TCP.
TABLE_CENTER_XY = (0.0, -1.175)
CUBE_TOP_Z      = 0.78
CUBE_SIZE       = 0.05
TCP_TO_TOOL0_DZ = 0.135

# PICKUP_L_TARGET removed — replaced by make_pickup_target(node) below
# which tracks the random spawn pose published on /scene/cube_pose.

# Handover geometry: LEFT presents the cube from ABOVE (gripper z-down,
# vertical approach), RIGHT receives HORIZONTALLY from the +Y side
# (gripper z=-Y, perpendicular approach). The two arms' tool0 frames
# sit on different axes — left above the cube, right beside it — so
# their bodies do NOT overlap, and both TCPs converge at the cube
# center.
HANDOVER_CUBE = (0.45, 0.0, 1.10)

HANDOVER_L_TARGET = {
    'group':  'left_arm',
    'prefix': 'left_',
    # LEFT tool0: 0.135 m above cube center, gripper z-down.
    # rpy=(π, 0, 0) — base z-down orientation (no yaw). The "flipped"
    # 90° wrist-roll is implicit: this is the *unrotated* z-down
    # gripper. Fingers close along world Y.
    'pos': (HANDOVER_CUBE[0], HANDOVER_CUBE[1], HANDOVER_CUBE[2] + TCP_TO_TOOL0_DZ),
    'rpy': (math.pi, 0.0, 0.0),
    'seeds': [
        (0.50, -1.10, 1.50, -1.60,  0.00,    0.00),
        (0.50, -1.10, 1.50, -1.60, -1.57,    0.00),
        (0.30, -1.30, 1.40, -1.50,  1.57,    0.00),
        (0.60, -1.30, 1.60, -1.85, -1.57,    1.57),
    ],
}

HANDOVER_R_TARGET = {
    'group':  'right_arm',
    'prefix': 'right_',
    # RIGHT tool0: 0.135 m in +Y from cube center.
    # rpy=(π/2, π/2, 0) — z=-Y horizontal approach + 90° pitch flip.
    # Fingers close along world X. Combined with LEFT's Y-closing
    # fingers, the finger boxes are X-separated at the cube → no
    # finger interference under the perpendicular grasp.
    'pos': (HANDOVER_CUBE[0],
            HANDOVER_CUBE[1] + TCP_TO_TOOL0_DZ,
            HANDOVER_CUBE[2]),
    'rpy': (math.pi / 2, math.pi / 2, 0.0),
    'seeds': [
        (-0.60, -1.20, 1.40, -1.50, 0.0000,  0.0000),
        (-0.60, -1.20, 1.40, -1.50, 0.0000,  1.5708),
        (-0.60, -1.20, 1.40, -1.50, 0.0000, -1.5708),
        (-0.40, -1.00, 1.30, -1.30, 0.0000,  0.0000),
        (-0.80, -1.30, 1.50, -1.70, 0.0000,  0.0000),
        (-0.60, -1.20, 1.40, -1.50, 3.1416,  0.0000),
    ],
}

# --------- 4-section stacking patterns (cabinet v2) -----------------
# Cabinet outer X[-0.05,+0.55], Y[+1.00,+1.20], Z[+0.90,+1.55].
# Vertical divider at X=+0.25, horizontal divider at Z=+1.22.
# Sections (inside dimensions):
#   TL Blue  : X[-0.03,+0.23] Z[+1.23,+1.53] — row of 5 cubes
#   TR Yellow: X[+0.27,+0.53] Z[+1.23,+1.53] — pyramid 3-2-1 (6)
#   BL Pink  : X[-0.03,+0.23] Z[+0.92,+1.21] — tower of 4
#   BR Purple: X[+0.27,+0.53] Z[+0.92,+1.21] — 2x2 base + 1 top (5)

CUBE_Y_INSIDE = +1.10   # cube centre Y, mid-depth in cabinet (Y in [+1.02,+1.18])

# IK seeds for horizontal +Y approach (gripper-z = +Y_world).
_STACK_SEEDS_HORIZ = [
    (-math.pi / 2, -1.30, 1.40, -1.65, 0.0000, 0.0000),
    (-math.pi / 2, -1.00, 1.50, -2.00, 1.5708, 0.0000),
    (-math.pi / 2, -1.10, 1.20, -1.70, 0.0000, 0.0000),
    (-1.5,         -1.20, 1.40, -1.65, 0.0000, 0.0000),
    (-2.0000,      -1.30, 1.40, -1.65, 0.0000, 0.0000),
    (-0.5000,      -1.00, 1.40, -1.50, 0.0000, 0.0000),
    (-0.80,        -1.10, 1.50, -1.90, 1.5708, 0.0000),
]

# IK seeds for z-DOWN approach (gripper-z = -Z_world). Right arm
# reaches into the cabinet from above. These mirror the WORKING
# LEFT pickup seeds (which use the same gripper z-down orientation),
# with pan negated since the right arm reaches +Y instead of -Y.
_STACK_SEEDS_ZDOWN = [
    ( 0.50, -1.20, 1.60, -1.85, -1.5708, 1.5708),
    ( 0.50, -1.20, 1.60, -1.85,  1.5708, 0.0),
    ( 0.80, -1.00, 1.50, -1.50,  0.0,    0.0),
    ( 1.0,  -1.20, 1.50, -1.85, -1.5708, 0.0),
    ( 1.20, -1.10, 1.40, -1.60,  0.0,    0.0),
    ( 1.5708, -1.30, 1.50, -1.70, 0.0,   0.0),
]

# ----- per-section target generators (i = 0-indexed placement count) -----
# Pitch is 0.055 m (cube edge 0.050 + 5 mm visible gap) on horizontal
# rows. The gap models the physical truth that a 2F-85 gripper
# (~24 mm finger pad) can't drop a cube flush against its neighbour;
# there must be space for the fingers + a brief retreat motion.
# Z stride on stacked patterns is 0.052 m (cube edge + 2 mm gap) so
# the cubes don't visually merge.
ROW_PITCH   = 0.055
STACK_STEP  = 0.052

def _blue_target(i):
    """TL Blue line of 5. Cubes spaced by ROW_PITCH, left-to-right, Z=+1.255."""
    return (0.00 + ROW_PITCH * i, CUBE_Y_INSIDE, +1.255)

def _yellow_target(i):
    """TR Yellow 3-2-1 pyramid (6 cubes). Centre X=+0.40, base Z=+1.255."""
    if i < 3:
        return (0.35 + ROW_PITCH * i, CUBE_Y_INSIDE, +1.255)
    if i < 5:
        # mid layer sits in the valley between two base cubes
        return (0.35 + ROW_PITCH * 0.5 + ROW_PITCH * (i - 3),
                CUBE_Y_INSIDE, +1.255 + STACK_STEP)
    return (0.35 + ROW_PITCH, CUBE_Y_INSIDE, +1.255 + 2 * STACK_STEP)

def _pink_target(i):
    """BL Pink tower of 4. Single X=+0.10, Z stride STACK_STEP, base Z=+0.945."""
    return (+0.10, CUBE_Y_INSIDE, +0.945 + STACK_STEP * i)

def _purple_target(i):
    """BR Purple 2x2 base + 1 on top. Base (i=0..3) at Z=+0.945;
    top (i=4) at Z=+0.945+STACK_STEP centred."""
    half = ROW_PITCH * 0.5
    if i < 4:
        x_offsets = [(-half, -half), (+half, -half),
                     (-half, +half), (+half, +half)]
        dx, dy = x_offsets[i]
        return (+0.40 + dx, CUBE_Y_INSIDE + dy, +0.945)
    return (+0.40, CUBE_Y_INSIDE, +0.945 + STACK_STEP)

SECTION_TARGETS = {
    'blue':   _blue_target,
    'yellow': _yellow_target,
    'pink':   _pink_target,
    'purple': _purple_target,
}
SECTION_CAPACITY = {'blue': 5, 'yellow': 6, 'pink': 4, 'purple': 5}
COLOUR_CYCLE = ['blue', 'yellow', 'pink', 'purple']

# Placement strategy: the right arm reaches a pose just OUTSIDE the
# cabinet front face (Y=+1.00) — tool0 at Y=PLACE_OUTSIDE_Y — releases
# the cube, then the cube-marker animation slides the cube from the
# fingertip into its section target over NUDGE_DURATION_S. Keeping the
# arm out of the cabinet sidesteps every finger-vs-panel collision the
# tight upper sections create. The slide doubles as the "nudge to
# settle next to the previous neighbour" the user asked for.
PLACE_OUTSIDE_Y = +0.85
PLACE_FINGERTIP_Y = PLACE_OUTSIDE_Y + TCP_TO_TOOL0_DZ   # = +0.985
NUDGE_DURATION_S = 0.55

# IK seeds for horizontal +Y placement: right-arm pan ≈ +π/2 so the
# wrist faces the cabinet, with a handful of lift/elbow variations
# covering the full Z range across all four sections (+0.945…+1.36).
_PLACE_SEEDS = [
    ( math.pi / 2, -1.20, 1.40, -1.60,  0.0,    0.0),
    ( math.pi / 2, -0.80, 1.10, -1.40,  0.0,    0.0),
    ( math.pi / 2, -1.40, 1.60, -1.80,  0.0,    0.0),
    ( 1.40,        -1.20, 1.40, -1.60,  0.0,    0.0),
    ( 1.60,        -1.20, 1.40, -1.60,  0.0,    0.0),
    ( math.pi / 2, -1.20, 1.40, -1.60,  1.5708, 0.0),
    ( math.pi / 2, -1.20, 1.40, -1.60, -1.5708, 0.0),
    ( math.pi / 2, -1.00, 1.30, -1.50,  0.0,    0.0),
]

def make_place_target(node):
    """Per-cube placement: tool0 at (cube_X, PLACE_OUTSIDE_Y, cube_Z)
    with horizontal +Y orientation. The arm stays outside the cabinet;
    the cube animates into its section target after release."""
    colour = node.current_colour
    i = node.placed_per_colour[colour]
    tx, _ty, tz = SECTION_TARGETS[colour](i)
    return {
        'group':  'right_arm',
        'prefix': 'right_',
        'pos':    (tx, PLACE_OUTSIDE_Y, tz),
        'rpy':    (0.0, -math.pi / 2, -math.pi / 2),
        'seeds':  _PLACE_SEEDS,
    }

# Hard-coded RIGHT-arm joint poses for each section.
# These were picked to be REACHABLE (JointConstraint planning ALWAYS
# succeeds for reachable joint targets — no IK gymnastics). The exact
# tool0 position differs from the section target, but the cube marker
# is re-anchored to the section's world position at RIGHT_RELEASE, so
# the cube ends up exactly where we want it visually.
SECTION_RIGHT_POSE = {
    # Pan, lift, elbow, wrist1, wrist2, wrist3 — tested in MoveIt
    # Motion Planning panel to confirm reachability.
    # Top sections — arm reaches into the upper cabinet compartment.
    'blue':   ( 1.40, -0.90, 1.50, -0.50, -1.5708, 0.0),
    'yellow': ( 1.20, -0.90, 1.50, -0.50, -1.5708, 0.0),
    # Bottom sections — arm reaches into the lower compartment.
    'pink':   ( 1.50, -1.30, 1.80, -1.20, -1.5708, 0.0),
    'purple': ( 1.30, -1.30, 1.80, -1.20, -1.5708, 0.0),
}
COLOUR_RGBA = {
    'blue':   (0.20, 0.55, 0.85, 1.0),
    'yellow': (0.95, 0.85, 0.20, 1.0),
    'pink':   (0.95, 0.55, 0.75, 1.0),
    'purple': (0.55, 0.30, 0.75, 1.0),
}

# Random table spawn bounds (used to pick where each new cube appears).
SPAWN_X_MIN = -0.20
SPAWN_X_MAX = +0.20
SPAWN_Y_MIN = -1.40
SPAWN_Y_MAX = -0.95
SPAWN_Z     = +0.805
SPAWN_ANTI_REPEAT = 0.06   # m — new spawn must be ≥ this far from recent ones
SPAWN_HISTORY     = 5


def _ik_target_for_pos(cx, cy, cz):
    """Build the IK target dict for the right arm reaching cube centre
    at world (cx, cy, cz).

    Approach choice by section height:
      Top sections (cz ≥ +1.20): z-DOWN approach (gripper drops into
        the upper compartment from above, tool0 above the cube).
        Easier to reach with the new cabinet at Y=+1.00.
      Bottom sections (cz < +1.20): horizontal +Y approach (gripper
        z-axis into the cabinet through the front face)."""
    if cz >= 1.20:
        # Z-DOWN: gripper-Z = -Z_world. tool0 is 0.135 m above TCP.
        # rpy matches the working LEFT pickup orientation.
        return {
            'group':  'right_arm',
            'prefix': 'right_',
            'pos': (cx, cy, cz + TCP_TO_TOOL0_DZ),
            'rpy': (math.pi, 0.0, math.pi / 2),
            'seeds': _STACK_SEEDS_ZDOWN,
        }
    # Bottom sections: horizontal +Y. tool0 is 0.135 m in -Y from TCP.
    return {
        'group':  'right_arm',
        'prefix': 'right_',
        'pos': (cx, cy - TCP_TO_TOOL0_DZ, cz),
        'rpy': (0.0, -math.pi / 2, -math.pi / 2),
        'seeds': _STACK_SEEDS_HORIZ,
    }

def make_stack_target(node):
    """LEGACY: only used if someone still references arm_ik_dyn for
    stacking. The new SEQ uses 'arm_dyn' with SECTION_RIGHT_POSE
    instead (joint-target → bulletproof planning, no IK retries).
    """
    colour = node.current_colour
    i = node.placed_per_colour[colour]
    cx, cy, cz = SECTION_TARGETS[colour](i)
    return _ik_target_for_pos(cx, cy, cz)

def section_joint_target_fn(node):
    """Return (RIGHT_ARM_JOINTS, target_joint_tuple) for the current
    colour's hard-coded above-cabinet pose. Used by the new 'arm_dyn'
    step kind in the SEQ."""
    return RIGHT_ARM_JOINTS, SECTION_RIGHT_POSE[node.current_colour]


def section_pose_target_fn(node):
    """Return (link_name, pos_xyz, quat_wxyz) for the current colour's
    section, using a Cartesian pose target so MoveIt's IK can find a
    workable joint config.

    Always horizontal +Y approach: gripper enters from the cabinet's
    open front (-Y side) with local +Z = +Y_world. The cabinet has a
    closed top panel at Z=+1.53–1.55, so z-down from above is
    geometrically blocked even for the upper sections — OMPL fails
    with "Unable to sample any valid states for goal tree" because
    the wrist + forearm can't fit through the closed top.

    For rpy=(0,-π/2,-π/2) the quaternion is (w,x,y,z) = (0.5,-0.5,-0.5,-0.5).
    """
    colour = node.current_colour
    i = node.placed_per_colour[colour]
    cx, cy, cz = SECTION_TARGETS[colour](i)
    quat = (0.5, -0.5, -0.5, -0.5)
    tool_pos = (cx, cy - TCP_TO_TOOL0_DZ, cz)
    return ('right_tool0', tool_pos, quat)

def make_pickup_target(node):
    """LEFT-arm pickup target uses the demo-owned cube spawn pose
    (set per-cycle by the round-robin scheduler)."""
    cube_x, cube_y, cube_z = node.cube_spawn_pose
    return {
        'group':  'left_arm',
        'prefix': 'left_',
        # LEFT picks z-down; tool0 0.135 m above cube centre.
        'pos': (cube_x, cube_y, cube_z + TCP_TO_TOOL0_DZ),
        'rpy': (math.pi, 0.0, math.pi / 2),
        'seeds': [
            (-0.50, -1.20, 1.60, -1.85, -1.5708, 1.5708),
            (-0.50, -1.20, 1.60, -1.85,  1.5708, 0.0),
            (-0.80, -1.00, 1.50, -1.50,  0.0,    0.0),
        ],
    }

# Gripper values — finger_joint geometry per robotiq_2f85.xacro:
#   finger_a starts at y=-0.042, axis +Y → moves TOWARD center as joint grows
#   finger_b starts at y=+0.042, axis -Y → moves TOWARD center as joint grows
# Therefore:
#   finger_joint = 0.0   → fingers at ±0.042 → 84 mm gap → OPEN
#   finger_joint = 0.040 → fingers at ±0.002 → 4 mm gap  → fully CLOSED
# The URDF macro's "lower=0.0 (closed)" comment is WRONG; we set the
# constants to match actual geometry. For a 5 cm cube, "closed to touch
# the cube" = each finger at ±0.025 → joint value = 0.042 - 0.025 = 0.017.
GRIP_OPEN   = 0.0      # fingers spread, clears the cube during approach
GRIP_CLOSED = 0.017    # fingers at the cube surface (5 cm cube), no over-drive

# --------------------------- cube -----------------------------------
# A 5 cm cube. The cube is rendered directly by this demo via Marker
# messages. While free (on the table / placed on a shelf), the marker
# uses world frame at the demo-tracked pose. While grasped, the
# marker is anchored to the gripper_base TF frame with an offset to
# the finger centre (gripper-Z = 0.0875). RViz then renders the cube
# via the SAME TF chain that places the gripper visual — zero skew,
# no MuJoCo physics needed for visualisation.

# Cube offset inside gripper_base when grasped: gripper-Z = 0.135
# (the TCP frame's distance from gripper_base). This is the exact
# point the LEFT arm plans to reach to pick up the table cube, so
# the cube doesn't visually jump when the grasp activates.
CUBE_GRIPPER_LOCAL = (0.0, 0.0, 0.135)

# State → (parent_frame, offset_xyz). Labels not listed default to
# the world frame at the cube's stored spawn/place pose.
CUBE_PARENT_BY_LABEL = {
    'LEFT_GRASP':     ('left_gripper_base',  CUBE_GRIPPER_LOCAL),
    'LEFT_HANDOVER':  ('left_gripper_base',  CUBE_GRIPPER_LOCAL),
    'RIGHT_APPROACH': ('left_gripper_base',  CUBE_GRIPPER_LOCAL),
    'RIGHT_GRASP':    ('right_gripper_base', CUBE_GRIPPER_LOCAL),
    'LEFT_RELEASE':   ('right_gripper_base', CUBE_GRIPPER_LOCAL),
    'LEFT_RETREAT':   ('right_gripper_base', CUBE_GRIPPER_LOCAL),
    'RIGHT_STACK':    ('right_gripper_base', CUBE_GRIPPER_LOCAL),
    'RIGHT_RELEASE':  ('right_gripper_base', CUBE_GRIPPER_LOCAL),
}

# Sequence step kinds:
#   ('arm',     group, joints_list, target_joint_tuple, label)
#   ('arm_ik',  ik_target_dict, label) — runtime IK via /compute_ik
#   ('grip',    joint_name, target_value, label, duration_s)
#
# Full pick → handover → stack cycle:
#   1. LEFT picks cube from the side table.
#   2. LEFT presents at midline, gripper vertical.
#   3. RIGHT meets LEFT with HORIZONTAL gripper (perpendicular grasp).
#   4. RIGHT closes; LEFT opens; LEFT retreats to READY.
#   5. RIGHT carries cube to the cabinet's lower shelf (gripper points
#      INTO the cabinet so tool0 stays within UR5e reach).
#   6. RIGHT releases on the shelf; RIGHT retreats to READY.
#   7. Cycle loops; cube reset to table for the next pass.
SEQ = [
    ('arm',     'left_arm',  LEFT_ARM_JOINTS,  READY,         'INIT_LEFT'),
    ('arm',     'right_arm', RIGHT_ARM_JOINTS, READY,         'INIT_RIGHT'),
    ('grip',    'left_finger_joint',  GRIP_OPEN,   'OPEN_L',  0.8),
    ('grip',    'right_finger_joint', GRIP_OPEN,   'OPEN_R',  0.8),

    # ---- LEFT: pick + present ----
    # LEFT_REACH uses dynamic target tracking the random spawn pose.
    ('arm_ik_dyn', make_pickup_target,              'LEFT_REACH'),
    ('grip',    'left_finger_joint',  GRIP_CLOSED, 'LEFT_GRASP',  0.8),
    ('arm_ik',  HANDOVER_L_TARGET,                  'LEFT_HANDOVER'),

    # ---- Simultaneous perpendicular handover ----
    ('arm_ik',  HANDOVER_R_TARGET,                  'RIGHT_APPROACH'),
    ('grip',    'right_finger_joint', GRIP_CLOSED, 'RIGHT_GRASP', 0.8),
    ('grip',    'left_finger_joint',  GRIP_OPEN,   'LEFT_RELEASE', 0.8),
    ('arm',     'left_arm',  LEFT_ARM_JOINTS,  READY,        'LEFT_RETREAT'),

    # ---- RIGHT: stack cube into its colour-section ----
    # Per-cube IK to a pose JUST OUTSIDE the cabinet (Y=+0.85, while
    # cabinet front is at Y=+1.00). The arm never enters the cabinet,
    # so finger-vs-panel collisions are impossible. After release the
    # cube animates from the gripper fingertip into the section over
    # NUDGE_DURATION_S — this is what the user sees as "a little nudge
    # to get close to previous neighbour after stacking."
    #
    # Earlier attempts (arm_pose_dyn with a pose-target inside the
    # cabinet; arm_dyn with a fixed joint pose per colour) both
    # failed: tight cabinet clearances meant either right_finger_b
    # clipped the side panels, or the legacy SECTION_RIGHT_POSE
    # joints (tuned for the v1 cabinet at Y=+1.55) put the goal in
    # collision with the v2 cabinet at Y=+1.00.
    ('arm_ik_dyn',  make_place_target,             'RIGHT_STACK'),
    ('grip',    'right_finger_joint', GRIP_OPEN,  'RIGHT_RELEASE', 0.6),
    ('arm',     'right_arm', RIGHT_ARM_JOINTS, READY,        'RIGHT_RETREAT'),
]

PUBLISH_RATE_HZ = 100.0   # joint_state / trajectory replay tick rate.
                          # Drives /tf rate, which drives mujoco_runner's
                          # mocap-target rate; bumping from 30 → 100 Hz
                          # eliminates visible stair-step on cube tracking.
PLANNING_TIME_S = 15.0       # longer budget for the flipped-grip handover
PLANNING_ATTEMPTS = 30       # more retries before OMPL gives up
PLAN_DELAY_BETWEEN_STEPS_S = 0.4   # tiny pause so the demo reads


class HandoffMoveItDemo(Node):
    def __init__(self):
        super().__init__('handoff_moveit_demo')

        self.cb_group = ReentrantCallbackGroup()

        # Current robot state (initialized to READY + grippers open).
        self.current = dict.fromkeys(ALL_JOINTS, 0.0)
        for j, v in zip(LEFT_ARM_JOINTS,  READY): self.current[j] = v
        for j, v in zip(RIGHT_ARM_JOINTS, READY): self.current[j] = v
        self.current['left_finger_joint']  = GRIP_OPEN
        self.current['right_finger_joint'] = GRIP_OPEN

        self.pub_js    = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_state = self.create_publisher(String,     '/handoff/state', 10)
        # Cube visualization is handled by hrc_physics' /scene/cube_marker
        # — this demo no longer publishes the cube marker. The cube IS
        # the MuJoCo physics body; we only consume its pose via
        # /scene/cube_pose and its contact pairs via /scene/contacts.
        self.current_label = 'INIT_LEFT'

        self.action_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.cb_group)

        # /compute_ik client for runtime IK on Cartesian step targets.
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.cb_group)

        # Cube marker output (no physics). Markers re-parented to
        # gripper_base TF during grasp; world frame otherwise.
        self.pub_cube_marker = self.create_publisher(
            Marker, '/scene/cube_marker', 10)
        # Cube state owned by the demo:
        #   cube_spawn_pose : (x, y, z) world where the current cube
        #                     starts on the table this cycle
        #   placed_cubes    : list of (colour, (x,y,z)) for every cube
        #                     that has been released into a section
        #   spawn_history   : last few (x, y) spawn positions for
        #                     anti-repeat
        self.cube_spawn_pose = (0.0, -1.175, 0.805)
        self.placed_cubes = []
        self.spawn_history = []
        self._rng = random.Random(42)
        # Once a cube has been released into a section, hide the active
        # cube marker through RIGHT_RETREAT so it doesn't flicker back
        # to the spawn pose. Reset when a new cube is spawned.
        self.active_cube_visible = True

        # Publish current state at 30 Hz — this is what move_group reads
        # to determine the start state for planning AND what the
        # MuJoCo runner reads (via robot_state_publisher → TF) to
        # drive the gripper mocap bodies.
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_current,
                          callback_group=self.cb_group)

        # Defer start until move_action is up.
        self.step_idx = 0
        self.startup_timer = self.create_timer(
            1.0, self._wait_for_server, callback_group=self.cb_group)
        self.busy = False

        # Round-robin scheduling state. Track per-colour placement
        # count; current_colour is set at each cycle start.
        self.colour_idx = 0   # which entry in COLOUR_CYCLE is up next
        self.placed_per_colour = {c: 0 for c in COLOUR_CYCLE}
        self.current_colour = COLOUR_CYCLE[0]

    # ----------------------- publishing -----------------------------
    def _publish_current(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ALL_JOINTS
        js.position = [self.current[n] for n in ALL_JOINTS]
        self.pub_js.publish(js)

    def _publish_state(self, label):
        self.current_label = label
        m = String(); m.data = f'{self.step_idx}:{label}'
        self.pub_state.publish(m)

    # ----------------------- cube marker -----------------------------
    def _publish_cube_markers(self):
        """Publish ONE marker for the active cube (re-parented to the
        gripper TF during grasp, or world-frame at the spawn pose
        when free) plus a persistent ghost marker for every cube
        already placed in a section."""
        now = self.get_clock().now().to_msg()

        # Active cube — id=0 in the 'active' namespace.
        if self.current_colour and self.active_cube_visible:
            r, g, b, a = COLOUR_RGBA[self.current_colour]
            m = Marker()
            m.ns = 'cube_active'; m.id = 0
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.r = float(r); m.color.g = float(g)
            m.color.b = float(b); m.color.a = float(a)
            parent = CUBE_PARENT_BY_LABEL.get(self.current_label)
            if parent is not None:
                # Anchor to gripper TF; zero stamp → RViz uses latest TF.
                frame, off = parent
                m.header.stamp = RosTime()
                m.header.frame_id = frame
                m.pose.position.x, m.pose.position.y, m.pose.position.z = off
                m.pose.orientation.w = 1.0
            else:
                # World frame — at the demo-owned spawn pose.
                m.header.stamp = now
                m.header.frame_id = 'world'
                x, y, z = self.cube_spawn_pose
                m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
                m.pose.orientation.w = 1.0
            self.pub_cube_marker.publish(m)

        # Placed cubes — each gets its own id in the 'cube_placed' ns.
        # Each entry is (colour, drop_pos, target_pos, t_placed); during
        # the NUDGE_DURATION_S window after t_placed, the rendered
        # position eases from drop_pos → target_pos using a smoothstep.
        t_now = time.monotonic()
        for idx, entry in enumerate(self.placed_cubes):
            colour, drop_pos, target_pos, t_placed = entry
            dt = t_now - t_placed
            if dt >= NUDGE_DURATION_S:
                x, y, z = target_pos
            else:
                u = max(0.0, dt / NUDGE_DURATION_S)
                e = u * u * (3.0 - 2.0 * u)
                x = drop_pos[0] + e * (target_pos[0] - drop_pos[0])
                y = drop_pos[1] + e * (target_pos[1] - drop_pos[1])
                z = drop_pos[2] + e * (target_pos[2] - drop_pos[2])
            r, g, b, a = COLOUR_RGBA[colour]
            m = Marker()
            m.ns = 'cube_placed'; m.id = idx
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.r = float(r); m.color.g = float(g)
            m.color.b = float(b); m.color.a = float(a)
            m.header.stamp = now; m.header.frame_id = 'world'
            m.pose.position.x = float(x); m.pose.position.y = float(y); m.pose.position.z = float(z)
            m.pose.orientation.w = 1.0
            self.pub_cube_marker.publish(m)

    def _pick_random_spawn(self):
        """Random pose on the pickup table, ≥ SPAWN_ANTI_REPEAT from
        recent spawns."""
        for _ in range(50):
            x = self._rng.uniform(SPAWN_X_MIN, SPAWN_X_MAX)
            y = self._rng.uniform(SPAWN_Y_MIN, SPAWN_Y_MAX)
            if all((x - hx) ** 2 + (y - hy) ** 2 >= SPAWN_ANTI_REPEAT ** 2
                   for hx, hy in self.spawn_history):
                break
        self.spawn_history.append((x, y))
        if len(self.spawn_history) > SPAWN_HISTORY:
            self.spawn_history.pop(0)
        return (x, y, SPAWN_Z)

    def _choose_next_colour(self):
        """Round-robin pick: find the next colour in COLOUR_CYCLE that
        still has cubes remaining. Returns None if all sections are full."""
        for offset in range(len(COLOUR_CYCLE)):
            idx = (self.colour_idx + offset) % len(COLOUR_CYCLE)
            c = COLOUR_CYCLE[idx]
            if self.placed_per_colour[c] < SECTION_CAPACITY[c]:
                self.colour_idx = (idx + 1) % len(COLOUR_CYCLE)
                return c
        return None

    def _spawn_next_cube(self, colour):
        """Pick a random table pose for the next cube. Demo owns the
        cube state — no physics RPC needed."""
        self.cube_spawn_pose = self._pick_random_spawn()
        self.active_cube_visible = True
        self.get_logger().info(
            f'spawning {colour} cube at ({self.cube_spawn_pose[0]:+.3f}, '
            f'{self.cube_spawn_pose[1]:+.3f}, {self.cube_spawn_pose[2]:+.3f})')

    def _full_reset(self):
        """Clear placed cubes and per-colour cursors so the round
        restarts visually clean."""
        self.placed_cubes = []
        self.placed_per_colour = {c: 0 for c in COLOUR_CYCLE}
        self.colour_idx = 0
        self.spawn_history.clear()

    # ----------------------- startup --------------------------------
    def _wait_for_server(self):
        if self.action_client.server_is_ready():
            self.startup_timer.cancel()
            self.get_logger().info('move_action ready; starting round-robin demo')
            self.current_colour = self._choose_next_colour() or COLOUR_CYCLE[0]
            self._spawn_next_cube(self.current_colour)
            # Marker publisher tick at 100 Hz — matches the joint_state
            # rate so the gripper-anchored cube doesn't lag behind the
            # moving arm. Zero-stamp markers ask RViz to look up TF at
            # render time, but RViz only re-renders the marker pose on
            # each new marker message, so the marker rate sets the
            # visible cube update rate.
            self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_cube_markers,
                              callback_group=self.cb_group)
            self._kick_next_step()
        else:
            self.get_logger().info('waiting for /move_action ...')

    # ----------------------- sequence -------------------------------
    def _kick_next_step(self):
        if self.busy:
            return
        self.busy = True
        # A new step begins — any IK-retry chain from the previous step
        # is irrelevant. If THIS step is arm_ik, _plan_arm_ik below
        # will create a fresh chain.
        self._ik_attempt_state = None
        if self.step_idx >= len(SEQ):
            self.step_idx = 0
            nxt = self._choose_next_colour()
            if nxt is None:
                self.get_logger().info(
                    'round complete (all 20 placed) — full reset, looping')
                self._full_reset()
                nxt = self._choose_next_colour() or COLOUR_CYCLE[0]
            self.current_colour = nxt
            self.get_logger().info(
                f'cycle complete — next colour: {nxt} '
                f'(placed {self.placed_per_colour[nxt]}/{SECTION_CAPACITY[nxt]})')
            self._spawn_next_cube(nxt)
        step = SEQ[self.step_idx]
        kind = step[0]
        if kind == 'arm':
            _, group, joints, target, label = step
            self._publish_state(label)
            self.get_logger().info(f'[{self.step_idx}] {label} → plan {group}')
            self._plan_arm(group, joints, target)
        elif kind == 'arm_ik':
            _, ik_target, label = step
            self._publish_state(label)
            self.get_logger().info(
                f'[{self.step_idx}] {label} → compute_ik {ik_target["group"]}')
            self._plan_arm_ik(ik_target)
        elif kind == 'arm_ik_dyn':
            _, target_fn, label = step
            ik_target = target_fn(self)
            self._publish_state(label)
            self.get_logger().info(
                f'[{self.step_idx}] {label} ({self.current_colour} '
                f'#{self.placed_per_colour[self.current_colour]+1}) → '
                f'compute_ik {ik_target["group"]}')
            self._plan_arm_ik(ik_target)
        elif kind == 'arm_dyn':
            _, target_fn, label = step
            joints, target = target_fn(self)
            self._publish_state(label)
            self.get_logger().info(
                f'[{self.step_idx}] {label} ({self.current_colour} '
                f'#{self.placed_per_colour[self.current_colour]+1}) → plan right_arm')
            self._plan_arm('right_arm', joints, target)
        elif kind == 'arm_pose_dyn':
            _, target_fn, label = step
            link, pos, quat = target_fn(self)
            self._publish_state(label)
            self.get_logger().info(
                f'[{self.step_idx}] {label} ({self.current_colour} '
                f'#{self.placed_per_colour[self.current_colour]+1}) → '
                f'pose-target {link} at ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})')
            self._plan_arm_pose('right_arm', link, pos, quat)
        else:  # 'grip'
            _, joint, target, label, dur = step
            self._publish_state(label)
            self.get_logger().info(f'[{self.step_idx}] {label}')
            self._animate_gripper(joint, target, dur)

    def _advance(self):
        # At RIGHT_RELEASE, commit the cube to its section target so
        # it persists as a ghost marker on the cabinet.
        if self.current_label == 'RIGHT_RELEASE':
            colour = self.current_colour
            i = self.placed_per_colour[colour]
            tx, ty, tz = SECTION_TARGETS[colour](i)
            # Drop position is wherever the gripper fingertip ended
            # up — outside the cabinet at PLACE_FINGERTIP_Y. The cube
            # then slides forward into the section over NUDGE_DURATION_S.
            drop_pos = (tx, PLACE_FINGERTIP_Y, tz)
            target_pos = (tx, ty, tz)
            self.placed_cubes.append(
                (colour, drop_pos, target_pos, time.monotonic()))
            self.placed_per_colour[colour] += 1
            # Cube is now a ghost at the section target — stop drawing
            # the active marker so it doesn't reappear at the spawn
            # pose during RIGHT_RETREAT.
            self.active_cube_visible = False
        self.step_idx += 1
        self.busy = False
        # Re-enter on the executor with a tiny delay.
        t = self.create_timer(
            PLAN_DELAY_BETWEEN_STEPS_S,
            lambda: (t.cancel(), self._kick_next_step()),
            callback_group=self.cb_group)

    # ----------------------- runtime IK helper ---------------------
    @staticmethod
    def _quat_from_rpy(r, p, y):
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        cp, sp = math.cos(p / 2), math.sin(p / 2)
        cy, sy = math.cos(y / 2), math.sin(y / 2)
        return (sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy)

    @staticmethod
    def _normalize_to_seed(sol, seed):
        """Wrap each joint into the [-π, π] window nearest the seed so
        downstream joint-goal planning doesn't sweep the long way
        through a 2π-equivalent solution that KDL likes to return."""
        out = []
        for s, r in zip(sol, seed):
            while s - r > math.pi:
                s -= 2 * math.pi
            while s - r < -math.pi:
                s += 2 * math.pi
            out.append(s)
        return tuple(out)

    # ---- Build a single IK request body. -----------------------------
    def _build_ik_request(self, ik_target, seed, joint_names, avoid_coll):
        prefix = ik_target['prefix']
        tx, ty, tz = ik_target['pos']
        qx, qy, qz, qw = self._quat_from_rpy(*ik_target['rpy'])

        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(tx)
        pose.pose.position.y = float(ty)
        pose.pose.position.z = float(tz)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        ik_req = PositionIKRequest()
        ik_req.group_name = ik_target['group']
        ik_req.ik_link_name = f'{prefix}tool0'
        ik_req.pose_stamped = pose
        ik_req.timeout.sec = 2
        ik_req.avoid_collisions = avoid_coll
        seed_js = JointState()
        seed_js.name = joint_names
        seed_js.position = list(seed)
        ik_req.robot_state = RobotState()
        ik_req.robot_state.joint_state = seed_js
        ik_req.robot_state.is_diff = True

        req = GetPositionIK.Request()
        req.ik_request = ik_req
        return req

    # ---- Cartesian step entry point: multi-seed retry + fallback. ----
    def _plan_arm_ik(self, ik_target):
        """Compute IK for a Cartesian tool0 target, then chain into the
        standard joint-goal planning path.

        Strategy:
          1. Iterate over ik_target['seeds'] with avoid_collisions=True.
             First success wins.
          2. If every seed fails collision-aware, retry the FIRST seed
             with avoid_collisions=False (last-resort kinematic-only
             solution; the downstream OMPL plan will still avoid
             collisions during the motion).
          3. If even that fails, log clearly and RESET the SEQ so the
             cube doesn't get fake-grasped by a stationary arm.
        """
        joint_names = [f'{ik_target["prefix"]}{s}' for s in ARM_JOINT_SUFFIXES]
        seeds = ik_target['seeds']
        self._ik_attempt_state = {
            'target':      ik_target,
            'seeds':       seeds,
            'seed_idx':    0,
            'joint_names': joint_names,
            'collisions':  True,        # phase 1: try each seed with avoid_collisions
            'fallback':    False,       # phase 2: any-collisions fallback
        }
        self._dispatch_ik_attempt()

    def _dispatch_ik_attempt(self):
        st = self._ik_attempt_state
        seed = st['seeds'][st['seed_idx']]
        req = self._build_ik_request(
            st['target'], seed, st['joint_names'],
            avoid_coll=st['collisions'])
        phase = 'kinematic-only' if not st['collisions'] else f'seed#{st["seed_idx"]}'
        self.get_logger().info(f'  IK attempt [{phase}]')
        future = self.ik_client.call_async(req)
        future.add_done_callback(self._on_ik_attempt_done)

    def _on_ik_attempt_done(self, future):
        st = self._ik_attempt_state
        res = future.result()
        ok = (res is not None) and (res.error_code.val == 1)
        if not ok:
            ec = ('no response' if res is None
                  else f'error_code={res.error_code.val}')
            self.get_logger().warn(f'  IK miss ({ec})')
            # advance to next seed or fallback
            st['seed_idx'] += 1
            if st['seed_idx'] < len(st['seeds']):
                self._dispatch_ik_attempt()
                return
            if st['collisions']:
                # all collision-aware seeds failed; try kinematic-only
                self.get_logger().warn(
                    '  all seeds failed with avoid_collisions=True; '
                    'falling back to kinematic-only')
                st['collisions'] = False
                st['fallback'] = True
                st['seed_idx'] = 0
                self._dispatch_ik_attempt()
                return
            # Total failure. Don't fake the step — restart the cycle.
            self.get_logger().error(
                'IK could not be solved for this target. Aborting cycle '
                'and restarting from step 0 to avoid a fake-grasp.')
            self._abort_and_restart()
            return

        # Success.
        sol_by_name = dict(zip(res.solution.joint_state.name,
                                res.solution.joint_state.position))
        try:
            raw = tuple(sol_by_name[n] for n in st['joint_names'])
        except KeyError as e:
            self.get_logger().error(f'IK response missing {e} — restarting')
            self._abort_and_restart()
            return
        seed = st['seeds'][st['seed_idx']]
        normalized = self._normalize_to_seed(raw, seed)
        tag = ('fallback' if st['fallback'] else f'seed#{st["seed_idx"]}')
        self.get_logger().info(
            f'  IK joints [{tag}]: '
            f'({normalized[0]:+.3f}, {normalized[1]:+.3f}, '
            f'{normalized[2]:+.3f}, {normalized[3]:+.3f}, '
            f'{normalized[4]:+.3f}, {normalized[5]:+.3f})')
        # Mark that the upcoming _plan_arm is part of the IK chain so
        # planning failures route back into _on_ik_attempt_done to try
        # the next seed (instead of aborting outright).
        st['in_planning'] = True
        self._plan_arm(st['target']['group'], st['joint_names'], normalized)

    def _ik_chain_active(self):
        """True iff we are currently inside an arm_ik step whose IK
        result has been handed to _plan_arm. Planning failures during
        this window should retry the next IK seed, not abort."""
        st = getattr(self, '_ik_attempt_state', None)
        return bool(st and st.get('in_planning'))

    def _ik_chain_try_next(self, reason):
        """Called from the planning-result callbacks when planning
        fails. Advance the IK seed chain and dispatch the next IK
        attempt — without re-running steps before this one."""
        st = self._ik_attempt_state
        st['in_planning'] = False
        st['seed_idx'] += 1
        if st['seed_idx'] < len(st['seeds']):
            self.get_logger().warn(
                f'  planning miss ({reason}); trying next IK seed')
            self._dispatch_ik_attempt()
            return
        if st['collisions']:
            self.get_logger().warn(
                f'  planning miss ({reason}); falling back to '
                'kinematic-only IK + replan')
            st['collisions'] = False
            st['fallback'] = True
            st['seed_idx'] = 0
            self._dispatch_ik_attempt()
            return
        self.get_logger().error(
            f'  planning miss ({reason}) and all seeds exhausted; '
            'aborting cycle.')
        self._abort_and_restart()

    def _abort_and_restart(self):
        """Reset to the start of the SEQ so a failed IK doesn't leave
        the cube held by a stationary arm in the next cycle."""
        # Snap state back to initial (both arms at READY, grippers open,
        # cube on table).
        for j, v in zip(LEFT_ARM_JOINTS,  READY): self.current[j] = v
        for j, v in zip(RIGHT_ARM_JOINTS, READY): self.current[j] = v
        self.current['left_finger_joint']  = GRIP_OPEN
        self.current['right_finger_joint'] = GRIP_OPEN
        self._publish_state('ABORTED_RESET')
        # step_idx = -1 so the subsequent _advance() lands on SEQ[0].
        self.step_idx = -1
        self._advance()

    # ----------------------- arm via MoveIt -------------------------
    # Cabinet middle divider top is at world Z=+1.23 (per
    # hrc_description). To keep the right wrist clear of it during
    # placement and nudge inside the cabinet, constrain wrist_2 below
    # Z=+1.18 (5 cm margin). Only applied to RIGHT_STACK and
    # NUDGE_SLIDE — other steps (handover, retreat, table-pick) need
    # the full workspace.
    # Wrist keepout disabled — was needed for the v1 cabinet's middle
    # divider issue. With v2 (cabinet at Y=+1.00 + hard-coded joint
    # poses per section), standard collision checking is enough.
    _WRIST_KEEPOUT_LABELS = set()
    _WRIST_KEEPOUT_Z_MAX  = 1.18
    _WRIST_KEEPOUT_LINK   = 'right_wrist_2_link'

    def _build_wrist_keepout_constraint(self):
        """Return a path Constraints object that confines
        right_wrist_2_link's origin to Z <= 1.18 throughout the
        trajectory. Implemented as a single large box (X,Y unbounded
        in practice; Z capped at 1.18) — the wrist_2 link origin must
        stay inside the box."""
        pc = PositionConstraint()
        pc.header.frame_id = 'world'
        pc.link_name = self._WRIST_KEEPOUT_LINK
        # The box extends Z from -1.0 (well below the floor) to
        # _WRIST_KEEPOUT_Z_MAX. Width and depth in X,Y are 20 m, which
        # is effectively unbounded for our workcell.
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        z_height = self._WRIST_KEEPOUT_Z_MAX + 1.0
        box.dimensions = [20.0, 20.0, z_height]
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = (self._WRIST_KEEPOUT_Z_MAX - 1.0) / 2.0
        pose.orientation.w = 1.0
        vol = BoundingVolume()
        vol.primitives.append(box)
        vol.primitive_poses.append(pose)
        pc.constraint_region = vol
        pc.weight = 1.0
        c = Constraints()
        c.position_constraints.append(pc)
        return c

    def _plan_arm_pose(self, group_name, link_name, pos_xyz, quat_wxyz,
                       pos_tol=0.02, ori_tol=0.30):
        """Plan to a Cartesian pose using MoveIt's built-in IK +
        sampling. Sends Position+Orientation constraints; MoveIt
        finds joint values that satisfy both, then plans a path.
        Far more robust than compute_ik (KDL) + JointConstraint,
        because OMPL can sample MULTIPLE IK branches."""
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group_name
        req.num_planning_attempts = PLANNING_ATTEMPTS
        req.allowed_planning_time = PLANNING_TIME_S
        req.max_velocity_scaling_factor = 0.40
        req.max_acceleration_scaling_factor = 0.40

        c = Constraints()

        # Position constraint: tool0 (or specified link) within a small
        # sphere around the target position.
        pc = PositionConstraint()
        pc.header.frame_id = 'world'
        pc.link_name = link_name
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(pos_tol)]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2]))
        pose.orientation.w = 1.0
        bv = BoundingVolume()
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)

        # Orientation constraint with generous tolerance — gives MoveIt
        # room to pick a wrist configuration that solves IK.
        oc = OrientationConstraint()
        oc.header.frame_id = 'world'
        oc.link_name = link_name
        q = Quaternion()
        q.w, q.x, q.y, q.z = (
            float(quat_wxyz[0]), float(quat_wxyz[1]),
            float(quat_wxyz[2]), float(quat_wxyz[3]))
        oc.orientation = q
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        req.goal_constraints = [c]
        req.start_state = RobotState()
        req.start_state.is_diff = True

        goal.request = req
        opts = PlanningOptions()
        opts.plan_only = True
        opts.replan = False
        opts.look_around = False
        goal.planning_options = opts

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _plan_arm(self, group_name, joint_names, target_tuple):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group_name
        req.num_planning_attempts = PLANNING_ATTEMPTS
        req.allowed_planning_time = PLANNING_TIME_S
        req.max_velocity_scaling_factor = 0.40
        req.max_acceleration_scaling_factor = 0.40

        cs = Constraints()
        for jn, jv in zip(joint_names, target_tuple):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(jv)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            cs.joint_constraints.append(jc)
        req.goal_constraints = [cs]

        # NOTE on wrist-divider clearance: we rely on MoveIt's standard
        # collision checking against the per-panel cabinet collision
        # (set in hrc_description/workcell.urdf.xacro) to prevent the
        # wrist from clipping the middle divider. An earlier
        # PositionConstraint (right_wrist_2_link.z < 1.18) was tried
        # but rejected too many valid trajectories — the wrist
        # legitimately swings above Z=1.18 outside the divider's X/Y
        # footprint. Standard collision checking is both faster and
        # geometrically correct.

        # Start from current — let MoveIt read from /joint_states.
        req.start_state = RobotState()
        req.start_state.is_diff = True

        goal.request = req
        opts = PlanningOptions()
        opts.plan_only = True   # we replay manually since no controllers
        opts.replan = False
        opts.look_around = False
        goal.planning_options = opts

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self._ik_chain_active():
                self._ik_chain_try_next('goal rejected')
            else:
                self.get_logger().error(
                    'MoveGroup rejected the goal — aborting cycle.')
                self._abort_and_restart()
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        # MoveItErrorCodes.SUCCESS = 1
        if result.error_code.val != 1:
            if self._ik_chain_active():
                self._ik_chain_try_next(
                    f'error_code={result.error_code.val}')
            else:
                self.get_logger().error(
                    f'planning failed (error_code={result.error_code.val}) — '
                    'aborting cycle to avoid a fake-grasp.')
                self._abort_and_restart()
            return
        traj = result.planned_trajectory.joint_trajectory
        if not traj.points:
            if self._ik_chain_active():
                self._ik_chain_try_next('empty trajectory')
            else:
                self.get_logger().error('empty trajectory — aborting cycle')
                self._abort_and_restart()
            return
        self._replay_trajectory(traj)

    # ----------------------- trajectory replay ----------------------
    # Minimum trajectory duration. If MoveIt returns a trajectory
    # shorter than this (or with zero time-parameterisation), we
    # STRETCH every waypoint's time_from_start so the arm motion is
    # visible instead of teleporting.
    _MIN_TRAJ_DURATION_S = 1.2

    def _replay_trajectory(self, traj):
        self._traj = traj
        self._traj_t0 = time.monotonic()
        raw_end = (traj.points[-1].time_from_start.sec
                   + traj.points[-1].time_from_start.nanosec * 1e-9)
        # Stretch short trajectories to a visible duration. Linearly
        # rescale every point's time_from_start by stretch_factor.
        if raw_end < self._MIN_TRAJ_DURATION_S:
            stretch = self._MIN_TRAJ_DURATION_S / max(raw_end, 1e-3)
            self.get_logger().info(
                f'  stretching trajectory: {raw_end:.2f}s × {stretch:.2f} '
                f'→ {self._MIN_TRAJ_DURATION_S:.2f}s ({len(traj.points)} points)')
            for p in traj.points:
                t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                t *= stretch
                p.time_from_start.sec = int(t)
                p.time_from_start.nanosec = int((t - int(t)) * 1e9)
        self._traj_end_t = (traj.points[-1].time_from_start.sec
                            + traj.points[-1].time_from_start.nanosec * 1e-9)
        self._replay_timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ, self._replay_tick,
            callback_group=self.cb_group)

    def _replay_tick(self):
        # Contact-aware halt: NUDGE_SLIDE terminates the slide as soon
        # as the held cube touches anything in HALT_CONTACT_GEOMS
        # (cabinet side/back walls today; other placed cubes once
        # Phase 1B introduces the multi-cube spawn pool). This is what
        # makes the row pack tight without hard-coding a stop
        # position — physics tells the demo "you've hit the previous
        # cube" and the slide ends right there.
        now = time.monotonic() - self._traj_t0
        traj = self._traj
        if now >= self._traj_end_t:
            for i, n in enumerate(traj.joint_names):
                self.current[n] = traj.points[-1].positions[i]
            self._replay_timer.cancel()
            self._replay_timer = None
            self._advance()
            return
        # Find bracketing waypoints.
        for i in range(len(traj.points) - 1):
            t1 = (traj.points[i].time_from_start.sec
                  + traj.points[i].time_from_start.nanosec * 1e-9)
            t2 = (traj.points[i + 1].time_from_start.sec
                  + traj.points[i + 1].time_from_start.nanosec * 1e-9)
            if t1 <= now <= t2:
                a = (now - t1) / (t2 - t1) if t2 > t1 else 0.0
                for j, n in enumerate(traj.joint_names):
                    p1 = traj.points[i].positions[j]
                    p2 = traj.points[i + 1].positions[j]
                    self.current[n] = p1 + a * (p2 - p1)
                return

    # ----------------------- gripper --------------------------------
    def _animate_gripper(self, joint, target, dur):
        self._g_joint  = joint
        self._g_from   = self.current[joint]
        self._g_to     = float(target)
        self._g_t0     = time.monotonic()
        self._g_dur    = max(0.05, float(dur))
        self._g_timer  = self.create_timer(
            1.0 / PUBLISH_RATE_HZ, self._g_tick,
            callback_group=self.cb_group)

    def _g_tick(self):
        t = (time.monotonic() - self._g_t0) / self._g_dur
        if t >= 1.0:
            self.current[self._g_joint] = self._g_to
            self._g_timer.cancel()
            self._g_timer = None
            self._advance()
            return
        # Cubic ease.
        e = t * t * (3.0 - 2.0 * t)
        self.current[self._g_joint] = self._g_from + e * (self._g_to - self._g_from)


def main():
    rclpy.init()
    node = HandoffMoveItDemo()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
