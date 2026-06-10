#!/usr/bin/env python3
"""IK helper for re-tuning handoff_demo poses against the current URDF.

Calls /compute_ik for a list of tool0 target poses and prints joint
tuples in the order handoff_demo.py expects:
  (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)

Run while move_group is up:
  ros2 launch hrc_moveit_config demo.launch.py      # term 1
  python3 src/hrc_handoff_demo/scripts/ik_helper.py # term 2

This is tooling, not part of the demo flow — kept here so it lives with
the package it serves and survives /tmp cleanup.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetStateValidity
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState


SUFFIXES = ['shoulder_pan_joint', 'shoulder_lift_joint',
            'elbow_joint',        'wrist_1_joint',
            'wrist_2_joint',      'wrist_3_joint']


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def seed(prefix, vals):
    """Seed IK with a sensible starting state. KDL is sensitive to start
    state — without a seed it tends to return elbow-flipped solutions."""
    js = JointState()
    js.name = [f'{prefix}{s}' for s in SUFFIXES]
    js.position = list(vals)
    return js


def call_ik(node, cli, group, prefix, tx, ty, tz, rpy, seed_vals):
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x = tx
    pose.pose.position.y = ty
    pose.pose.position.z = tz
    qx, qy, qz, qw = quat_from_rpy(*rpy)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    r = PositionIKRequest()
    r.group_name = group
    r.ik_link_name = f'{prefix}tool0'
    r.pose_stamped = pose
    r.timeout.sec = 2
    r.avoid_collisions = True
    r.robot_state = RobotState()
    r.robot_state.joint_state = seed(prefix, seed_vals)
    r.robot_state.is_diff = True

    req = GetPositionIK.Request()
    req.ik_request = r

    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res = fut.result()
    if res is None:
        return None, 'no response'
    if res.error_code.val != 1:
        return None, f'error_code={res.error_code.val}'
    js = res.solution.joint_state
    name_to_pos = dict(zip(js.name, js.position))
    return tuple(name_to_pos[f'{prefix}{s}'] for s in SUFFIXES), 'OK'


def call_fk(node, cli_fk, prefix, joint_vals):
    """Forward kinematics for one arm's tool0. joint_vals is a 6-tuple
    in SUFFIXES order."""
    req = GetPositionFK.Request()
    req.header.frame_id = 'world'
    req.fk_link_names = [f'{prefix}tool0']
    req.robot_state = RobotState()
    req.robot_state.joint_state = seed(prefix, joint_vals)
    req.robot_state.is_diff = True
    fut = cli_fk.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res = fut.result()
    if res is None or res.error_code.val != 1 or not res.pose_stamped:
        return None
    p = res.pose_stamped[0].pose
    return (p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)


def call_validity(node, cli_sv, group, joint_names, joint_vals):
    """Check if a joint state is collision-free + in joint limits."""
    req = GetStateValidity.Request()
    req.group_name = group
    req.robot_state = RobotState()
    js = JointState()
    js.name = list(joint_names)
    js.position = list(joint_vals)
    req.robot_state.joint_state = js
    req.robot_state.is_diff = True
    fut = cli_sv.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res = fut.result()
    if res is None:
        return None, []
    contacts = []
    for c in res.contacts:
        contacts.append(f'{c.contact_body_1} <-> {c.contact_body_2}')
    return res.valid, contacts


def main():
    rclpy.init()
    node = Node('ik_helper')
    cli    = node.create_client(GetPositionIK,    '/compute_ik')
    cli_fk = node.create_client(GetPositionFK,    '/compute_fk')
    cli_sv = node.create_client(GetStateValidity, '/check_state_validity')
    if not cli.wait_for_service(timeout_sec=15.0):
        print('compute_ik service not available', file=sys.stderr)
        sys.exit(1)
    cli_fk.wait_for_service(timeout_sec=5.0)
    cli_sv.wait_for_service(timeout_sec=5.0)

    READY = (0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0)

    # TCP-to-tool0 offset along world +Z when gripper points down:
    # the gripper macro mounts the TCP at +0.135 m along tool0's z-axis,
    # so for a z-down tool0 the TCP sits 0.135 m BELOW tool0.
    # tool0_z = TCP_z + 0.135 m.
    TCP_TO_TOOL0_DZ = 0.135

    # New table top z=0.78, cube 0.05 m → cube center z = 0.805.
    # Cube center xy at table center (0.30, -0.725).
    CUBE_X, CUBE_Y, CUBE_Z = 0.30, -0.725, 0.805
    STANDOFF_DZ = 0.040   # 4 cm above the cube before final descent

    # Seed close to a natural reach toward -Y from the left arm base:
    # shoulder_pan negative (atan2(-0.5, 0.3) ~ -1.0 rad) so the arm
    # faces the target instead of wrapping through 2pi.
    PICKUP_SEED = (-1.0, -1.0, 1.5, -1.5708, 0.0, 0.0)
    # When IK returns elbow-flip / wrap solutions, this reference picks
    # the equivalent angle nearest a "natural" reaching pose so we get
    # values in [-pi, pi].
    NORMALIZE_REF = PICKUP_SEED

    def normalize(sol, ref):
        out = []
        for s, r in zip(sol, ref):
            while s - r >  math.pi: s -= 2 * math.pi
            while s - r < -math.pi: s += 2 * math.pi
            out.append(s)
        return tuple(out)

    targets = [
        # PICKUP_L: top-down approach over the new table
        ('PICKUP_L_grasp',
         'left_arm', 'left_',
         CUBE_X, CUBE_Y, CUBE_Z + TCP_TO_TOOL0_DZ,
         (math.pi, 0.0, 0.0), PICKUP_SEED),
        ('PICKUP_L_standoff',
         'left_arm', 'left_',
         CUBE_X, CUBE_Y, CUBE_Z + STANDOFF_DZ + TCP_TO_TOOL0_DZ,
         (math.pi, 0.0, 0.0), PICKUP_SEED),
        # alt yaw in case wrist_3 picks an awkward value
        ('PICKUP_L_grasp_yaw90',
         'left_arm', 'left_',
         CUBE_X, CUBE_Y, CUBE_Z + TCP_TO_TOOL0_DZ,
         (math.pi, 0.0, math.pi / 2), PICKUP_SEED),
    ]

    print(f'\nIK results normalized to [-pi, pi] near a natural reach:')
    print(f'  (shoulder_pan, shoulder_lift, elbow, w1, w2, w3)\n')
    for label, group, prefix, tx, ty, tz, rpy, seed_vals in targets:
        sol, msg = call_ik(node, cli, group, prefix, tx, ty, tz, rpy, seed_vals)
        if sol is None:
            print(f'  {label:24s}: FAIL ({msg})')
        else:
            n = normalize(sol, NORMALIZE_REF)
            travel = sum(abs(a - b) for a, b in zip(n, READY))
            print(f'  {label:24s}: '
                  f'({n[0]:+.4f}, {n[1]:+.4f}, {n[2]:+.4f}, '
                  f'{n[3]:+.4f}, {n[4]:+.4f}, {n[5]:+.4f})  '
                  f'travel-from-READY={travel:.2f}')

    # -------- Diagnostic: FK + validity for the existing handover poses --
    print('\n--- Diagnostics for current handover poses ---')
    HANDOVER_L_CUR = (0.35, -1.20,   1.50, -1.85, 0.0, 0.0)
    HANDOVER_R_CUR = (-0.35, -1.20,  1.50, -1.85, 0.0, 1.5708)

    fk_l = call_fk(node, cli_fk, 'left_',  HANDOVER_L_CUR)
    fk_r = call_fk(node, cli_fk, 'right_', HANDOVER_R_CUR)
    if fk_l:
        print(f'  left_tool0 @ HANDOVER_L:  '
              f'pos=({fk_l[0]:+.3f}, {fk_l[1]:+.3f}, {fk_l[2]:+.3f})  '
              f'q=({fk_l[3]:+.3f}, {fk_l[4]:+.3f}, {fk_l[5]:+.3f}, {fk_l[6]:+.3f})')
    if fk_r:
        print(f'  right_tool0 @ HANDOVER_R: '
              f'pos=({fk_r[0]:+.3f}, {fk_r[1]:+.3f}, {fk_r[2]:+.3f})  '
              f'q=({fk_r[3]:+.3f}, {fk_r[4]:+.3f}, {fk_r[5]:+.3f}, {fk_r[6]:+.3f})')

    LJOINTS = [f'left_{s}'  for s in SUFFIXES]
    RJOINTS = [f'right_{s}' for s in SUFFIXES]
    valid_l, contacts_l = call_validity(node, cli_sv, 'left_arm',  LJOINTS, HANDOVER_L_CUR)
    valid_r, contacts_r = call_validity(node, cli_sv, 'right_arm', RJOINTS, HANDOVER_R_CUR)
    print(f'  HANDOVER_L valid={valid_l}, contacts={contacts_l}')
    print(f'  HANDOVER_R valid={valid_r}, contacts={contacts_r}')

    # -------- HANDOVER_R search: scan rpy x seed combinations ---------
    # Target: right tool0 at (0.45, +0.05, 1.10) — just on the +Y side
    # of the handover midline at workpiece-presentation height. Try a
    # battery of (orientation, seed) combinations to find one KDL can
    # solve, that's also collision-free.
    print('\n--- HANDOVER_R search (target 0.45, +0.05, 1.10) ---')
    tx_r, ty_r, tz_r = 0.45, 0.05, 1.10
    rpy_candidates = [
        ('+pi/2, 0, 0   (z=-Y)',        ( math.pi/2,  0.0,        0.0)),
        ('-pi/2, 0, 0   (z=+Y)',        (-math.pi/2,  0.0,        0.0)),
        ('+pi/2, 0, +pi (z=-Y, twist)', ( math.pi/2,  0.0,    math.pi)),
        ('+pi/2, 0, -pi/2',             ( math.pi/2,  0.0, -math.pi/2)),
        ('pi, 0, 0      (z=-Z, down)',  ( math.pi,    0.0,        0.0)),
        ('pi, 0, +pi/2  (down, yaw90)', ( math.pi,    0.0,  math.pi/2)),
        ('0, +pi/2, 0   (z=+X, fwd)',   ( 0.0,    math.pi/2,      0.0)),
        ('0, -pi/2, 0   (z=-X, back)',  ( 0.0,   -math.pi/2,      0.0)),
    ]
    seed_candidates = [
        ('A natural',  (-0.6, -1.2, 1.6, -1.9,  0.0,    0.0)),
        ('B w2=pi/2',  (-0.6, -1.2, 1.4, -1.5,  1.5708, 0.0)),
        ('C w2=-pi/2', (-0.6, -1.4, 1.4, -1.5, -1.5708, 0.0)),
        ('D w1=0',     (-0.5, -1.0, 1.5,  0.0,  0.0,    0.0)),
    ]
    best = None  # (travel, label, sol, valid)
    for rlabel, crpy in rpy_candidates:
        for slabel, cseed in seed_candidates:
            sol, msg = call_ik(node, cli, 'right_arm', 'right_',
                               tx_r, ty_r, tz_r, crpy, cseed)
            if sol is None:
                continue
            n = normalize(sol, cseed)
            v, contacts = call_validity(node, cli_sv, 'right_arm', RJOINTS, n)
            travel = sum(abs(a - b) for a, b in zip(n, READY))
            tag = f'{rlabel:35s} / {slabel:9s}'
            mark = 'OK' if v else 'colliding'
            print(f'  {tag} → travel={travel:5.2f} {mark}')
            if v and (best is None or travel < best[0]):
                best = (travel, rlabel, slabel, n, crpy, cseed)
    if best:
        travel, rlabel, slabel, n, crpy, cseed = best
        print(f'\n  BEST HANDOVER_R candidate:')
        print(f'    rpy_label  = {rlabel}')
        print(f'    seed_label = {slabel}')
        print(f'    rpy        = {crpy}')
        print(f'    seed       = {cseed}')
        print(f'    joints     = ({n[0]:+.4f}, {n[1]:+.4f}, {n[2]:+.4f}, '
              f'{n[3]:+.4f}, {n[4]:+.4f}, {n[5]:+.4f})')
        print(f'    travel-from-READY = {travel:.2f}')
    else:
        print('\n  No valid HANDOVER_R found in the scan. Target may be unreachable.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
