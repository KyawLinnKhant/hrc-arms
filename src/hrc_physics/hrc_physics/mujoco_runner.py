"""MuJoCo physics sidecar for HRC-ARMS.

Architecture:
  Plan-only motion is in hrc_handoff_demo (publishes /joint_states).
  robot_state_publisher updates the TF tree.
  This node reads TF for the gripper frames, teleports MuJoCo mocap
  bodies to match, then steps physics at 500 Hz. The cube is a free
  body that responds to gripper contact via real friction. Cube pose
  and contact pairs are published back so the demo state machine can
  close loops on real physics (e.g., halt the nudge slide when the
  cube touches its neighbour).

  /joint_states → robot_state_publisher → /tf
                                            │
                  /tf → look-up mocap targets ─┐
                                                ▼
                                       MuJoCo step (500 Hz)
                                                │
                              ┌─────────────────┼─────────────────┐
                              ▼                 ▼                 ▼
                      /scene/cube_pose   /scene/contacts   /scene/cube_marker
"""

import os
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ament_index_python.packages import get_package_share_directory

from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker

import mujoco


# Pickup-table bounds for random cube spawn. The cube top sits at the
# table top + cube_half = 0.78 + 0.025 = +0.805. We allow X across
# most of the table width (avoiding the front-edge legs by 5 cm) and
# Y over most of the depth.
TABLE_SPAWN_X_MIN = -0.20
TABLE_SPAWN_X_MAX = +0.20
TABLE_SPAWN_Y_MIN = -1.40
TABLE_SPAWN_Y_MAX = -0.95
TABLE_SPAWN_Z     = +0.805
SPAWN_MIN_DIST    = 0.06       # m — anti-repeat threshold from each of the last 5 spawns
SPAWN_HISTORY     = 5
RNG_SEED          = 42

# Stash pose for cubes not yet spawned (out of view).
CUBE_STASH_POS = (5.0, 5.0, -1.0)
CUBE_STASH_STRIDE_X = 0.10

# Cube pool per colour. Order in this list is the spawn order within
# that colour. Total = 20.
CUBE_COLOURS = {
    'blue':   (5, (0.20, 0.55, 0.85, 1.0)),   # 5 cubes — TL row of 5
    'yellow': (6, (0.95, 0.85, 0.20, 1.0)),   # 6 cubes — TR pyramid 3-2-1
    'pink':   (4, (0.95, 0.55, 0.75, 1.0)),   # 4 cubes — BL tower
    'purple': (5, (0.55, 0.30, 0.75, 1.0)),   # 5 cubes — BR 2x2 + top
}
N_CUBES = sum(n for n, _ in CUBE_COLOURS.values())  # 20


# Frames we steer mocap bodies to. Names must match the MJCF bodies
# and the URDF link names (TF frame names).
MOCAP_FRAMES = [
    'left_gripper_base',
    'left_finger_a',
    'left_finger_b',
    'right_gripper_base',
    'right_finger_a',
    'right_finger_b',
]

PHYS_HZ      = 200.0      # MuJoCo step rate
PUB_HZ_ACTIVE = 100.0     # Active-cube publish rate (TF-anchored, near-zero cost)
PUB_HZ_BULK   = 15.0      # All-cube + contacts publish rate (the heavy loop)
TF_WAIT_S    = 0.0        # NON-BLOCKING lookup: we just want the latest known
                          # transform. Blocking would starve the executor's
                          # other callbacks (TF listener, subscriptions).

# Geoms whose contact with cube_geom we surface as "shelf-style" contacts.
# These are the cabinet's interior surfaces; the cube touching one of them
# means it has landed on the shelf (used by the demo's nudge-end logic).
SHELF_GEOMS    = {'cab_bottom', 'cab_divider'}
SIDEWALL_GEOMS = {'cab_left', 'cab_right', 'cab_back'}
GRIPPER_GEOMS  = {'left_finger_a_geom', 'left_finger_b_geom',
                  'right_finger_a_geom', 'right_finger_b_geom',
                  'left_gripper_base_geom', 'right_gripper_base_geom'}


_GRIPPER_BODIES = ['left_gripper_base', 'left_finger_a', 'left_finger_b',
                   'right_gripper_base', 'right_finger_a', 'right_finger_b']


def _build_mjcf(template_path: str) -> tuple:
    """Read the MJCF template and inject N cube bodies + gripper-cube
    contact excludes. Returns (xml_string, colour_pool) where
    colour_pool is a dict {colour_name: [cube_idx,...]} mapping each
    colour to the list of cube indices it owns."""
    with open(template_path, 'r') as f:
        xml = f.read()

    cube_lines = []
    excl_lines = []
    colour_pool = {}
    i = 0
    for colour, (count, rgba) in CUBE_COLOURS.items():
        colour_pool[colour] = []
        for _ in range(count):
            x = CUBE_STASH_POS[0] + CUBE_STASH_STRIDE_X * i
            y = CUBE_STASH_POS[1]
            z = CUBE_STASH_POS[2]
            rgba_str = f'{rgba[0]:.2f} {rgba[1]:.2f} {rgba[2]:.2f} {rgba[3]:.2f}'
            cube_lines.append(
                f'    <body name="cube_{i}" pos="{x:.3f} {y:.3f} {z:.3f}">\n'
                f'      <freejoint name="cube_{i}_joint"/>\n'
                f'      <geom name="cube_{i}_geom" type="box" '
                f'size="0.025 0.025 0.025" mass="0.1" '
                f'rgba="{rgba_str}" friction="0.8 0.05 0.001"/>\n'
                f'      <inertial pos="0 0 0" mass="0.1" '
                f'diaginertia="4.17e-5 4.17e-5 4.17e-5"/>\n'
                f'    </body>')
            for grip in _GRIPPER_BODIES:
                excl_lines.append(
                    f'    <exclude body1="{grip}" body2="cube_{i}"/>')
            colour_pool[colour].append(i)
            i += 1
    xml = xml.replace('<!-- __CUBE_BODIES__ -->', '\n'.join(cube_lines))
    xml = xml.replace('<!-- __GRIPPER_CUBE_EXCLUDES__ -->',
                      '\n'.join(excl_lines))
    return xml, colour_pool


class MujocoRunner(Node):
    def __init__(self):
        super().__init__('mujoco_runner')

        mjcf_path = self.declare_parameter(
            'mjcf_path',
            os.path.join(get_package_share_directory('hrc_physics'),
                         'config', 'workpiece.xml')
        ).get_parameter_value().string_value
        self.get_logger().info(f'loading MJCF: {mjcf_path}')
        xml, self.colour_pool = _build_mjcf(mjcf_path)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data  = mujoco.MjData(self.model)
        # Per-colour next-cube-to-spawn index.
        self.colour_cursor = {c: 0 for c in self.colour_pool}
        # Anti-repeat history: last few spawn (x,y) positions.
        self._spawn_history = []
        self._rng = __import__('random').Random(RNG_SEED)

        # Pre-resolve body/geom IDs so the inner loop is cheap.
        self.mocap_ids = {}
        for frame in MOCAP_FRAMES:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, frame)
            if bid < 0:
                raise RuntimeError(f'MJCF missing mocap body: {frame}')
            mid = self.model.body_mocapid[bid]
            if mid < 0:
                raise RuntimeError(f'body {frame} is not mocap in MJCF')
            self.mocap_ids[frame] = mid

        # Per-cube IDs for the spawn pool. cube_data[i] is a dict with
        # the body id, qpos/qvel addresses, and geom name for cube_<i>.
        self.cube_data = []
        for i in range(N_CUBES):
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f'cube_{i}')
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f'cube_{i}_joint')
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f'cube_{i}_geom')
            self.cube_data.append({
                'bid': bid,
                'qadr': int(self.model.jnt_qposadr[jid]),
                'vadr': int(self.model.jnt_dofadr[jid]),
                'gid': gid,
                'geom_name': f'cube_{i}_geom',
            })
        # Index of the cube currently on the table awaiting grasp.
        # Starts at -1; advances on each /scene/reset_cube call.
        self.active_cube_idx = -1

        # Callback groups. The physics tick is mutually exclusive (must
        # not interleave with itself). Everything else is reentrant so
        # the executor can dispatch them concurrently with the tick,
        # avoiding starvation.
        self.cb_phys     = MutuallyExclusiveCallbackGroup()
        self.cb_pub      = MutuallyExclusiveCallbackGroup()
        self.cb_services = ReentrantCallbackGroup()

        # TF buffer + listener. Mutex on the buffer is internal.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Lock to guard `data` between physics tick (writer) and publish
        # callback (reader). Cheap; ~µs per acquire on this VM.
        self._data_lock = threading.Lock()

        # Publishers.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_cube     = self.create_publisher(PoseStamped, '/scene/cube_pose',  qos)
        self.pub_contacts = self.create_publisher(String,      '/scene/contacts',   qos)
        self.pub_marker   = self.create_publisher(Marker,      '/scene/cube_marker', qos)

        # Spawn-by-colour: demo publishes the colour for the next cube
        # → runner picks the next unused cube of that colour, teleports
        # it to a random spot on the pickup table (anti-repeat).
        self.create_subscription(
            String, '/scene/spawn_request', self._on_spawn_request, 10,
            callback_group=self.cb_services)
        # Full reset: send all cubes back to stash, reset cursors.
        self.create_service(Empty, '/scene/full_reset', self._on_full_reset,
                            callback_group=self.cb_services)

        # Grasp state topic: "none" | "left" | "right". The demo state
        # machine publishes this on each transition. When grasper !=
        # 'none', the cube's qpos is overridden each physics tick to
        # track the corresponding gripper TCP (mocap_pos +
        # 0.135*gripper_z_axis). Free physics resumes when grasper
        # returns to 'none'.
        self.create_subscription(
            String, '/scene/grasp_state', self._on_grasp_state, 10,
            callback_group=self.cb_services)
        self.grasp_state = 'none'

        # Initial state: cube already settled on the table from the MJCF
        # default pose. Step a few times to settle anyway.
        for _ in range(20):
            mujoco.mj_step(self.model, self.data)

        # Diagnostics state.
        self._last_tf_warn_t = 0.0
        self._step_count = 0
        self._physics_t_start = time.time()

        # Drive physics at PHYS_HZ in its own callback group. Publish in
        # a separate group so they don't block each other.
        self.create_timer(1.0 / PHYS_HZ,        self._tick,          callback_group=self.cb_phys)
        self.create_timer(1.0 / PUB_HZ_ACTIVE,  self._publish_active, callback_group=self.cb_pub)
        self.create_timer(1.0 / PUB_HZ_BULK,    self._publish_bulk,   callback_group=self.cb_pub)

        # Periodic rate diagnostic so we can see if physics is actually
        # keeping up with PHYS_HZ.
        self._diag_last_count = 0
        self._diag_last_t = time.time()
        self.create_timer(5.0, self._diag_rate, callback_group=self.cb_services)

        self.get_logger().info('mujoco_runner ready')

    def _diag_rate(self):
        now = time.time()
        dt = now - self._diag_last_t
        d_steps = self._step_count - self._diag_last_count
        rate = d_steps / dt if dt > 0 else 0.0
        self.get_logger().info(
            f'physics rate: {rate:.0f} Hz ({d_steps} steps in {dt:.1f}s, target {PHYS_HZ:.0f})')
        self._diag_last_count = self._step_count
        self._diag_last_t = now

    # -------------------- per-tick physics step --------------------
    def _tick(self):
        # Look up every mocap-target frame from TF (non-blocking — just
        # take the latest known transform). Do TF reads OUTSIDE the
        # data lock so we don't hold the lock during TF buffer access.
        any_missing = False
        new_mocap = {}
        for frame, mid in self.mocap_ids.items():
            try:
                tf = self.tf_buffer.lookup_transform(
                    'world', frame, rclpy.time.Time(),
                    timeout=RclpyDuration(seconds=TF_WAIT_S))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                any_missing = True
                continue
            t = tf.transform.translation
            r = tf.transform.rotation
            new_mocap[mid] = ((t.x, t.y, t.z), (r.w, r.x, r.y, r.z))

        if any_missing:
            now = time.time()
            if now - self._last_tf_warn_t > 5.0:
                self._last_tf_warn_t = now
                self.get_logger().warn(
                    'TF lookup missing for one or more mocap frames; mocap bodies '
                    'frozen at last-known pose until robot_state_publisher catches up')

        # Apply mocap, grasp override, and step under the data lock so
        # the publish callback never reads mid-mutation state.
        with self._data_lock:
            for mid, (pos, quat) in new_mocap.items():
                self.data.mocap_pos[mid]  = pos
                self.data.mocap_quat[mid] = quat
            self._apply_grasp_override()
            mujoco.mj_step(self.model, self.data)
            self._step_count += 1

    def _cube_rgba(self, cube_idx):
        """Map cube_idx → (r,g,b,a) tuple by the colour pool order."""
        for colour, (count, rgba) in CUBE_COLOURS.items():
            pool = self.colour_pool[colour]
            if cube_idx in pool:
                return rgba
        return (0.5, 0.5, 0.5, 1.0)

    # ----------------------- publish (fast) ------------------------
    def _publish_active(self):
        """Fast path — runs at PUB_HZ_ACTIVE. Publishes the ACTIVE
        cube. Critical: while grasped, the marker is anchored to the
        gripper TCP TF frame with IDENTITY pose AND ZERO STAMP. Zero
        stamp tells RViz "look up the latest TF for this frame at
        render time" — so the cube renders at the same TCP world
        position that the URDF gripper visual uses at the same render
        frame. This is the only way to get zero-lag tracking; world-
        frame marker poses always lag by however stale mocap_pos is
        relative to the latest TF.
        When free, marker is world-frame with the cube's qpos."""
        active_idx = self.active_cube_idx
        if active_idx < 0:
            return
        grasped_by = self.grasp_state
        with self._data_lock:
            cd = self.cube_data[active_idx]
            pos  = self.data.qpos[cd['qadr']:cd['qadr']+3].copy()
            quat = self.data.qpos[cd['qadr']+3:cd['qadr']+7].copy()
        now = self.get_clock().now().to_msg()

        # /scene/cube_pose: world-frame for downstream consumers
        ps = PoseStamped()
        ps.header.stamp = now; ps.header.frame_id = 'world'
        ps.pose.position.x = float(pos[0]); ps.pose.position.y = float(pos[1]); ps.pose.position.z = float(pos[2])
        ps.pose.orientation.w = float(quat[0]); ps.pose.orientation.x = float(quat[1])
        ps.pose.orientation.y = float(quat[2]); ps.pose.orientation.z = float(quat[3])
        self.pub_cube.publish(ps)

        r, g, b, a = self._cube_rgba(active_idx)
        m = Marker()
        m.ns = 'physics_cube'; m.id = active_idx; m.type = Marker.CUBE; m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color.r = float(r); m.color.g = float(g); m.color.b = float(b); m.color.a = float(a)
        if grasped_by in ('left', 'right'):
            m.header.stamp = RosTime()      # zero → RViz uses LATEST TF
            # Anchor to gripper_base with offset to FINGER CENTRE
            # (gripper-Z = 0.0875). TCP sits at 0.135 — past the
            # fingertips — which made the cube look 1 cm beyond the
            # grip during motion.
            m.header.frame_id = f'{grasped_by}_gripper_base'
            m.pose.position.x = 0.0; m.pose.position.y = 0.0; m.pose.position.z = 0.0875
            m.pose.orientation.w = 1.0
        else:
            m.header.stamp = now
            m.header.frame_id = 'world'
            m.pose = ps.pose
        self.pub_marker.publish(m)

    # ----------------------- publish (bulk) ------------------------
    def _publish_bulk(self):
        """Slow path — runs at PUB_HZ_BULK. Publishes every cube's
        marker (so placed cubes stay visible) and the contact list.
        These don't need 100 Hz; the placed cubes don't move and
        contact reports only drive the demo's halt logic (gone now
        in v2 simpler stacking). 15 Hz keeps RViz happy without
        hogging the executor."""
        with self._data_lock:
            cube_poses = []
            for cd in self.cube_data:
                cube_poses.append((
                    self.data.qpos[cd['qadr']:cd['qadr']+3].copy(),
                    self.data.qpos[cd['qadr']+3:cd['qadr']+7].copy()))
            ncon = self.data.ncon
            contact_geoms = [(self.data.contact[i].geom1, self.data.contact[i].geom2)
                             for i in range(ncon)]
        active_idx  = self.active_cube_idx
        active_geom = (self.cube_data[active_idx]['geom_name']
                       if active_idx >= 0 else None)
        now = self.get_clock().now().to_msg()

        # Publish every non-active cube. The active cube is handled by
        # _publish_active at higher rate.
        for i, (pos, quat) in enumerate(cube_poses):
            if i == active_idx:
                continue
            r, g, b, a = self._cube_rgba(i)
            m = Marker()
            m.ns = 'physics_cube'; m.id = i; m.type = Marker.CUBE; m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.r = float(r); m.color.g = float(g); m.color.b = float(b); m.color.a = float(a)
            m.header.stamp = now
            m.header.frame_id = 'world'
            m.pose.position.x = float(pos[0]); m.pose.position.y = float(pos[1]); m.pose.position.z = float(pos[2])
            m.pose.orientation.w = float(quat[0]); m.pose.orientation.x = float(quat[1])
            m.pose.orientation.y = float(quat[2]); m.pose.orientation.z = float(quat[3])
            self.pub_marker.publish(m)

        # Contacts: pairs involving the ACTIVE cube only.
        pairs = []
        if active_geom is not None:
            for g1, g2 in contact_geoms:
                n1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g1)
                n2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g2)
                if n1 == active_geom:
                    pairs.append(f'cube_geom:{n2}')
                elif n2 == active_geom:
                    pairs.append(f'cube_geom:{n1}')
        s = String()
        s.data = ';'.join(pairs)
        self.pub_contacts.publish(s)


    def _on_grasp_state(self, msg):
        """Update which gripper currently owns the cube.

        When 'left' or 'right', _tick will OVERRIDE the cube's qpos
        each step to track the named gripper's TCP. When 'none', the
        cube resumes free physics (gravity + contact).

        On the rising edge (none → side), we capture the cube's CURRENT
        relative pose w.r.t. the gripper TCP so the cube stays exactly
        where it is at the moment of grasp (no teleport). On the
        falling edge (side → none), we zero the cube's velocity so it
        doesn't shoot off with the gripper's last virtual velocity.

        The handover case ('left' → 'right' directly) is handled by
        the falling-then-rising edge: cube relpose to right gripper is
        captured at the moment of transition."""
        s = msg.data.strip()
        if s not in ('none', 'left', 'right'):
            self.get_logger().warn(f'unknown grasp_state: {s!r}')
            return
        if s == self.grasp_state:
            return
        prev = self.grasp_state
        self.grasp_state = s

        if s != 'none':
            # SNAP grasp: cube forced to be at the gripper TCP with
            # identity orientation relative to the gripper. Teleports
            # the cube into the gripper at the moment of activation.
            self._grasp_rel_pos  = np.array([0.0, 0.0, 0.0])
            self._grasp_rel_quat = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            # Zero velocity on release of the ACTIVE cube so it drops
            # cleanly under gravity (or stays put if it's already on a
            # support surface).
            if self.active_cube_idx >= 0:
                cd = self.cube_data[self.active_cube_idx]
                with self._data_lock:
                    self.data.qvel[cd['vadr']:cd['vadr']+6] = 0.0

        self.get_logger().info(f'grasp_state: {prev} → {s}')

    def _apply_grasp_override(self):
        """Override the ACTIVE cube's qpos each tick so it tracks the
        gripper TCP. Reads mocap_pos/mocap_quat (just-written this
        tick) for zero lag. Other cubes in the pool are NOT touched —
        they remain wherever physics put them (e.g., stacked on the
        shelf or stashed)."""
        if self.grasp_state == 'none' or self.active_cube_idx < 0:
            return
        mid = self.mocap_ids[f'{self.grasp_state}_gripper_base']
        grip_pos_w  = self.data.mocap_pos[mid].copy()
        grip_quat_w = self.data.mocap_quat[mid].copy()

        # Cube held at finger centre (gripper-Z = 0.0875), not TCP (0.135).
        tcp_local = np.array([0.0, 0.0, 0.0875])
        tcp_offset_w = np.empty(3)
        mujoco.mju_rotVecQuat(tcp_offset_w, tcp_local, grip_quat_w)
        tcp_w = grip_pos_w + tcp_offset_w

        rel_world = np.empty(3)
        mujoco.mju_rotVecQuat(rel_world, self._grasp_rel_pos, grip_quat_w)
        cube_pos_w = tcp_w + rel_world
        cube_quat_w = np.empty(4)
        mujoco.mju_mulQuat(cube_quat_w, grip_quat_w, self._grasp_rel_quat)

        cd = self.cube_data[self.active_cube_idx]
        self.data.qpos[cd['qadr']:cd['qadr']+3]   = cube_pos_w
        self.data.qpos[cd['qadr']+3:cd['qadr']+7] = cube_quat_w
        self.data.qvel[cd['vadr']:cd['vadr']+6] = 0.0

    def _pick_random_spawn_pos(self):
        """Pick a random (x, y, z) on the pickup table that is at least
        SPAWN_MIN_DIST from each of the last SPAWN_HISTORY spawn poses."""
        for _ in range(50):
            x = self._rng.uniform(TABLE_SPAWN_X_MIN, TABLE_SPAWN_X_MAX)
            y = self._rng.uniform(TABLE_SPAWN_Y_MIN, TABLE_SPAWN_Y_MAX)
            ok = all((x - hx) ** 2 + (y - hy) ** 2 >= SPAWN_MIN_DIST ** 2
                     for hx, hy in self._spawn_history)
            if ok:
                break
        self._spawn_history.append((x, y))
        if len(self._spawn_history) > SPAWN_HISTORY:
            self._spawn_history.pop(0)
        return (x, y, TABLE_SPAWN_Z)

    def _on_spawn_request(self, msg):
        """Spawn the next unused cube of the requested colour at a
        random table pose. Demo publishes the colour string ('blue',
        'yellow', 'pink', 'purple'); we look up the next index in
        that colour's pool and teleport it."""
        colour = msg.data.strip().lower()
        if colour not in self.colour_pool:
            self.get_logger().warn(f'unknown colour: {colour!r}')
            return
        pool = self.colour_pool[colour]
        cursor = self.colour_cursor[colour]
        if cursor >= len(pool):
            self.get_logger().warn(
                f'{colour} pool exhausted (cursor={cursor}, size={len(pool)})')
            return
        cube_idx = pool[cursor]
        self.colour_cursor[colour] += 1

        pos = self._pick_random_spawn_pos()
        with self._data_lock:
            cd = self.cube_data[cube_idx]
            self.data.qpos[cd['qadr']:cd['qadr']+3]   = pos
            self.data.qpos[cd['qadr']+3:cd['qadr']+7] = (1.0, 0.0, 0.0, 0.0)
            self.data.qvel[cd['vadr']:cd['vadr']+6]   = 0.0
            mujoco.mj_forward(self.model, self.data)
        self.active_cube_idx = cube_idx
        self.get_logger().info(
            f'spawned cube_{cube_idx} ({colour}) at '
            f'({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})')

    def _on_full_reset(self, request, response):
        """Send every cube back to its stash position and reset all
        colour cursors. Called by the demo when a full 20-cube round
        is complete and the cycle restarts."""
        with self._data_lock:
            for i, cd in enumerate(self.cube_data):
                x = CUBE_STASH_POS[0] + CUBE_STASH_STRIDE_X * i
                self.data.qpos[cd['qadr']:cd['qadr']+3]   = (x, CUBE_STASH_POS[1], CUBE_STASH_POS[2])
                self.data.qpos[cd['qadr']+3:cd['qadr']+7] = (1.0, 0.0, 0.0, 0.0)
                self.data.qvel[cd['vadr']:cd['vadr']+6]   = 0.0
            mujoco.mj_forward(self.model, self.data)
        for c in self.colour_cursor:
            self.colour_cursor[c] = 0
        self._spawn_history.clear()
        self.active_cube_idx = -1
        self.get_logger().info('full reset: all cubes stashed, cursors zeroed')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MujocoRunner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
