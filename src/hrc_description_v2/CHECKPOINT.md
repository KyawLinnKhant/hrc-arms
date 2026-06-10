# CHECKPOINT — hrc_description

User signed off on the **wide-pedestal / centered-mast / flush-table
layout** on 2026-06-06: *"yes now lock this scene"*. Locked.

**Re-amended 2026-06-08** (geometry unchanged): the cabinet's coarse
outer-shell collision (one solid 0.70×0.20×0.65 box) was replaced
with per-panel collision boxes mirroring the visual panels. The old
shell made the cabinet interior solid for MoveIt and blocked all
right-arm stacking. Per-panel collisions leave the interior hollow so
the right arm can reach the shelves. No visual change, no kinematic
change — only the collision representation was edited. User approved
this targeted unlock on 2026-06-08.

This supersedes both earlier 2026-06-05 locks. History of the iteration:
- Morning 2026-06-05: original front-table layout (table in front of
  pedestal, narrow 0.44 m arm separation, no cabinet).
- Afternoon 2026-06-05: side-station revision (table on -Y, cabinet on
  +Y, still narrow arm separation).
- 2026-06-06: wide-pedestal revision because the narrow arm separation
  produced reliable OMPL failures on RIGHT_APPROACH (the two arms'
  swing volumes overlapped). User asked for "more further" separation;
  also reported the table-pedestal X collision after widening; also
  asked for the mast to be centered on Y. All addressed here.

## What's locked

- `package.xml`, `CMakeLists.txt`
- `urdf/workcell.urdf.xacro` — top-level (wide pedestal + flush table
  + wall + cabinet + centered mast + D435)
- `urdf/macros/robotiq_2f85.xacro` — Robotiq 2F-85 gripper (unchanged)
- `urdf/macros/realsense_d435.xacro` — D435 (unchanged)
- `launch/view_workcell.launch.py` — visual smoke test
- `rviz/view_workcell.rviz`
- `config/initial_positions.yaml`

## Scene geometry (do not change)

World origin at floor level, Z+ up. Y+ is the right-arm side.

### Pedestal + arms — WIDER

- **Pedestal** at world origin: **0.50 × 1.80 × 0.80 m** dark grey,
  yellow safety stripe at top (Z = 0.78 → 0.80).
  X in [-0.25, +0.25], Y in [-0.90, +0.90].
- **Left arm base** at world **(0, -0.60, 0.80)** — UR5e via
  `ur_description/urdf/ur_macro.xacro`, `tf_prefix=left_`.
- **Right arm base** at world **(0, +0.60, 0.80)** — UR5e, `tf_prefix=right_`.
- **Arm separation: 1.20 m** (was 0.44 m originally → 0.80 → 1.20).
- **Robotiq 2F-85** attached to each arm's `*_tool0`. Finger stroke
  0.040 m one-sided.

### Worktable (left-arm pick station, -Y side) — FLUSH WITH PEDESTAL

- Square **0.55 × 0.55 m × 0.04 m**, top at Z = 0.78.
- Centered at world **(0, -1.175, 0)**.
- Footprint: X in [-0.275, +0.275], Y in [-1.45, -0.90].
- Sits flush with the pedestal's -Y face (table -Y edge at -0.90
  touches pedestal -Y edge). Same X centerline as pedestal (table
  overhangs pedestal X by 2.5 mm per side — visually negligible).
- Four steel legs at corner offsets (±0.225, ±0.225) from table center.
- Mass 12 kg.
- Reads as a workbench extending the mount in the -Y direction, not as
  a separate piece floating in front. This is the geometry the user
  signed off on after multiple iterations.
- Left-arm reach (base (0,-0.60,0.80), reach 0.85 m):
  - near corners (±0.275, -0.90, 0.78) → 0.41 m ✓
  - center       (0,      -1.175,0.78) → 0.58 m ✓
  - far corners  (±0.275, -1.45, 0.78) → 0.89 m → fingertip extension

### Wall + wall-mounted cabinet (right-arm place station, +Y side)

- **Back wall slab** at Y = +1.75 → +1.77 (thin), X in [-0.30, +1.10],
  Z in [0, 2.20]. `cell_wall` (light grey).
- **Cabinet outer body** at:
  - X in [-0.05, +0.65], Y in [+1.55, +1.75], Z in [+0.90, +1.55]
  - Wood-brown body (`cell_cabinet`).
- **Panels** (2 cm thick each): back at Y∈[+1.73,+1.75]; side panels
  at X = -0.05 and +0.65; top at Z = 1.53–1.55; bottom at Z = 0.90–0.92.
- **Middle divider** at Z = +1.21 → +1.23.
- **Two usable shelves**:
  - Lower: Z in [0.92, 1.21] — 0.29 m clear height
  - Upper: Z in [1.23, 1.53] — 0.30 m clear height
- **Yellow front-edge stripes** at Z = 0.918 and Z = 1.222 (Y = +1.555).
- Coarse collision: single box approximating outer shell.
- Right-arm reach (base (0,+0.60,0.80), reach 0.85 m + 0.135 m fingertip):
  - lower shelf closest (0.20, +1.55, 0.92) → 0.98 m → fingertip just covers
  - upper shelf closest (0.20, +1.55, 1.24) → 1.07 m → past fingertip
    (stacking on the upper shelf will need an inclined approach pose;
    handover is unaffected because it happens at the midline Y=0)

### Human walkway

- Gap between right-arm side of pedestal (Y = +0.90) and cabinet front
  (Y = +1.55) = **0.65 m clearance** on the floor.

### Camera mast — CENTERED BEHIND PEDESTAL

- Mast base at world **(-0.80, 0, 0)** — directly behind pedestal on
  the Y=0 midline. Symmetric clearance: dist to each arm base
  (0, ±0.60) = 1.00 m ✓ > 0.85 m UR5e reach.
- Single +X boom (no Y leg, simpler than before):
  - vertical post: local (0, 0, 0) → (0, 0, 2.10)
  - boom +X: local (0, 0, 2.10) → (+1.45, 0, 2.10)
  - camera bracket at local (+1.45, 0, 2.06) → world (0.65, 0, 2.06)
  - D435 at local (+1.45, 0, 1.97) → world (0.65, 0, 1.97)
- Diagonal brace in the X-Z plane (was Y-Z): from local (0, 0, 1.20)
  to (0.50, 0, 2.10), tilt rpy=(0, +0.507, 0).
- D435 rpy=(0, π/2, 0) so optical +Z points straight down → looks at
  world (0.65, 0). Note: the camera world XY is over the area in front
  of the pedestal; the new flush table is BEHIND that (Y=-1.175). The
  D435 wide FOV partially covers the new table; for a v2 we may want
  to add a second camera or re-aim, but the user has not asked for it.

## Hard rules

- **Pedestal width** stays 1.80 m in Y. Narrowing it brings the arms
  too close together and OMPL fails on RIGHT_APPROACH.
- **Arm separation** stays at ±0.60 in Y.
- **Worktable position** stays at world (0, -1.175, 0) — flush with
  pedestal -Y edge. Moving it forward (+X) re-introduces the "table
  floating in front" look the user explicitly rejected.
- **Cabinet front face** stays at Y = +1.55.
- **Walkway = 0.65 m** between Y = +0.90 (pedestal edge) and Y = +1.55
  (cabinet front).
- **Mast position** stays at world (-0.80, 0, 0). Off-center mast
  positions break the symmetric look the user asked for.
- **Robot mounts** use `ur_macro` from upstream `ur_description`.
  No private UR5e copy.
- **Materials defined once** at the top of `workcell.urdf.xacro`.
  Block: `cell_pedestal`, `cell_steel`, `cell_table`, `cell_safety_yellow`,
  `robotiq_dark`, `robotiq_steel`, `d435_dark`, `d435_lens`, `cell_wall`,
  `cell_cabinet`, `cell_shelf`. Macros (gripper, D435) DO NOT declare
  materials; defining a named `<material>` inside a macro that is
  instantiated more than once crashes the URDF parser.
- **JSP-GUI initial pose** is the "ready" stance (shoulder_lift = -π/2,
  elbow = +π/2, wrist_1 = -π/2). Both arms identical.
- **Description-only.** No ros2_control tags, no Gazebo plugins. Macro
  invoked with `generate_ros2_control_tag=false`.

## How to run

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/hrc_arms/install/setup.bash

ros2 launch hrc_description view_workcell.launch.py
```

You should see:
- Wide dark-grey pedestal (1.80 m in Y) with yellow safety stripe
- Two UR5e arms in ready pose, **clearly separated** (1.20 m apart)
- **Wood-brown worktable** flush against the pedestal's left (-Y) edge,
  four steel legs visible
- **Light grey back wall** on the right (+Y) side
- **Wood-brown cabinet** mounted on the wall with two open shelves
  (yellow front-edge stripes)
- **0.65 m floor gap** between right arm and cabinet (the human walkway)
- **Centered steel mast** directly behind the pedestal, simple +X boom
  reaching forward, diagonal brace tilted forward
- **Dark camera bracket + D435** hanging at the boom tip

## Observable correctness checks

- `xacro` expansion + `check_urdf` returns "Successfully Parsed XML"
- **40 links / 39 joints** in the expanded URDF
- Full UR5e chain on both arms with `left_` / `right_` tf prefix
- Root link `world` has 5 children: back_wall_link, cabinet_link,
  camera_mast_link, pedestal_link, table_link
- No table-pedestal collision (table min X = +0.275, pedestal max X = +0.25
  → only X-edge over-hangs by 2.5 mm, no overlap in collision body)
- RViz Global Status: **Ok**

## If a future task seems to require changes

Propose a SEPARATE package (e.g., `hrc_description_v2`) and confirm
with the user before touching this one. The locked-package convention
from twoarms applies here — see [[feedback-hrc-arms-isolated]] and
[[project-hrc-arms]].
