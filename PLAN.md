# HRC-ARMS — Adaptive Dual-Arm Collaborative Workcell with Modern AI

**Standalone project. Independent of `twoarms_ws`. No shared code, URDF, or packages.**

**Target roles:** A*STAR — SIMTech ARM (Req 860), ARTC ASR (Req 627),
ARTC Process Robotisation, ARTC SafeHRC (Req 680)

**Created:** 2026-06-05
**Status:** Phase 0 (planning) — no code yet

---

## 1. One-paragraph pitch

A dual-arm collaborative workcell in ROS2 + MuJoCo where two cobots perform
a coordinated bimanual sort task while a human operator works alongside
them: the **left arm picks** cubes (random spawn positions, four colours
— blue, yellow, pink, purple) from a side worktable, **hands them
across** to the right arm in mid-air, and the **right arm stacks** them
into a **4-section task cabinet** on the +Y side. Each section has its
own stacking pattern: blue → row of 5, yellow → 3-2-1 pyramid, pink →
4-cube tower, purple → 2×2 base + top. The cabinet is pushed against
the back wall (no walkway behind it); the **human interaction zone is
the front of the pedestal on the −Y side**, near the pickup table —
that's where the human walks (along ±X), reaches into the workspace,
and may interrupt handovers. The cell uses **anticipatory** safety —
a learned LSTM predicts the operator's trajectory 1–2 seconds ahead and
the arms pre-emptively pause *before* the operator enters the
protective stop zone. Task commands are **language-conditioned**:
voice → Whisper-tiny → Phi-3-mini intent parser → action plan. Object
grounding is **open-vocabulary**: Moondream2 (1.6 B vision-language
model) runs on CPU and locates targets from natural descriptions (no
ArUco markers). The scene is dressed with **ISO-aligned safety
infrastructure**: painted SSM floor zones, light curtain, area
scanners, operator workstation, overhead signage. A research-grade
benchmark compares anticipatory vs reactive vs naive-halt baselines on
fixed scenarios with quantitative metrics. Everything runs **CPU-only,
on-device, no paid APIs.**

---

## 2. What this defends

**Thesis:** *Anticipatory + language-conditioned dual-arm HRC, fully
on-device, validated against reactive and naive-halt baselines on a
shared-task benchmark.*

**Falsifiable claims to defend at interview:**

1. Anticipatory dodging reduces task downtime by ≥ X% vs reactive halt.
2. The LSTM predictor achieves ≤ Y cm RMSE at 1 s horizon, ≤ Z cm at 2 s.
3. Moondream2 on CPU grounds the target object in ≤ N seconds at ≥ M%
   accuracy on a fixed 20-object test set.
4. Whisper-tiny + Phi-3-mini achieves ≥ P% intent-classification accuracy
   on a 20-utterance test set with vocab-constrained prompting.
5. Total CPU usage stays below Q cores during demo execution.

(Targets X/Y/Z/N/M/P/Q populated in Phase 5.)

---

## 3. Hardware platform (sim-only)

| Component | Choice | Why |
|---|---|---|
| Arms | **Dual UR5e** | ISO 10218-1 certified cobots, standard in industry, larger reach than UR3e |
| Grippers | **Robotiq 2F-85** | Industry-standard parallel gripper, ROS2 driver, force feedback hook |
| Wrist sensor | **Robotiq FT-300S** (sim model) | Real PFL story — contact-force monitoring per arm |
| Camera | **Intel RealSense D435** (sim plugin) | RGB + depth in one URDF block, the standard depth cam in robotics labs |
| Human avatar | **gbionics/human-gazebo** humanoid URDF | Articulated, walks, can have a reaching arm sub-animation |
| Workcell furniture | Side worktable (left-arm pick station, −Y) + 4-section task cabinet (right-arm place station, +Y, no walkway behind) + back wall slab. Human interaction zone is the −Y side near the table, not between pedestal and cabinet. | New layout for v2 |

If any of the above need swapping (e.g., Franka Panda for native impedance
control), decide before Phase 1.

---

## 4. Software stack

| Layer | Tool | CPU OK? |
|---|---|---|
| OS / middleware | Ubuntu 22.04, ROS2 Humble | ✓ |
| Rigid-body physics (manipulated objects only) | **MuJoCo (headless, Python API)** | ✓ Apple-Silicon native, sub-ms/step |
| Motion planning | MoveIt2 (OMPL/RRTConnect) — plan-only | ✓ |
| ML framework | PyTorch (CPU build) | ✓ |
| Vision-language | Moondream2 1.6 B (int8) | ✓ slow (~2–5 s/query) |
| Speech-to-text | whisper.cpp (tiny.en) | ✓ real-time |
| LLM intent | Phi-3-mini-128k-instruct via Ollama | ✓ ~10 tok/s |
| Trajectory prediction | Custom LSTM ~10 K params (PyTorch) | ✓ real-time |
| Object detection (fallback) | YOLOv8n via ultralytics | ✓ ~5–15 FPS |

**Architectural choice — plan-only motion + rigid-body twin on the
workpiece, not full-scene physics.** Arms are driven via MoveIt2 plan-only
(joint-state interpolation), not torque-controlled in physics. The cube
*is* physics-simulated in MuJoCo: gravity, friction, gripper-cube
contact, cube-cube contact, cube-shelf contact. This pattern keeps the
demo lightweight on a Mac VM (Gazebo Fortress + full robot dynamics is
too heavy) while still proving real contact-rich manipulation on the
hard part of the problem. Defensible at interview: *"I chose to put
the simulator effort where the failure modes are — at the contact
interface — and use planning-only for the kinematic chain."*

**Zero paid APIs. Zero GPU dependencies. Zero cloud calls at runtime.**

---

## 5. Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                    RViz visualization                                │
│   [UR5e L] [UR5e R] [D435 marker] [Operator avatar] [Workpiece]      │
│                                                                      │
│   Joint states ←── plan-only state machine (no controllers)          │
│   Cube pose    ←── MuJoCo sidecar (real contact physics)             │
└──────────────────────────────────────────────────────────────────────┘
                                  │
            /joint_states · /camera/{rgb,depth} · TF
                                  ▼
                          ┌───────────────┐
                          │  ROS2 Graph   │
                          └───────────────┘
                                  │
   ┌──────────────┬───────────────┼───────────────┬──────────────┐
   │              │               │               │              │
┌──▼──────┐ ┌─────▼─────┐ ┌───────▼────────┐ ┌────▼──────┐ ┌────▼─────┐
│  Voice  │ │   VLM     │ │  Anticipatory  │ │   Task    │ │   HRC    │
│  Whisper│ │ Perception│ │     Safety     │ │  Planner  │ │ Coordin- │
│ + Phi-3 │ │ Moondream2│ │   LSTM + zone  │ │  (state   │ │  ator    │
│ intent  │ │ + YOLO    │ │   classifier   │ │  machine) │ │  speed + │
│ parser  │ │           │ │                │ │           │ │ checkpt  │
└──┬──────┘ └─────┬─────┘ └────────┬───────┘ └────┬──────┘ └────┬─────┘
   │/task/command │/vlm/locate     │/safety/zone_predicted     │
   │              │                │                │              │
   ▼              ▼                ▼                ▼              ▼
                         ┌─────────────────┐
                         │  MoveIt2 ×2     │
                         │  (left, right)  │
                         └─────────────────┘
```

---

## 6. ROS2 package layout

```
hrc_arms/                                  ← workspace root (this folder)
├── PLAN.md                                ← THIS file
├── README.md                              ← user-facing readme (Phase 6)
├── src/
│   ├── hrc_description/                   ← workcell URDF (Phase 1, LOCKED)
│   │   ├── urdf/
│   │   │   ├── workcell.urdf.xacro        ← pedestal + arms + side table
│   │   │   │                                + wall + cabinet w/ 2 shelves
│   │   │   └── macros/
│   │   │       ├── robotiq_2f85.xacro
│   │   │       └── realsense_d435.xacro
│   │   ├── launch/view_workcell.launch.py
│   │   ├── rviz/view_workcell.rviz
│   │   ├── config/initial_positions.yaml
│   │   └── CHECKPOINT.md                  ← geometry + hard rules
│   │   (operator avatar lives in hrc_operator below, not here)
│   │
│   ├── hrc_moveit_config/                 ← MoveIt2 SRDF + OMPL (Phase 1)
│   │   └── config/
│   │       ├── hrc.srdf
│   │       ├── kinematics.yaml
│   │       ├── ompl_planning.yaml
│   │       └── joint_limits.yaml
│   │
│   ├── hrc_physics/                       ← MuJoCo cube physics sidecar (Phase 1)
│   │   ├── hrc_physics/
│   │   │   ├── mujoco_runner.py           ← loads URDF, steps physics @500Hz
│   │   │   ├── contact_reporter.py        ← /scene/contacts publisher
│   │   │   └── attach_detector.py         ← derive gripper↔cube attachment
│   │   ├── config/cube.xml                ← MuJoCo MJCF for the workpiece
│   │   └── launch/physics.launch.py
│   │
│   ├── hrc_handoff_demo/                  ← scripted handoff + raster stack (Phase 1)
│   │   └── (cube state comes from /scene/cube_pose, not marker reparenting)
│   │
│   ├── hrc_safety/                        ← zones, e-stop, watchdog (Phase 2)
│   │   ├── hrc_safety/
│   │   │   ├── zone_classifier.py         ← 4-zone SSM (CLEAR/Y/O/R)
│   │   │   ├── safety_monitor.py          ← pre-flight + watchdog + e-stop
│   │   │   ├── stack_light.py
│   │   │   └── force_monitor.py           ← PFL grip-force hook
│   │   ├── config/safety_limits.yaml
│   │   └── test/                          ← pytest
│   │
│   ├── hrc_scene/                         ← ISO-aligned scene infra (Phase 2)
│   │   ├── hrc_scene/
│   │   │   ├── floor_zones.py             ← painted SSM rings
│   │   │   ├── light_curtain.py           ← vertical breach plane
│   │   │   ├── scanner_cones.py           ← 2D LIDAR coverage sectors
│   │   │   └── workstation_props.py       ← bench, tray, signage
│   │   └── config/scene.rviz
│   │
│   ├── hrc_operator/                      ← walking + reaching avatar (Phase 2)
│   │   ├── hrc_operator/
│   │   │   ├── walker.py                  ← walks into the workspace
│   │   │   └── reaching_arm.py            ← arm sub-animation when near
│   │   └── config/operator_paths.yaml
│   │
│   ├── hrc_predictor/                     ← LSTM trajectory prediction (Phase 3)
│   │   ├── hrc_predictor/
│   │   │   ├── data_gen.py
│   │   │   ├── model.py                   ← nn.LSTM(2→16→2)
│   │   │   ├── train.py
│   │   │   ├── predictor_node.py
│   │   │   └── anticipatory_zone.py       ← uses predicted pose
│   │   ├── models/lstm.pt
│   │   ├── data/trajectories.npz
│   │   └── test/test_predictor.py
│   │
│   ├── hrc_vlm_perception/                ← open-vocab grounding (Phase 4)
│   │   ├── hrc_vlm_perception/
│   │   │   ├── moondream_runner.py
│   │   │   ├── vlm_perception_node.py     ← service /vlm/locate (str query)
│   │   │   ├── yolo_fallback.py
│   │   │   └── prompt_templates.py
│   │   └── models/moondream2-int8/        ← ~1.8 GB, downloaded once
│   │
│   ├── hrc_voice/                         ← speech + intent (Phase 4)
│   │   ├── hrc_voice/
│   │   │   ├── whisper_node.py            ← whisper.cpp tiny.en
│   │   │   ├── intent_parser.py           ← Phi-3-mini via Ollama
│   │   │   └── action_schema.py           ← pydantic JSON schema
│   │   └── prompts/intent_system.txt
│   │
│   ├── hrc_task_planner/                  ← state machine + glue (Phase 5)
│   │   ├── hrc_task_planner/
│   │   │   ├── task_planner.py            ← state machine
│   │   │   ├── primitives.py              ← pick/place/handover/drop
│   │   │   └── scenarios.py
│   │   └── launch/demo.launch.py
│   │
│   ├── hrc_benchmark/                     ← research-grade evaluation (Phase 5)
│   │   ├── hrc_benchmark/
│   │   │   ├── scenario_runner.py
│   │   │   ├── baseline_halt_only.py      ← naive halt baseline
│   │   │   ├── baseline_reactive.py       ← v3-style reactive baseline
│   │   │   ├── metrics_collector.py
│   │   │   └── analyze.py
│   │   ├── scenarios/                     ← YAML scenario specs
│   │   └── results/                       ← auto-generated tables + plots
│   │
│   └── hrc_bringup/                       ← top-level launchers (Phase 6)
│       ├── launch/
│       │   ├── full_demo.launch.py
│       │   ├── benchmark.launch.py
│       │   └── headless.launch.py
│       └── config/full_demo.rviz
│
├── scripts/
│   ├── run_demo.sh                        ← one-command launcher
│   ├── record_demo.sh                     ← ffmpeg + ROS bag
│   └── render_split_screen.sh
│
└── docs/
    ├── architecture.png                   ← rendered diagram
    ├── demo_video.mp4                     ← 90s split-screen
    └── failure_cases.md                   ← honest writeup
```

---

## 7. Phases — build order

### Phase 0 — Planning ✓ (this document)
- Decide hardware/sim platform → see §3
- Decide stack → see §4
- Map A*STAR pitch → see §13

### Phase 1 — Workcell + dual arms (~2 evenings)
- [x] `hrc_description` (v1): workcell URDF, dual UR5e on **wide pedestal**
      (1.80 m in Y, arms at ±0.60 → 1.20 m separation), 2F-85 grippers,
      RealSense D435, single cabinet on +Y side. **Locked 2026-06-06**.
      Replaced by `hrc_description_v2` below.
- [x] `hrc_description_v2` (2026-06-08): new **4-section task cabinet**
      at Y=+1.00 (pulled forward, no +Y walkway). 60 × 65 × 20 cm
      cabinet with a vertical divider at X=+0.25 and a horizontal
      divider at Z=+1.22, creating four sections:
        - **TL — Blue**: row of 5 cubes (single line, touching)
        - **TR — Yellow**: 3-2-1 pyramid (6 cubes)
        - **BL — Pink**: tower of 4 cubes (vertical stack, single X-Y)
        - **BR — Purple**: 2×2 base + 1 on top (5 cubes, mini pyramid)
      Each section's front edge has a colour-tag strip. Back wall
      flush with cabinet back at Y=+1.20. Pedestal, table, mast, arms
      identical to v1. The +Y side has no walkway by design; the
      human-robot collaboration plays out on the **−Y side** instead,
      near the pickup table (see Phase 2).
- [x] `hrc_moveit_config`: SRDF planning groups, OMPL, KDL kinematics,
      joint limits, cross-arm finger disable_collisions.
      **2026-06-08**: depends on `hrc_description_v2` for the new
      cabinet geometry. SRDF unchanged (arm groups stable across
      cabinet revisions).
- [x] Smoke launch: RViz shows both arms + v2 cabinet, move_group
      ready to plan for either arm.
- [x] Plan-only handoff: MoveIt-driven for BOTH arms via Cartesian
      targets + runtime `/compute_ik`, with seed-retry chain and
      fallback to kinematic-only IK. Gripper constants:
      `GRIP_OPEN=0.0`, `GRIP_CLOSED=0.017` (5 cm cube).
- [🟡] **Four-section colour-coded stacking demo with random cube
      spawn.** Replacing the earlier single-cube raster.
  - `hrc_physics` (existing): extended to **20 coloured cubes** (5
    blue + 6 yellow + 4 pink + 5 purple). Same plan-only motion +
    rigid-body twin pattern. `/scene/reset_cube` becomes
    `/scene/spawn_next` — accepts a colour, picks the next unused
    cube of that colour, teleports it to a **random pose on the
    pickup table** (X ∈ [−0.20, +0.20], Y ∈ [−1.40, −0.95], Z=+0.81)
    with anti-repeat (≥ 6 cm from last 5 spawns).
  - `hrc_handoff_demo` rewrite: **round-robin task scheduler**
    cycling through Blue → Yellow → Pink → Purple → … . Each cycle:
    1. Decide next colour (skip sections already complete).
    2. Request a spawn of that colour at a random table pose.
    3. LEFT picks → handover → RIGHT stacks at the next slot in
       that section's pattern.
    4. Loop until all 20 placed; then full reset.
  - **Per-section target tables** in the demo (computed at startup,
    no MoveIt planning constraint changes vs single-section):
    - TL Blue line: cube centres at Y=+1.10, Z=+1.255, X ∈ {0.00,
      0.05, 0.10, 0.15, 0.20}
    - TR Yellow pyramid 3-2-1: base X ∈ {0.35, 0.40, 0.45} at
      Z=+1.255; middle X ∈ {0.375, 0.425} at Z=+1.305; top X=0.40
      at Z=+1.355. All Y=+1.10.
    - BL Pink tower: single X=+0.10, Y=+1.10; Z ∈ {0.945, 0.995,
      1.045, 1.095}
    - BR Purple 2×2+1: base 4 cubes at (X,Y) ∈ {(0.375,1.075),
      (0.425,1.075), (0.375,1.125), (0.425,1.125)} at Z=+0.945;
      top at X=0.40, Y=+1.10, Z=+0.995.
  - **Approach for each section**:
    - TL, TR top sections: horizontal approach (gripper z=+Y) where
      reach allows; z-down where it doesn't.
    - BL tower: z-down on each successive cube (top cubes pile on
      lower ones via physics, no separate nudge needed).
    - BR 2×2+1: z-down for base cubes; top cube placed centrally
      using contact-aware lower (release when bottom face touches
      both base cubes).
  - **Acceptance metric**: full 20-cube round completes without
    abort. Sections fill in round-robin order so the user sees all
    4 colours build up gradually. Cube spawn positions visibly
    differ across the round (no fixed pattern).

### Phase 2 — Safety + scene + operator (~3 evenings)
- [ ] `hrc_safety`: zone_classifier (CLEAR/Y/O/R per arm), safety_monitor
      (pre-flight + watchdog + e-stop), stack_light, force_monitor (PFL hook)
- [ ] `hrc_scene`: painted floor zones, light curtain, scanner cones,
      workstation props, signage. All MarkerArray. ISO-aligned visual story
- [ ] `hrc_operator`: walker traversing the **−Y front of the cell**
      along ±X. Reaching arm sub-animation triggers when the walker
      reaches X≈0 (centered on the pickup table / handover region).
      No walker path on the +Y side (cabinet flush against the wall).
- [ ] Verify visually in RViz: safety story now reads as collaborative cell

### Phase 3 — Anticipatory safety (the research contribution) (~2 evenings)
- [ ] `hrc_predictor`: synthetic data gen from walker, train LSTM, export
      TorchScript, predictor_node publishes `/safety/human_pose_predicted`
      at 1 s and 2 s horizons
- [ ] `anticipatory_zone`: same interface as zone_classifier but uses
      predicted pose. Publishes `/safety/zone_predicted`
- [ ] **Metric:** prediction RMSE on held-out trajectories, dodge lead-time

### Phase 4 — Modern AI layer (~3 evenings)
- [ ] `hrc_vlm_perception`: Moondream2-int8 loaded once, service
      `/vlm/locate` (string query → bbox + 3D pose via depth lookup)
- [ ] `hrc_voice`: whisper.cpp tiny.en + Phi-3-mini intent parser,
      publishes `/task/command` as structured JSON
- [ ] Together: voice command → VLM grounding → primitive selection
- [ ] **Metric:** VLM grounding accuracy + intent classification accuracy

### Phase 5 — Task planner + benchmark (~2 evenings)
- [ ] `hrc_task_planner`: state machine (IDLE → DETECTING → PICKING_A
      → HANDOVER → PLACING → COMPLETE), uses anticipatory zone for outward
      motions, language-conditioned via `/task/command`
- [ ] `hrc_benchmark`: scenario runner, three baselines (naive halt,
      reactive, anticipatory), metrics collector, analysis script
- [ ] **Metrics:** task completion time, intervention count, dodge lead time,
      per the falsifiable claims in §2

### Phase 6 — Demo + writeup (~2 evenings)
- [ ] `hrc_bringup`: full_demo.launch, headless.launch, RViz config
- [ ] `run_demo.sh` one-command launcher
- [ ] 90s split-screen demo video with voice command captions
- [ ] Architecture diagram render
- [ ] `README.md` research-paper style: motivation, method, metrics,
      ablations, failure modes
- [ ] `failure_cases.md`: honest writeup of where it breaks

**Total effort estimate: ~14 evenings (~50–70 focused hours).**

---

## 8. Metrics & acceptance criteria

A phase is "done" only when its metrics file exists and meets target.

| Phase | Metric | Target |
|---|---|---|
| 1 | RViz shows full scene, plan-only handoff succeeds | manual ✓ |
| 1 | MuJoCo cube physics step time @ 500 Hz | ≤ 1.5 ms (real-time on Mac VM) |
| 1 | Full row stacked tight (`|nnn...nnn|`) via contact-aware nudge, 10 cubes | manual ✓ — no gaps wider than CUBE_SIZE/10 |
| 1 | Nudge-slide terminations from `/scene/contacts`, not timeouts | 100% |
| 2 | All scene displays publish, walker enters cell, zone transitions on /safety/zone | manual ✓ |
| 3 | LSTM RMSE @ 1 s | ≤ 15 cm |
| 3 | LSTM RMSE @ 2 s | ≤ 30 cm |
| 3 | Mean dodge lead time vs reactive | ≥ 400 ms |
| 4 | Moondream2 grounding accuracy (20-object set) | ≥ 80% |
| 4 | Moondream2 latency / query | ≤ 5 s |
| 4 | Whisper + Phi-3 intent accuracy (20-utterance set) | ≥ 90% |
| 5 | Task completion time anticipatory vs reactive | ≥ 15% faster |
| 5 | Full stops/minute anticipatory vs reactive | ≥ 30% fewer |
| 6 | Demo video runs end-to-end without manual intervention | manual ✓ |
| 6 | CPU usage during demo | ≤ 4 cores avg |

---

## 9. Technical decisions

- **From scratch.** No code, URDF, mesh, or config copied from
  `twoarms_ws`. The projects share an architectural philosophy but
  nothing else. `twoarms_ws` is preserved as-is for the locked-package
  archive.
- **Sim-only by design.** No real-hardware story in this project.
  Mitigated by metric-driven evaluation that signals research rigor.
- **CPU-only by constraint.** VM has no GPU. All ML components were
  chosen specifically for CPU inference: Moondream2 (1.6 B), whisper
  tiny.en (39 M), Phi-3-mini (3.8 B at int4), custom LSTM (~10 K params).
- **No paid APIs by user preference.** Everything runs on-device. No
  network calls in the control loop.
- **Anticipatory safety as the thesis.** This is the falsifiable
  research claim. The benchmark methodology in Phase 5 exists
  specifically to defend it.
- **Plan-only MoveIt2.** No real controllers in sim; primitives publish
  `/joint_states` directly. Same trick as twoarms used. Safety monitor
  still gates the flow.
- **MuJoCo rigid-body twin on the manipulated object (cube), not the
  full robot.** Gazebo Fortress with `gz_ros2_control` is feasible per
  [[project_adac_headless_gz_sim]] but too heavy for a Mac-host VM and
  doesn't pay back here — the failure modes worth showing are at the
  cube-gripper / cube-cube / cube-shelf contact interface, not in the
  robot dynamics. MuJoCo runs headless, Apple-Silicon native, with one
  Python `pip` dependency. The cube has real gravity, real friction,
  real contact with neighbours; the demo node closes the nudge loop on
  contact reports rather than open-loop teleport. This is also the
  pattern used in modern grasp-synthesis research (Dex-Net, CGN, etc.)
  — credible architecturally.

---

## 10. Risks & mitigations

| Risk | Mitigation |
|---|---|
| Moondream2 latency too high for closed-loop | Use it only at episode start / explicit re-grounding; not in control loop |
| whisper.cpp tiny accuracy on free speech | Constrain Phi-3 to a fixed intent JSON schema; misrecognition → re-prompt instead of misexecute |
| LSTM blurs under random walker behavior | Condition on current walker target (no leak — walker target is observable in sim). If still poor, switch to a Mixture-Density LSTM |
| Phi-3 hallucinates non-existent actions | Constrain decode with grammar/JSON-schema (e.g., outlines / jsonformer) |
| UR5e MoveIt config breakage on Humble (similar to ADAC `wrist_3` issue) | Vendor a patched joint_limits.yaml from day one ([[feedback_adac_ur3e_avoid]] applies here too) |
| Gazebo / gz-sim headless without GPU | NOT USING — see §9. MuJoCo sidecar instead. Reference [[project_adac_headless_gz_sim]] preserved for the ADAC project, not applicable here. |
| MuJoCo URDF parsing rejects xacro-generated URDF (mesh paths, gz-tags, etc.) | Pre-flatten via `xacro` → vanilla URDF, then convert with `mujoco`'s URDF importer or hand-write MJCF for the manipulated objects only (cube + cabinet collision proxies) and load arms by reference if needed. Spike this early in Phase 1. |
| MuJoCo + ROS2 timing — physics drift vs `/joint_states` rate | Run MuJoCo at 500 Hz (2 ms step), publish cube pose at 30 Hz. Drive MuJoCo robot joints by *position-tracking* against the latest `/joint_states` (PD on each joint) rather than torque control — the demo's motion is kinematic, the physics-twin just follows. |
| Wrist collides with cabinet middle divider on upper-row stacks | MoveIt path-constraint on `right_wrist_2_link.z < 1.18` for placement trajectories. |
| Scope creep | Each phase has a hard metric in §8; do not advance until metric is met or explicitly waived |

---

## 11. Out of scope (explicitly)

- Real hardware deployment
- Sim-to-real transfer
- Multi-robot fleet beyond two arms
- Cloud / API-based AI
- ROS1 compatibility
- Touching anything in `twoarms_ws`

---

## 12. How this maps to each A*STAR role

| Role | What this project demonstrates |
|---|---|
| **ARTC SafeHRC (Req 680)** | ISO/TS 15066 SSM zones (visual + classifier), PFL hook (force_monitor), anticipatory safety with learned trajectory prediction, validated against reactive and naive baselines |
| **SIMTech ARM (Req 860)** | Bimanual coordinated manipulation, language-conditioned task specification, open-vocab perception — frontier of robotic manipulation in 2026 |
| **ARTC ASR (Req 627)** | End-to-end ROS2 system with clean package boundaries, modular AI components, research-grade evaluation methodology |
| **ARTC Process Robotisation** | Operator workstation + shared workpiece + scanner infrastructure proves cell-level thinking; voice/VLM/predictor are drop-in for new processes |

---

## 13. The 60-second interview pitch

> *"I built a dual-arm collaborative workcell in ROS2 with a MuJoCo
> physics twin on the manipulated object — MoveIt2 plans the arms
> kinematically and MuJoCo handles real contact-rich behaviour at the
> cubes. The cell has ISO-aligned visual safety infrastructure — SSM
> floor zones, light curtains, scanner coverage. Two UR5e arms perform
> a colour-coded sort task: the left arm picks cubes (random spawn
> positions on the table, four colours) and hands them across to the
> right arm, which stacks them into a four-section cabinet — each
> section a different pattern (row, pyramid, tower, 2×2 + top). A
> human operator works on the −Y side of the cell (in front of the
> pickup table) — walking past, reaching toward the table, or
> interrupting the handover. The system uses a learned LSTM to predict
> the operator's trajectory 1–2 seconds ahead and **anticipates** the
> slow-down instead of reacting after intrusion. I benchmarked this against a reactive
> baseline and a naive halt-only baseline across N scenarios and
> measured X% reduction in task downtime with Y ms mean lead-time on
> dodge decisions. Task commands are language-conditioned: voice goes
> through Whisper-tiny + Phi-3-mini for intent parsing, and a 1.6 B
> vision-language model (Moondream2) handles open-vocabulary object
> grounding — so 'pick up the red block' works without ArUco markers.
> Everything runs **CPU-only on a single VM** — no cloud dependency for
> the safety-critical loop."*

Every clause maps to something the interviewer cares about; every claim
is backed by a metric in §8.

---

## 14. Decisions to confirm before Phase 1

- [ ] Hardware: **dual UR5e + Robotiq 2F-85 + RealSense D435** (default).
      Alternatives considered: Franka Panda (for native impedance, less
      industrial); UR10e (heavier payload, overkill here)
- [x] Simulator: **MuJoCo (Python, headless) as sidecar on the cube
      only** — decided 2026-06-08. Gazebo Fortress was the original
      candidate but is too heavy on a Mac-host VM and pays for things
      we don't need (robot dynamics). See §9 for the rationale.
- [ ] Ollama model for intent: **Phi-3-mini-128k-instruct (Q4)** vs
      Llama-3.2-3B. Phi-3 wins on instruction-following at this size
- [ ] Walker pattern: **deterministic-ish with random stuck-recovery**
      (predictor-friendly) vs **fully random** (more challenging benchmark)

---

## 15. Locked-package convention

Following the pattern established in `twoarms_ws`: once a package is
deemed feature-complete and demo-stable, **lock it** with a
`CHECKPOINT.md` containing:
- What's locked (files, behavior, constants)
- Hard rules (don't edit, don't change these values)
- How to run
- Observable behaviors
- Fallback: future changes go in a new package (e.g., `hrc_safety_v2`),
  ask the user before proposing

This prevents demo regressions and keeps each phase reviewable.

---

*End of PLAN.md. Next step: confirm decisions in §14, then start Phase 1.*
