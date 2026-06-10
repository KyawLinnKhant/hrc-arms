# HRC-ARMS — Adaptive Dual-Arm Collaborative Workcell with Modern AI

> **Status: ongoing — Phase 1 (workcell + dual-arm handoff + 4-section stacking) in active iteration. See [PLAN.md](PLAN.md) for the full design and the phase roadmap.**

A dual-arm human-robot-collaboration (HRC) workcell in ROS 2 + MoveIt 2 + MuJoCo where two UR5e cobots perform a coordinated bimanual sort task while a human operator works alongside them. Built CPU-only, on-device, with no paid APIs or cloud calls — targeted at the A\*STAR SIMTech / ARTC robotics roles (ARM, ASR, SafeHRC, Process Robotisation).

---

## The pitch in one paragraph

The **left arm picks** coloured cubes (blue / yellow / pink / purple, random spawn pose on a side table) and **hands them across** to the **right arm** in mid-air with a perpendicular grasp. The right arm then **stacks** them into a 4-section task cabinet — each section a different pattern: blue → row of 5, yellow → 3-2-1 pyramid, pink → 4-cube tower, purple → 2×2 base + top. The cell has ISO-aligned safety infrastructure (SSM floor zones, light curtain, scanner cones, signage) and uses **anticipatory** safety — a learned LSTM predicts the operator's trajectory 1–2 s ahead so the arms pre-emptively slow down *before* the operator enters the protective stop zone. Task commands are language-conditioned (Whisper-tiny → Phi-3-mini intent parser), object grounding is open-vocabulary (Moondream2 1.6 B on CPU), and a research-grade benchmark compares anticipatory vs reactive vs naive-halt baselines on fixed scenarios.

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                       RViz visualization                             │
│   [UR5e L] [UR5e R] [D435 marker] [Operator avatar] [Workpiece]      │
│                                                                      │
│   Joint states ←── plan-only state machine (no controllers)          │
│   Cube pose    ←── MuJoCo sidecar (real contact physics)             │
└──────────────────────────────────────────────────────────────────────┘
                                  │
            /joint_states · /camera/{rgb,depth} · TF
                                  ▼
                          ┌───────────────┐
                          │  ROS 2 Graph  │
                          └───────────────┘
                                  │
   ┌──────────────┬───────────────┼───────────────┬──────────────┐
   │              │               │               │              │
┌──▼──────┐ ┌─────▼─────┐ ┌───────▼────────┐ ┌────▼──────┐ ┌────▼─────┐
│  Voice  │ │   VLM     │ │  Anticipatory  │ │   Task    │ │   HRC    │
│ Whisper │ │ Perception│ │     Safety     │ │  Planner  │ │ Coordin- │
│ + Phi-3 │ │ Moondream2│ │   LSTM + zone  │ │  (state   │ │  ator    │
│ intent  │ │ + YOLO    │ │   classifier   │ │  machine) │ │  speed + │
│ parser  │ │           │ │                │ │           │ │ checkpt  │
└──┬──────┘ └─────┬─────┘ └────────┬───────┘ └────┬──────┘ └────┬─────┘
   │              │                │                │              │
   ▼              ▼                ▼                ▼              ▼
                         ┌─────────────────┐
                         │  MoveIt2 ×2     │
                         │  (left, right)  │
                         └─────────────────┘
```

**Architectural choice — plan-only motion + rigid-body twin on the workpiece, not full-scene physics.** Arms are driven via MoveIt 2 plan-only (joint-state interpolation), not torque-controlled in physics. The cubes *are* physics-simulated in MuJoCo (gravity, friction, gripper–cube contact, cube–cube contact, cube–shelf contact). This keeps the demo lightweight on a CPU-only VM while still proving real contact-rich manipulation on the hard part of the problem.

---

## Repository layout

```
hrc_arms/
├── PLAN.md                                   ← full design document
├── README.md                                 ← this file
└── src/
    ├── hrc_description/        (locked) v1 workcell URDF + RViz
    ├── hrc_description_v2/     (locked) 4-section cabinet, dual UR5e, pedestal, mast
    ├── hrc_moveit_config/      MoveIt 2 SRDF + OMPL + KDL kinematics for both arms
    ├── hrc_physics/            MuJoCo sidecar — workpiece physics, contact reporter
    └── hrc_handoff_demo/       Round-robin scheduler + state machine for the
                                pick-handover-stack cycle (left picks, right stacks)
```

Each locked package has a `CHECKPOINT.md` documenting what the user signed off on, the geometry, and the hard rules.

---

## Hardware platform (sim-only)

| Component | Choice | Why |
|---|---|---|
| Arms | **Dual UR5e** | ISO 10218-1 certified cobots, larger reach than UR3e |
| Grippers | **Robotiq 2F-85** | Industry-standard parallel gripper, force-feedback hook |
| Wrist sensor | **Robotiq FT-300S** (sim) | PFL story — contact-force monitoring per arm |
| Camera | **Intel RealSense D435** (sim) | RGB + depth in one URDF block |
| Workcell | Side table (left-arm pick, −Y) + 4-section task cabinet (right-arm place, +Y) + back wall | Designed for the bimanual sort task |

---

## Software stack

| Layer | Tool | CPU OK? |
|---|---|---|
| OS / middleware | Ubuntu 22.04, ROS 2 Humble | ✓ |
| Physics (workpiece only) | **MuJoCo** (headless, Python API, 500 Hz) | ✓ |
| Motion planning | MoveIt 2 (OMPL / RRTConnect) — plan-only | ✓ |
| ML framework | PyTorch (CPU build) | ✓ |
| Vision-language | Moondream2 1.6 B (int8) | ✓ ~2–5 s/query |
| Speech-to-text | whisper.cpp (tiny.en) | ✓ real-time |
| LLM intent | Phi-3-mini-128k-instruct via Ollama | ✓ ~10 tok/s |
| Trajectory prediction | Custom LSTM ~10 K params | ✓ real-time |
| Object detection (fallback) | YOLOv8n | ✓ 5–15 FPS |

**Zero paid APIs. Zero GPU dependencies. Zero cloud calls at runtime.**

---

## Status — phases

| Phase | Scope | Status |
|---|---|---|
| **0** | Plan (`PLAN.md`) — hardware, stack, falsifiable claims | ✅ done |
| **1** | Workcell URDF, MoveIt config, MuJoCo sidecar, **round-robin 4-section stacking demo** | 🟡 in progress — single-cube handoff loops; 4-section round-robin scheduler implemented, placement strategy under iteration |
| **2** | Safety (`hrc_safety`) — SSM zones, e-stop, PFL hook; scene props; walking operator | ⏳ not started |
| **3** | Anticipatory safety — LSTM predictor + dodge lead-time metric | ⏳ not started |
| **4** | Modern AI — Moondream2 grounding service, Whisper + Phi-3 intent parser | ⏳ not started |
| **5** | Task planner + research benchmark (anticipatory vs reactive vs halt-only) | ⏳ not started |
| **6** | Top-level launchers, demo video, write-up | ⏳ not started |

Phase 1 acceptance: full 20-cube round (5 blue + 6 yellow + 4 pink + 5 purple) completes without abort.

### Remaining steps (live TODO)

**Phase 1 — finish the stacking demo**
- [ ] Make all four colour cubes nudge laterally toward the previous neighbour (currently only blue does this cleanly; yellow / pink / purple still lag in their animation).
- [ ] Eliminate the residual "cube lags behind the moving gripper" artefact on the right arm during the stack-approach motion (bump marker republish rate, confirm zero-stamp TF lookup).
- [ ] Plan reliably for every cube in every section (5 blue + 6 yellow + 4 pink + 5 purple = 20 placements per round) without OMPL aborts. Today some seeds intermittently miss IK on the second blue / pink cubes — expand the IK seed bank for the placement-from-outside pose.
- [ ] Lock `hrc_handoff_demo` with a `CHECKPOINT.md` once the 20-cube round completes 5× in a row without abort.

**Phase 2 — safety + scene + operator**
- [ ] `hrc_safety` — zone_classifier (CLEAR / Yellow / Orange / Red per arm), safety_monitor (pre-flight + watchdog + e-stop), stack_light, force_monitor (PFL grip-force hook).
- [ ] `hrc_scene` — painted SSM floor zones, light curtain, scanner cones, workstation props, signage (all MarkerArray).
- [ ] `hrc_operator` — walker traversing the −Y front of the cell along ±X, with a reaching-arm sub-animation when near the pickup table.

**Phase 3 — anticipatory safety (the research contribution)**
- [ ] `hrc_predictor` — synthetic data gen from the walker, train a small LSTM (~10 K params), export TorchScript, publish `/safety/human_pose_predicted` at 1 s and 2 s horizons.
- [ ] `anticipatory_zone` — same interface as `zone_classifier` but uses the predicted pose; publishes `/safety/zone_predicted`.
- [ ] Metrics: prediction RMSE on held-out trajectories, dodge lead-time vs reactive.

**Phase 4 — modern AI layer**
- [ ] `hrc_vlm_perception` — Moondream2-int8 loaded once, service `/vlm/locate` (string query → bbox + 3D pose via depth lookup).
- [ ] `hrc_voice` — whisper.cpp tiny.en + Phi-3-mini intent parser, publishes `/task/command` as structured JSON.
- [ ] Metrics: VLM grounding accuracy + intent classification accuracy.

**Phase 5 — task planner + benchmark**
- [ ] `hrc_task_planner` — full state machine (IDLE → DETECTING → PICKING_A → HANDOVER → PLACING → COMPLETE), language-conditioned via `/task/command`.
- [ ] `hrc_benchmark` — scenario runner + three baselines (naive halt, reactive, anticipatory), metrics collector, analysis script.
- [ ] Populate the X / Y / Z / N / M / P / Q targets in the falsifiable claims with measured numbers.

**Phase 6 — demo + write-up**
- [ ] `hrc_bringup` — full_demo.launch, headless.launch, RViz config.
- [ ] `run_demo.sh` one-command launcher.
- [ ] 90 s split-screen demo video with voice command captions.
- [ ] Architecture diagram render + research-paper-style write-up of motivation, method, metrics, ablations, failure modes.

**Total remaining effort: ~12 evenings (~45–60 focused hours) on top of what's already in main.**

---

## Quick start

Requires Ubuntu 22.04 + ROS 2 Humble + MuJoCo Python bindings.

```bash
# clone
git clone https://github.com/KyawLinnKhant/hrc-arms.git
cd hrc-arms

# build
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# launch the dual-arm handoff + stacking demo
source install/setup.bash
ros2 launch hrc_handoff_demo handoff.launch.py
```

You should see RViz come up with the workcell, both UR5e arms, the side table, the 4-section cabinet, and the demo running its round-robin pick → handover → stack cycle.

The cubes are 5 cm, coloured by section:
- **Blue** (TL section) — row of 5
- **Yellow** (TR section) — 3-2-1 pyramid
- **Pink** (BL section) — tower of 4
- **Purple** (BR section) — 2×2 base + 1 on top

---

## Falsifiable claims to defend at interview

The project is structured around five testable claims (full targets in [PLAN.md §2 + §8](PLAN.md)):

1. Anticipatory dodging reduces task downtime by ≥ X % vs reactive halt.
2. The LSTM predictor achieves ≤ Y cm RMSE at 1 s horizon, ≤ Z cm at 2 s.
3. Moondream2 on CPU grounds the target object in ≤ N s at ≥ M % accuracy on a 20-object set.
4. Whisper-tiny + Phi-3-mini achieves ≥ P % intent-classification accuracy on a 20-utterance set.
5. Total CPU usage stays below Q cores during demo execution.

X / Y / Z / N / M / P / Q targets are populated and validated in Phase 5.

---

## How this maps to each A\*STAR role

| Role | What this project demonstrates |
|---|---|
| **ARTC SafeHRC** (Req 680) | ISO/TS 15066 SSM zones (visual + classifier), PFL hook, anticipatory safety with learned trajectory prediction, validated against reactive and naive baselines |
| **SIMTech ARM** (Req 860) | Bimanual coordinated manipulation, language-conditioned task specification, open-vocab perception — frontier of robotic manipulation in 2026 |
| **ARTC ASR** (Req 627) | End-to-end ROS 2 system with clean package boundaries, modular AI components, research-grade evaluation methodology |
| **ARTC Process Robotisation** | Operator workstation + shared workpiece + scanner infrastructure proves cell-level thinking; voice / VLM / predictor are drop-in for new processes |

---

## Locked-package convention

Once a package is feature-complete and demo-stable, it is **locked** via a `CHECKPOINT.md` containing what's locked, hard rules, observable behaviours, and a "propose a v2 package before editing" escape hatch. This prevents demo regressions and keeps each phase reviewable. Currently locked: `hrc_description`, `hrc_description_v2`.

---

## Author

**Kyaw Linn Khant** — building this as a portfolio demonstration for the A\*STAR robotics research roles. Feedback and pull requests welcome.

---

*Ongoing project. The code in `main` is the latest known-good state of Phase 1; expect iteration on the right-arm placement strategy as the round-robin scheduler is tuned for all four colour sections.*
