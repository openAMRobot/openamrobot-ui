# STATE

Read this file before any work in this repository and update it in the same PR as the work.
The PR evidence check fails when STATE.md exists and a PR neither updates it nor says
"no change" with a reason. Keep it under 60 lines; facts only, each with a date or SHA.

## Repository

- Type and lifecycle: ui, active
- Readiness: not yet assessed (added 2026-10-30 with the upper-body view; owner to fill)
- Owner role: operator UI and backend integration (Mohamed, interim, per AGENTS.md)
- Verification: `cd web && CI=true npm test -- --watchAll=false`; on main 0ca798b4 it ran
  1 suite, 3 tests, all passing (local run, 2026-10-30)

## Current work

| Work package | Branch or PR | Status | Next step |
|---|---|---|---|
| Read-only upper-body view (prototype) | ui/upper-body-view | draft | Work-package owner writes adopt, adapt or reject |

## Upper-body view (prototype, 2026-10-30)

- Robot Description page subscribes to `/robot_description` and `/joint_states` only. When the
  published URDF has `lift_joint` and OpenArm joints it shows that model under the banner
  "Candidate model, provisional geometry" (openamr-upperbody-sw PR #14, checked against 4629075).
- Configuration identity (column, base_plate, head_pitch, base_tilt, datums revision) shows
  "model or configuration unknown": no runtime producer exists (PR #14 known gaps). A reported
  hp25 would be flagged against the P-03 rev20.1 head pitch hp30.
- Per-group freshness (lift, each arm, each gripper) uses the measured publish rate.
- Lift panel: shoulder height from the reported lift_joint and the published model, against
  L1000, L1175 and L1350. Stop sensors: not reported until the I8 producer exists
  (openamrobot-interfaces #30).
- Camera panel: head camera is "not reported"; no contracted head camera topic exists.
- No publishers, services, actions or parameters; Jest tests fail if the page uses any.

## Decisions this repository implements

- Lift stops L1000, L1175, L1350 as calibration IDs; head_pitch hp30 per P-03 rev20.1.

## Known gaps

- Meshes of packages other than openamrobot_description are not served by the backend, so the
  combined model shows the arms with placeholder boxes until that is scoped.

## Not verified

- Real robot, an accepted upper-body model and the Getac ZX80 panel, since 4629075 and this PR.
- rosbridge delivery of the transient-local `/robot_description` topic.
