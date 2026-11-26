# Operator UI: consolidation and integration sequence

**Status:** Proposal for review · **Prepared for:** Alex · **Code owner:** Mohamed (operator UI, #27)
**Scope:** the work *after* RC.1. Nothing here is rc.1 content — the released candidate stays `334e0ad`
and no one should read this as a change to it.

---

## 1. What the UI work actually is today

Everything below is open and **draft**. `main` currently carries the command path (#50, merged
2026-11-22) and CI (#48); the rest of the UI work sits in eighteen draft PRs on `openamrobot-ui`.

| Stack | PRs | Base |
|---|---|---|
| **Command path** | #50 *(merged)* | `main` |
| **No UI-launched navigation** | #40 → #52 | #40 → `main` |
| **Maps / data directory** | #47 → #53 | #47 → `main` |
| **Reporting mode** | #54 → #61 | #54 → `main` |
| **Twin operator view** | #62 | on #50; ships with platform-sw #90 or not at all |
| **Restricted bridge (contract)** | #64 + issue #63 | proposal, awaiting the software lead |
| **Upper body (read-only)** | #55 | `main` |
| **Independent** | #41 #42 #43 #44 #45 #46 #49 #51 #58 | `main` |
| **Integration candidate** | #56 *do not merge* | merges #40–#55 heads; a test artifact, not a merge plan |

Message/interface side, none merged: openamrobot-interfaces #38 (I8 `openamr_base_msgs`),
#31 (whole-robot status and stop scope, docs only), #42 (C9 radians), #36 (A3/I1 manipulation),
#28 (SiteConfigStatus). Docs #102 (rc.1 Digital Twin) and manifest #23 (rc.1 pins + supported
Digital Twin profile) are the reference points for the twin side.

**The honest summary:** the work is not fragmented because people wrote different things. It is
fragmented because eighteen drafts were written against `main` *before* the two contracts they depend
on — the restricted bridge and the canonical interfaces — were decided. Sequencing is what is missing,
not code.

---

## 2. Alex's constraints, restated as tests

| Requirement | How it is checked, not asserted |
|---|---|
| One integrated operator interface | One launch profile; one web app; no second UI process in the release manifest (#23) |
| Canonical message interfaces | UI imports only `openamr_base_msgs` / published interfaces; **no `.msg`/`.srv`/`.action` file under `openamrobot-ui`** — CI grep, hard fail |
| Restricted ROS bridge | rosbridge runs with an allowlist; `/cmd_vel` is *not* publishable; a websocket client cannot reach twist_mux's inputs (issue #63) |
| No second Nav2 stack | No UI code launches Nav2 or SLAM — #40/#52 delete the legacy launches; CI check that `ros2 launch` targets in UI contain no nav2/slam packages |
| No duplicate message packages | Same as row 2, plus: UI constants that mirror message fields are generated or tested against the interface version |
| Approved command paths only | `/ui/cmd_vel` is the UI's only motion output; exactly one publisher per topic in the UI (the pre-#50 state had four: joystick, status bar, map page, Blockly actions) |
| Stop behaviour | Stop (software) binds `/safety/stop_lock` (#50, merged); the twin's stop path (platform-sw #90) must be merged before twin evidence counts |

---

## 3. Dependency graph

```
main
 ├─ #50 merged ────── /ui/cmd_vel + /safety/stop_lock
 │     ├─ #62 twin operator view ── requires platform-sw #90 (stop chain) to merge first
 │     └─ #64 allowlist ══ issue #63 ══ needs software-lead accept/reject
 ├─ #40 → #52      no UI-launched navigation
 ├─ #47 → #53      maps on a mountable volume
 ├─ #54 → #61      reporting mode
 ├─ #55            upper-body read-only view
 └─ #41 #42 #43 #44 #45 #46 #49 #51 #58   independent

interfaces: #38 (I8 msgs) ─┬─ #31 (status/stop scope)
                           ├─ #42 (C9 radians)
                           └─ #36 (manipulation)   #28 (site config)
```

Anything that consumes I8 or the allowlist cannot be *adopted* before those are accepted — it can be
reviewed, adapted, and kept draft.

---

## 4. The six areas, in the order they must settle

### 4.1 Canonical message interfaces
**Decision:** accept I8 (#38) and the status/stop scope (#31); accept C9 units (#42).
**Then:** publish a version. The UI codes against a version, not against a branch.
**Gate:** `openamrobot-ui` contains zero message definitions and its CI proves it.

**Already broken by a merged change:** the gripper change announced in `software-ai` (upperbody-sw #66)
flipped the meaning of open/closed — *"+0.7854 rad on the left and -0.7854 rad on the right"*, joint
positions and physical motion unchanged, only the label semantics — and named the UI as an owner that
must fix its use of gripper names or values. Any UI control, status field or Blockly action that says
"open"/"closed" is now wrong until updated.

### 4.2 Restricted ROS bridge
**Decision:** the software lead accepts or rejects #63. Until then no consumer may rely on the allowlist.
**Then:** land #64 against `main` (it was drafted into #50, which has merged, so it needs a rebase), and
re-run the live-revert evidence already on branch `evidence/allowlist-live-revert-2026-11-22`.
**Gate:** with the allowlist on, a websocket client publishing `/cmd_vel` is refused, and the robot still
moves through `/ui/cmd_vel` → twist_mux → stop gate → velocity_smoother → collision_monitor.

### 4.3 Robot status
**Decision:** status panel scope — base only, or base + arm + lift (#31 covers the whole robot).
**Then:** the panel reads approved topics only; it publishes nothing.
**Gate:** panel shows disconnected / stale / live distinctly; no status value is invented when a topic is
missing.

**SIL caveat that affects what the panel may claim:** the DART finding in `software-ai` — odometry
reporting 0.98 m while the true position moved 0.006 m, lift joint staying at 0.0 — means odom-only
simulation results are unverified until ros2_control/gz_ros2_control is fixed. A status panel that shows
odometry as fact in the twin is showing a number that does not describe the robot.

### 4.4 Navigation commands
**Already true on `main`:** the UI no longer publishes `/cmd_vel`; #40/#52 remove the legacy launches.
**Then:** nothing in the UI starts a navigation stack; route outcomes come from real feedback (#46).
**Gate:** CI check (no nav2/slam launch targets in UI) + one manual run where the platform stack is
started independently and the UI only commands it.

### 4.5 Stop behaviour
**Already on `main`:** Stop (software) → `/safety/stop_lock` (#50).
**Blocks:** the twin's stop path lives in platform-sw #90 and is unmerged, so #62's twin results are
evidence on an unmerged path — not rc.1 content, and not adoptable until #90 lands.
**Gate:** stop is reachable from every screen (status bar, map, Blockly), one publisher, and the twin
reproduces it after #90 is merged.

### 4.6 Digital Twin profile
**Exists as:** `twin.launch.py` with `stack:=platform|build1|none` (#62), docs #102, manifest pin (#23).
**Decision:** confirm the twin profile is the demo vehicle for the operator UI.
**Gate:** from a clean checkout, `twin.launch.py` reaches a working room in the documented number of
steps, and the manifest pins exactly that profile.

---

## 5. Sequence

**S0 — decisions (before RC.1 is published).** No code merges that touch the command path or the
manifest. Deliver: #63 accept/reject, I8/C9/status scope accepted, twin profile confirmed, Mohamed's
adopt/adapt/reject on the draft batch. This is the only stage that needs other people; everything after
it is work.

**S1 — foundation.** Rebase and land #64 on `main`; land #40→#52, #47→#53, #41, #42, #43, #44, #45, #46,
#49, #51, #55, #58 in dependency order. Each is small and already reviewed as a unit; #56 stops being
needed once they are on `main`, and should be closed rather than merged.

**S2 — twin.** Merge platform-sw #90, then rebase #62 and re-run its evidence on merged heads.

**S3 — operator view.** #62 without the twin-only caveat: status panel, nav commands, stop, all through
approved paths.

**S4 — reporting and upper body.** #61 on #54; #55 stays read-only.

**S5 — consolidation.** One launch profile, one web app, manifest updated, `docs/integration-sequence.md`
(this file) updated to say what actually shipped.

---

## 6. Decisions needed

| Decision | Owner | Blocks |
|---|---|---|
| Accept/reject #63 (allowlist) | software lead | #64, and every claim about a restricted bridge |
| Accept I8 (#38), C9 (#42), status/stop scope (#31) | interfaces + software lead | UI status panel, any UI consuming base messages |
| Adopt/adapt/reject on the draft batch | Mohamed (code owner, #27) | S1 |
| Confirm UI is RC.2 and the twin profile is the demo vehicle | Alex | S2–S4 framing |

## 7. Risks

- **The four-publisher problem returns.** It was fixed in #50; any new screen that publishes motion
  re-opens finding 6 on #90. One publisher per topic, tested, not reviewed by eye.
- **`#56` mistaken for a merge plan.** It is a test artifact that proves the drafts combine; merging it
  would land eighteen changes with one approval.
- **Twin evidence on an unmerged stop path.** #62's results are only evidence once #90 is merged.
- **A stale browser tab.** Allowlisting helps only if `/cmd_vel` is not publishable at all; a tab that
  survived a reload must not be able to drive.
- **Interface drift.** Eighteen drafts written before I8 landed will contain hand-rolled status fields —
  and the gripper semantics change (4.1) has already invalidated any open/closed mapping in the UI.
- **A status panel that believes odometry in SIL.** See 4.3: the twin's odom is not yet truthful.

## 8. What I will do next

1. Open this as a PR for review (docs only, no code).
2. Ask Mohamed for adopt/adapt/reject per PR in the batch, with this sequence as the order.
3. Prepare the S1 rebase list, with the exact rebase order and conflict notes, so S1 is mechanical.
4. Keep the twin work parked until #90 merges — no evidence is published from an unmerged path.
