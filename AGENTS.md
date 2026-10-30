<!-- BEGIN OPENAMROBOT SHARED RULES v2 -->
# OpenAMRobot rules for contributors and agents
Canonical: openAMRobot/.github, agent-rules/SHARED_RULES.md, copied verbatim into every
repository's AGENTS.md (tools/check_agent_rules.py reports drift). This file holds process;
approved technical values live in openAMRobot/.github decisions.yaml. Labels: [check: tool]
means the tool detects that violation, and only in repositories where its workflow is installed
and required (rollout/README.md); [template: file]; [human: role, evidence] is a reviewer
decision. A text check proves textual consistency, never mechanical, electrical or safety correctness.

## Decisions
- decisions.yaml is the only register of approved values, limits, exclusions and distinctions.
  No file states a contradicting value; kept history carries `decision-allow: <ID> <reason>`.
  [check: check_decisions.py, listed patterns only] [human: entry's reviewer, entry's evidence]
- Changing a decision: (1) open a contract change request issue naming the entry; (2) the
  owner updates the source document; (3) one reviewed PR updates the register entry, its
  supersedes history, its check patterns with a test, and every affected consumer, or links
  each consumer PR; (4) the owner approves. [template: contract change request] [human: owner]
- Source documents are provenance. CI reads only the pinned register, never a drive or the
  docs site; a disagreement is reported to the owner, and no tool rewrites either side. [human: owner]
- Read STATE.md before work in a repository; update it in the same PR. [check: check_pr_evidence.py]

## Contracts
- Messages, services, actions, schemas, topic names, launch argument names and configuration
  IDs change only through a contract change request and one PR that updates the contract
  package and its consumers together. [template: contract change request] [human: software lead]
- New contract proposals stay labelled Proposed until the owner accepts them. [human: software lead]

## Tests
- Every behaviour change carries a test that fails when the change is reverted; the Tests
  section shows that failing run. [template: PR Tests section] [human: reviewer, the revert run]
- A suite that executes zero tests fails. [check: verify.sh; check_pr_evidence.py on reported counts]
- skip, xfail and importorskip name a tracking issue on the same line. [check: verify.sh]
- SKIP or BLOCKED is not PASS. Fixtures and fake hardware are not simulation, physical
  acceptance or release readiness. [human: reviewer, Not verified section]

## Evidence
- Every PR states base SHA, head SHA, exact commands, test counts and a Not verified section.
  [check: check_pr_evidence.py, presence only] [human: reviewer, that the commands were run]
- A draft becomes ready only when the evidence check passes on the current head. [human: author;
  ruleset required check once installed]

## Dependencies and licences
- Nothing is added, removed or upgraded as a side effect. A changed dependency manifest needs
  a Dependencies section naming each change, its licence and source. [check: check_pr_evidence.py]
- Licence headers and package.xml tags match the licence map: MIT software and firmware,
  CERN-OHL-P-2.0 hardware, CC-BY-4.0 documentation. [human: repository owner]
- The Integration Gate section lists overlapping PRs, reused existing or upstream work and
  what was rejected. Third-party provenance stays intact. [template: PR template]
- LICENSE, LICENSING.md, NOTICE and CODEOWNERS change only in a PR whose task names them.
  [human: platform lead]

## Safety
- No agent authors or modifies E-stop, brake, contactor, watchdog, motor-enable or
  charge-inhibit logic; agents report the need in an issue. [check: check_pr_evidence.py fails
  a safety-path change whose AI disclosure is not None] [human: platform lead, undisclosed use]
- A safety-path change needs two human approvals including the platform lead. The check only
  reports "safety path touched, two human approvals required" and that reviewers are requested;
  approvals are a ruleset requirement. [human: platform lead and one more maintainer]
- Functional telemetry, watchdogs, status displays and fixtures are never safety evidence.
  [check: check_decisions.py wording only] [human: platform lead, hardwired safety-chain test record]
- Automated or untrusted PR jobs never reach motion hardware or secrets. [human: CI owner]

## Publication
- Public material (docs/, assets/, README.md, any path containing "public") has no internal
  document links, prices, contact data or credentials. [check: check_public_extract.py]
- It names no private person, customer or partner; the application name is Use_Case_1. [human: docs owner]

## Agents
- Before any write, state a precondition block: repository (exact full name), branch, parent
  SHA, expected outcome. Read-only unless the task says otherwise. [template: agent-prompts/]
- Agent PRs stay draft until the work-package owner writes adopt, adapt or reject in the thread.
  [human: work-package owner]
- The PR's AI disclosure section names the tool and what it produced. [check: section present]
- Commits carry DCO sign-off with the contributor's own identity; never invent an identity or
  attestation. [human: maintainer; DCO check where installed]
- Gate A Teensy/MPU6500 and Gate B STM32/ICM-42688-P stay distinct; Jetson is the 2.0 compute.
  [check: check_decisions.py] Arm vendor SDKs stay behind device packages. [human: software lead]

## Failure
- A failed precondition (repository, branch, SHA, access, source) stops the task. Report the
  command, the error and the next step; never work around it. [template: agent-prompts/]
- Never merge, force-push, change settings, weaken a check or invent a result. [human: ruleset]

## Learning
- Every agent or process mistake gets an issue labelled harness. [template: harness mistake form]
- The monthly retro turns harness issues into one PR against this block. [human: software lead]
<!-- END OPENAMROBOT SHARED RULES v2 -->

# Repository-specific rules: openamrobot-ui
## Layout and ownership
- Existing React frontend: web/. Flask/ROS packages: ros2/.
- Ownership (decision of 19 Oct): Parth Keshari is UI lead and Reporting Mode owner;
  Mohamed Sayed and ND are reviewers; Nikolai runs simulation checks.
- Reporting Mode is a full-screen route in this app, sharing backend state and adapters.
- Execution belongs to the robot-side executor; a browser view never establishes success.
- Operational state and terminal outcome are separate. Preserve UNKNOWN/stale semantics.
- No new backend, database, UI framework or vendor SDK path without approved scope.
- Read [UI WP](https://drive.google.com/file/d/1YSe7szOrYOsakToSa7ZCfAEj15Pb94rL/view).

## Commands and pitfalls
- Match web/package.json engines (currently >=18 <21); existing CI selects Node 20.
- Frontend install: cd web && npm ci
- Tests: cd web && CI=true npm test -- --watchAll=false
- Production build: cd web && npm run build
- Do not use --passWithNoTests as evidence that a feature suite ran.
- npm run lint currently rewrites files and lacks a working standalone ESLint config.
  Do not run it as a supposedly read-only check; inspect scripts before choosing lint.
- Production sync: bash scripts/build_frontend.sh then bash scripts/sync_frontend_to_ros.sh.
- ROS build: source /opt/ros/jazzy/setup.bash then bash scripts/build_ros.sh.
- In ros2/, with ROS and built overlay sourced: colcon test --packages-select
  openamr_ui_package openamr_ui_msgs ; then colcon test-result --verbose.
- Keep pure fixture tests runnable with python3 -m pytest at their documented path,
  without importing ROS. Do not claim ROS integration when only pure tests ran.
- Never commit node_modules, web/build, ros2/build, ros2/install or ros2/log.

## Development data isolation
- Existing Demo Mode is browser-side sample telemetry; preserve its Config toggle.
- Backend run fixtures are development/test-only, default off, never a Config option.
- With both enabled, fixture run-state/freshness comes only from Flask, not Demo Mode.
- Clearly label fixture data distinctly from the existing Demo banner; avoid a global
  claim that all app data is fake when only the fixture view is isolated.
- Share one consumer-neutral adapter for operator and future Reporting consumers.
- No fixture command path to hardware. Absence of forbidden imports alone is not proof.
- Authentication, production command controls and software E-STOP labelling require
  separate explicitly scoped work; do not slip these into fixture-only changes.

## Canonical context
- [Plans](https://drive.google.com/drive/folders/15zWoBPd6qSt96TToWakN9rhfjNyq-hoz)
- [D-02 AI framework](https://drive.google.com/file/d/1Drs4tKbxAo6jsRlaCRK1eAMkzds7NTx-/view)
- [D-03 consistency](https://drive.google.com/file/d/1jbELSAeWRxuxB-IO-s7QlSlWK38WemQA/view)
- [Interfaces](https://github.com/openAMRobot/openamrobot-interfaces)
- [Contribution rules](https://github.com/openAMRobot/.github/blob/main/CONTRIBUTING.md)
Read relevant sources; if inaccessible, use an approved supplied excerpt and disclose limits.
