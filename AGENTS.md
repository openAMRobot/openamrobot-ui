<!-- BEGIN OPENAMROBOT SHARED RULES v1 -->
# OpenAMRobot agent rules
Canonical shared block: openAMRobot/.github, agent-rules/SHARED_RULES.md.
This block is copied verbatim; GitHub does not propagate it between repositories.

## Before editing
- Read AGENTS.md, CONTRIBUTING.md, applicable nested instructions, code and tests.
- Record the base SHA; inspect relevant open PRs and accessible branches/forks for overlap.
- Do not infer contributor inactivity from absent public branches; disclose inaccessible work.
- Follow the approved task scope and applicable plan/contracts. Report contradictions.
- Reuse maintained upstream packages and existing implementation; minimize custom glue.
- Do not replace working legacy support merely because the new-robot BOM excludes it.

## Boundaries
- Shared ROS contracts belong in openamrobot-interfaces; identify their actual acceptance status.
- New contract proposals stay isolated and labelled Proposed, pending owner review.
- Do not author or modify safety implementation: E-stop, brakes, motion interlocks,
  watchdogs, actuator enable or power-protection logic. Report required changes.
- Status display and isolated test fixtures do not implement or validate physical safety.
- Arm vendor SDKs stay behind Device Packages; none in UI or mission consumers.
- Preserve Gate A Teensy/MPU6500 and Gate B STM32/ICM-42688-P distinctions.
- Jetson is the 2.0 reference compute; retain correctly labelled historical material.
- Public application name: Use_Case_1. No customer/partner names, secrets or private data.
- Preserve third-party provenance. Do not change licensing, NOTICE or CODEOWNERS
  without an explicit task that authorizes those files and the appropriate review.
- Never connect untrusted/automated PR tests to physical motion hardware or secrets.

## Delivery
- Use a contributor branch/fork and draft PR by default. Never merge, force-push,
  modify protection/settings or bypass checks in ordinary implementation tasks.
- Read applicable CLA/DCO rules. Never invent an exemption, identity or attestation.
- Use git commit -s only with the verified contributor identity and provenance authority.
- Disclose material AI assistance, dependencies and licence implications.
- Report base/head SHAs, scope, safety impact, exact commands/results and evidence links.
- For bug fixes show the regression fails before and passes after; for new features
  demonstrate a meaningful deliberate fault is detected. Explain non-applicability.
- Keep a Not verified section. SKIP/BLOCKED is not PASS; fixtures/fake hardware are
  not integrated simulation, physical acceptance or release readiness.
- Do not weaken checks, use empty suites as evidence or invent successful test results.
- If blocked, stop the blocked activity, report command/error/next step and continue
  independent in-scope work. Do not repeatedly reinstall or expand the architecture.
- Owner alignment and approval status must be truthful. A draft or notification is
  not evidence that a required discussion or technical acceptance has happened.
<!-- END OPENAMROBOT SHARED RULES v1 -->

# Repository-specific rules: openamrobot-ui
## Layout and ownership
- Existing React frontend: web/. Flask/ROS packages: ros2/.
- Moazzam owns operator UI/backend integration; Parth owns Reporting Mode presentation.
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
