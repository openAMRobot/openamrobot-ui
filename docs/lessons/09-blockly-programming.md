# Lesson 09 — Blockly Visual Programming

| Audience | Time | Prerequisites |
| --- | --- | --- |
| Operators and Blockly contributors | 20 minutes | [Lesson 00](00-your-first-10-minutes.md) and the Programs section of [Lesson 06](06-the-pages.md) |

## What you'll learn

You will learn how blocks become validated robot actions, which state is
stored in the browser or backend, and how to test plans without unsafe motion.

[Lesson 06](06-the-pages.md#programs--blockspagejsx) introduced this page
(the sidebar calls it "Programs") at a glance. This lesson is the deep dive
on it specifically:
what it is, how a block becomes a robot action, and how Voice Command fits
in. For the exhaustive block-by-block reference, every
example program, and setup/troubleshooting steps, see the practical guide:
[`web/src/features/blocks/README.md`](../../web/src/features/blocks/README.md).
This lesson is the "why it works this way"; that guide is the "how do I use
it" reference.

## What it is

Blockly is a visual programming editor: instead of writing code, you drag
blocks onto a workspace and connect them into a chain. The Blocks page
([`web/src/pages/BlocksPage.jsx`](../../web/src/pages/BlocksPage.jsx)) wraps
that editor with OpenAMR-specific blocks — navigate, wait, drive, dock, and so
on — so a non-programmer can build a robot program like "go here, wait, then
dock" without touching JavaScript or ROS. It's the same idea as the rest of
this UI ([Lesson 01](01-what-is-this-ui.md)): a friendlier way to do things a
person could otherwise only do by publishing ROS messages by hand.

## Screenshots and page layout

![OpenAMR UI Programs page: a start-robot-program block chain in the workspace, the toolbox on the left, and ROSBridge status, Voice Command, Program Templates, and Run History on the right](<../assets/programs/Blockly.png>)

The left toolbox groups blocks into `Program`, `Navigation`, `Motion`,
`Docking`, and `Robot State`. The center workspace is where blocks are
assembled. The right panel shows connection status ("Robot connected"/"Robot
offline"), Run/Stop
buttons, Voice Command, program templates, run history, backend saved
programs, named locations, plan checks, and the Generated Plan created from
the connected blocks (some of these sit further down the panel than the
screenshot above scrolls to — each gets its own close-up below).

The top toolbar (visible above the workspace in the screenshot) contains
local program controls. `Save` and `Load` use browser storage, `Import` and
`Export` move Blockly programs as JSON files, and `Reset` clears the
workspace back to the starter program.

### Program Templates

![OpenAMR UI Programs page Program Templates panel with a template selector and Load Template button](<../assets/programs/ProgramTemplates.png>)

`Program Templates` provides ready-made examples such as a safe motion test or
low-battery docking routine. Selecting a template shows a short description;
`Load Template` then replaces the current workspace with that template's real
Blockly blocks. Loading does not execute the program. Review the resulting
blocks, Plan Checks, and Generated Plan before pressing `Run`.

### Backend Programs

`Backend Programs` stores Blockly workspaces through the Flask backend
(further down the right panel, past Run History). Enter a program name and
press `Save`, choose an existing program and press `Load`, or remove the
selected program with `Delete`. `Refresh` reloads the list from the backend.
This is separate from the toolbar's browser-local `Save` and `Load`: backend
programs remain available even if browser storage is cleared and can be
opened from another browser that reaches the same UI server.

### Run History

![OpenAMR UI Programs page Run History panel listing two past runs with success/stopped badges](<../assets/programs/Runhistory.png>)

Every `Run` (however the program was built) is logged here — name, result
badge, timestamp, step count, and duration. `Refresh` reloads the list;
`Clear` empties it. Useful for confirming a program actually ran, and how
long it took, without needing the Events or Metrics pages open.

### Named Locations

![OpenAMR UI Programs page Named Locations panel with a saved "Charging Station" location and its x/y/yaw](<../assets/programs/named-location.png>)

`Named Locations` associates a readable name, such as `Charging Station`,
with a map pose: `x`, `y`, and `yaw`. These entries populate the
`navigate to location` block so programs do not have to repeat raw coordinates.
`Save Location` creates or updates an entry, `Delete` removes the selected
entry, and `Refresh` fetches the latest list from the backend. Deleting a
location that a program still references causes Plan Checks to flag that plan.

A Blockly program should start with `start robot program`. Robot actions
must be connected below that start block to run. Loose blocks can remain on
the workspace while you're experimenting, but they aren't part of the
generated robot plan unless connected under the start block the planner
reads — the same point made below in
[The pipeline](#the-pipeline-block--action--execution--ros). For real robot
tests, keep one start block in the workspace to avoid confusion.

The Plan Checks panel reports safety and validation warnings before
execution; the Generated Plan panel shows the exact step list built from
connected blocks — see
[Plan Checks](#plan-checks-one-safety-gate-regardless-of-origin) below.

![OpenAMR UI Programs page Plan Checks panel showing the configured speed limits and "Ready. No validation warnings."](<../assets/programs/Planchecks.png>)

![OpenAMR UI Programs page Generated Plan panel listing three queued steps](<../assets/programs/generatedplan.png>)

The status beside each generated step starts as `QUEUED` and changes as the
program runs. The step count and ordered descriptions are a final preview of
what the executor will do; if they do not match the intended behavior, edit
the blocks before running.

## The pipeline: block → action → execution → ROS

Every block, however it got onto the workspace (dragged, loaded from a
template, or generated from a voice command — more below), goes through the
same four-stage pipeline before anything reaches the robot:

```text
Block in the toolbox / workspace
        |
        v
Block definition               (web/src/features/blocks/blockDefinitions.js)
        |
        v
Action object in the Generated Plan   (e.g. "navigate", "wait", "dock")
        |
        v
Execution logic                (web/src/features/blocks/robotActions.js)
        |
        v
ROS topic or service, over the shared connection (Lesson 10)
```

`blockDefinitions.js` defines what each block looks like and converts a
connected chain of blocks into a flat list of plain action objects — the
**Generated Plan** shown in the right panel. `robotActions.js` is the only
place that actually talks to ROS: it walks that action list step by step and
publishes/calls the matching topic or service for each one (the same
`AppConfig` constants used everywhere else in the UI — see
[Lesson 10](10-topics-as-the-contract.md) — not a separate set of names).
Splitting "what a block means" from "how it executes" this way is what makes
it possible for a block, a saved template, and a voice command to all produce
the exact same kind of Generated Plan and go through the exact same execution
path.

Only blocks connected below the single `start robot program` block are read
into the Generated Plan — loose blocks sitting elsewhere on the workspace are
ignored, which is why the guide's "0 steps" troubleshooting entry exists.

## Block categories, at a glance

The toolbox ([`web/src/features/blocks/toolbox.js`](../../web/src/features/blocks/toolbox.js))
groups blocks into five categories, each mapping to one theme covered
elsewhere in these lessons:

| Category | What it's for |
| --- | --- |
| **Program** | Structure — the required `start` block, `repeat`, and `log` for debugging. Doesn't move the robot by itself. |
| **Navigation** | Nav2 goals — coordinate or named-location goals, waiting on navigation status, patrol loops. The same `/goal_pose` and navigation-status mechanics as the Map page ([Lesson 06](06-the-pages.md#map--mappagejsx)). |
| **Motion** | Direct `/cmd_vel` commands — drive, rotate, stop, emergency stop. Unlike Navigation, these don't plan around obstacles. |
| **Docking** | Publishes the same dock/undock trigger topics as the Map page's `DockingControl` panel ([Lesson 07](07-ui-components.md#dockingcontrol--dockingcontroljsx)). |
| **Robot State** | Reads battery data or publishes a UI mode string — the only category that branches on robot state rather than just commanding the robot. |

For the full reference — every block's exact fields, screenshots, and worked
examples — see
[the category reference in the practical guide](../../web/src/features/blocks/README.md#current-block-categories).

## Plan Checks: one safety gate, regardless of origin

Before `Run` is enabled, `planValidation.js` checks the Generated Plan for
problems — speeds over the configured limit, a named location that no longer
exists, and so on — and the page asks for confirmation before running
anything containing direct motion, docking, or emergency stop. This gate
applies identically no matter how the plan was built: hand-dragged blocks, a
loaded template, or a voice command all produce the same kind of Generated
Plan and pass through the same checks. There's no "trusted" path that skips
validation.

## Voice Command

The `Voice Command` panel is an alternate way to *build* a plan — it still
goes through the exact same pipeline and safety gate above, it just adds a
speech-to-plan step in front of it:

![OpenAMR UI Programs page Voice Command panel with a "Tap to speak a command" button and transcript field](<../assets/programs/Voicecommand.png>)

Tap the button, say the wake word `Monsieur`, and then speak the command. The
panel shows the recognized transcript only after the wake word is heard. A
short pause ends capture and sends the command for plan generation.

```text
You speak
        |
        v
Browser Web Speech API produces a transcript
        |
        v
Everything up to and including the wake word ("Monsieur") is stripped
        |
        v
POST /api/voice-plan  (Flask backend, web/src/features/blocks/voicePlan.js)
        |
        v
Claude API returns a structured action plan
        |
        v
planToWorkspace() (blockDefinitions.js) turns it into real Blockly blocks
        |
        v
The workspace updates — same Generated Plan, same Plan Checks, same Run button
```

Two things worth understanding conceptually, not just operationally: Claude
is constrained to the exact same action types the blocks already define — the
`/api/voice-plan` endpoint in
[`flask_app.py`](../../ros2/src/openamr_ui_package/openamr_ui_package/flask_app.py)
forces a tool call against that fixed schema and drops anything that doesn't
match, so voice can't invent a new kind of action the executor wouldn't
recognize. And voice only ever *builds* blocks — it never runs anything by
itself; the resulting plan is just an ordinary Generated Plan that still
needs a manual `Run` press and still passes through Plan Checks like any
other program.

For setup requirements (browser support, secure-origin microphone rules, API
key), the exact wake-word behavior, and troubleshooting, see
[Voice Command in the practical guide](../../web/src/features/blocks/README.md#voice-command).

## Try it

In Demo Mode, load a template, inspect its generated plan and validation
warnings, then run it and review the run-history result. Do not repeat on real
hardware until the operating area and physical emergency stop are ready.

**You're ready to continue when:** you can trace one block from workspace to
generated action and identify the topic, service, or browser wait it uses.

## Next

[Lesson 10 — Topics as the Contract](10-topics-as-the-contract.md) steps back
from individual pages and pipelines to explain why the topic *names* used
throughout every page — including the ones Blockly publishes to — are the
real interface between the UI and the robot.

---

[← Lesson 08](08-map-and-route-model.md) · [Lesson index](README.md) ·
[Next: Lesson 10 →](10-topics-as-the-contract.md)
