# Lesson 00 — Your First 10 Minutes

| Audience | Time | Prerequisites |
| --- | --- | --- |
| First-time operators and evaluators | 10 minutes | A running UI; no robot required for Demo Mode |

## What you'll learn

You will learn how to enter Demo Mode, recognize the shared controls, open the
Health page, try a navigation goal without moving hardware, and prepare for a
safe real-robot session.

## Before touching a real robot

> [!CAUTION]
> The dashboard's red **E-STOP** is a software stop. It publishes one
> zero-velocity command and asks Nav2 to cancel the active goal. It is not
> latched or safety-rated and depends on the browser, network, rosbridge, and
> robot controller. Always keep a tested physical emergency stop within reach.

For real hardware:

- Clear people, pets, cables, tools, and loose objects from the operating area.
- Confirm the physical emergency stop works before enabling motion.
- Use one active operator and tell nearby people that the robot may move.
- Begin with the lowest practical linear and angular speed limits.
- Do not send a goal until the map, robot pose, and live surroundings agree.
- Treat Scheduler, Missions, and Programs as motion commands, not harmless UI
  previews.

## Step 1: open the dashboard

Open:

```text
http://127.0.0.1:5050/
```

If the UI runs on another computer, replace `127.0.0.1` with that computer's
IP address.

The first-run guide appears once per browser profile. Choose **Explore without
a robot**. If it was previously dismissed:

1. Open **Config** from the sidebar.
2. Enable **Demo Mode**.
3. Return to **Map**.

You should see a purple Demo Mode banner and a green connection indicator.
That green indicator represents the simulated browser transport; it does not
mean a physical robot is connected.

## Step 2: learn the controls shared by every page

The shared layout contains:

- The sidebar for moving between pages.
- A connection indicator showing the browser-to-rosbridge state.
- Battery status.
- The red software-stop control.
- A **?** help button with page-specific guidance.

Connection and robot-data freshness are different. A green connection means
the browser can talk to rosbridge. It does not prove that `/map`, `/odom`, or
other robot topics are fresh. The Health page checks those separately.

## Step 3: explore the Map page safely

In Demo Mode:

1. Toggle map layers and identify the simulated robot pose.
2. Select **Goal Mode** and place a practice goal.
3. Open the saved-waypoint area and inspect its actions.
4. Move the joystick briefly and release it.
5. Press the red software-stop control once to learn where it is.

Demo Mode intercepts these actions in the browser. Repeat them on real
hardware only after completing the real-robot checklist below.

## Step 4: inspect system readiness

Open **Health** and find:

- Overall readiness.
- ROS connection state.
- Topic freshness.
- Nav2 lifecycle state.
- Battery and registered-device status.
- Recent faults and diagnostic export.

Use the issue details rather than relying on the green connection indicator
alone.

## Step 5: understand background automation

Scheduler and Missions run in the browser tab:

- The tab must stay open for a scheduled action to fire.
- Closing, suspending, or refreshing the tab can interrupt browser-side
  automation.
- Do not assume a browser restart resumes an interrupted mission safely.
- Stop automation and verify the robot is stationary before closing the UI.

Blockly Programs also require an active connection while running. A network
drop can interrupt a wait or service call without completing the remaining
steps.

## Switching from Demo Mode to a real robot

1. Start the robot or simulation workspace.
2. Start the UI workspace.
3. Open **Config** and turn Demo Mode off.
4. Confirm the expected rosbridge address.
5. Open **Health** and wait for required topics to become fresh.
6. Confirm localization visually on the Map page.
7. Set conservative speed limits.
8. Confirm the physical emergency stop is reachable.
9. Make one short, low-speed movement or nearby goal.
10. Stop and verify that the robot is stationary before continuing.

If the UI is green but the map or pose is frozen, do not drive. Continue with
[Lesson 11](11-failure-modes-and-reconnection.md) or the plain-language table
in [Lesson 12](12-debugging-with-ros-cli.md).

## Ending a session

1. Stop active Programs, Missions, recordings, and manual motion.
2. Use the software stop if motion is still active, then verify the robot
   physically stops.
3. Follow the robot platform's shutdown or docking procedure.
4. Stop the UI process with `Ctrl+C`, or use `docker compose down`.
5. Never rely on closing the browser tab as an emergency-stop method.

## Try it

Complete Steps 1–4 in Demo Mode and locate the physical emergency stop you
would use during a real session.

**You're ready to continue when:** you can explain the difference between a
green connection indicator, fresh robot data, the dashboard's software stop,
and the robot's physical emergency stop.

## Next

[Lesson 01 — What Is This UI?](01-what-is-this-ui.md) explains which parts of
the complete robot system belong to this repository.

---

[← Main README](../../README.md) · [Lesson index](README.md) ·
[Next: Lesson 01 →](01-what-is-this-ui.md)
