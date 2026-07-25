---
title: Home
---

# OpenAMRobot UI Documentation

OpenAMRobot UI is a browser dashboard for seeing and controlling a robot
that's already running elsewhere — maps, routes, manual driving, Blockly
programs, missions, diagnostics, and more, all talking to ROS 2 through
rosbridge. This site is the documentation for it. The code itself lives at
[github.com/rajindulkar22/openamrobot-ui](https://github.com/rajindulkar22/openamrobot-ui);
start there for the [full README](https://github.com/rajindulkar22/openamrobot-ui/blob/theory_practical/README.md).

> **Heads up:** the lesson pages link out to source files
> (`web/src/pages/...`) and a few repo-root docs (`README.md`,
> `CONTRIBUTING.md`) using relative links. Those resolve correctly when
> you're reading this documentation on GitHub itself, but this published
> site only contains the `docs/` folder — so a link like that will 404
> here. If you hit one, you're one click away: open the same page from the
> [repository](https://github.com/rajindulkar22/openamrobot-ui/tree/theory_practical/docs)
> instead.

## Get started

- [Installation and Launch Guide](installation.md) — Docker Compose or
  manual install, ports, backups, and uninstalling.
- [Development Guide](development.md) — dev server, the production build
  pipeline, and where to make a given kind of change.
- [Troubleshooting Guide](troubleshooting.md) — a symptom-first table for
  when something isn't working.

## Learn how it works

The lessons are a numbered curriculum (00–13) meant to be read in order the
first time — each one names its audience, a reading-time estimate, and
prerequisites, and links straight to the real source instead of pasting
code that can go stale.

- [Lessons index](lessons/README.md) — the full list, plus shorter reading
  paths for specific roles (operator, panel developer, Blockly, etc.).
- [Glossary](lessons/glossary.md) — quick term lookup without reading a
  full lesson.

## Extending the system

- [Extending guides](extending/README.md) — hands-on, task-first guides for
  adding a UI panel, connecting a new device, or adding a Blockly block,
  plus one complete worked example.

## Architecture at a glance

![OpenAMRobot UI architecture: browser, rosbridge, UI backend nodes, and the robot/simulation stack](<assets/openamr_ui_architecture.svg>)

For the full picture, [Lesson 03](lessons/03-how-the-browser-talks-to-ros.md)
walks through this chain in detail, and a
[narrated feature tour](assets/openamrobot_ui_feature_tour_with_audio.mp4)
([transcript](assets/openamrobot_ui_feature_tour_transcript.md)) walks
through the UI itself page by page.
