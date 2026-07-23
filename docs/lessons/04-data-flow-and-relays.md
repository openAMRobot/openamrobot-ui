# Lesson 04 — Data Flow and Relays

This is the most important lesson for anyone planning to connect a new
external device, sensor, or actuator to the UI (see
[`docs/extending/connect-external-device.md`](../extending/connect-external-device.md)).
It explains a pattern that repeats every time robot-side data needs to reach
the browser reliably: **robot topic → relay node → browser-safe topic**.

## The problem: QoS, not code

ROS 2 topics carry a Quality of Service (QoS) profile alongside their data —
a set of delivery-behavior settings each publisher and subscriber must agree
on (close enough) to actually connect. QoS has several independent settings;
the only one that matters for this lesson is **durability**, which has two
values:

- **TRANSIENT_LOCAL** ("latched"): a late-joining subscriber still receives
  the last published message, not just future ones. The map server publishes
  `/map` this way — the map doesn't change every frame, so it's published
  once (or infrequently) and any node that starts up later still gets it.
- **VOLATILE**: only subscribers that are already listening receive new
  messages. Nothing is replayed for late joiners.

(There's a separate setting called *reliability* — RELIABLE vs BEST_EFFORT,
about whether dropped packets get retried — but it isn't the source of the
problem this lesson solves, so it's not discussed further here.)

Nav2 nodes expect and handle TRANSIENT_LOCAL correctly. A browser client
connecting through rosbridge, though, is much more likely to subscribe
*after* a TRANSIENT_LOCAL topic's one-time publish already happened — and
depending on timing, it can miss it. The fix used throughout this workspace
is a small relay node: subscribe to the robot-side TRANSIENT_LOCAL topic,
republish the same data as VOLATILE (and, for the map, on a timer so
late-connecting browser clients still get it) on a separate topic name.

**The symptom this actually prevents:** without the relay, if you open the
Map page a few seconds after `map_server` started (which is the normal case —
the robot stack is almost always already running before you open the UI),
the map panel could stay blank forever, even though the map server published
correctly and `ros2 topic echo /map` on the command line would show the data
was there. The relay's 2-second republish timer is what guarantees a
just-opened browser tab gets a copy no matter when it subscribed.

## The pattern in this codebase

```text
Robot / simulation stack            This UI workspace                Browser
--------------------------          ------------------------         ---------
/map (TRANSIENT_LOCAL)      --->    map_relay node          --->    subscribes to
  published by map_server           republishes as VOLATILE          /ui/map
                                     on /ui/map, every 2s
```

See
[`ros2/src/openamr_ui_package/openamr_ui_package/map_relay.py`](../../ros2/src/openamr_ui_package/openamr_ui_package/map_relay.py)
for the exact implementation — it is intentionally small and is a good
template to copy for a new relay.

The same pattern covers navigation and docking status, all handled by one
node:
[`ros2/src/openamr_ui_package/openamr_ui_package/nav_relays.py`](../../ros2/src/openamr_ui_package/openamr_ui_package/nav_relays.py),
which relays:

| Robot-side topic (TRANSIENT_LOCAL) | UI-facing topic (VOLATILE) |
| --- | --- |
| `/amcl_pose` | `/ui/amcl_pose` |
| `/navigate_to_pose/_action/status` | `/ui/navigate_to_pose/status` |
| `/dock_robot/_action/status` | `/ui/dock_robot/status` |
| `/undock_robot/_action/status` | `/ui/undock_robot/status` |

Both relay nodes are started alongside Flask and rosbridge in
[`ros2/src/openamr_ui_package/launch/new_ui_launch.py`](../../ros2/src/openamr_ui_package/launch/new_ui_launch.py),
under the `ui` namespace.

## Not every topic needs a relay

Plenty of topics the UI subscribes to — `/odom`, `/scan_filtered`,
`/global_costmap/costmap`, `/plan`, `/tf`, `/tf_static` — are already VOLATILE
or don't have the late-subscriber problem in practice, so they're subscribed
to directly with no relay in between. A relay is a targeted fix for a
specific QoS mismatch, not a mandatory hop for every topic. Whether a new
topic needs one is exactly the question
[`docs/extending/connect-external-device.md`](../extending/connect-external-device.md)
walks through.

## The naming convention

Relayed topics live under a `/ui/` prefix (`/ui/map`, `/ui/amcl_pose`, …).
That prefix is a convention, not a technical requirement — it exists so
anyone reading a topic list can tell at a glance which topics are
browser-facing relays versus original robot-side topics. Follow it when
adding a new relay.

## Next

[Lesson 05 — Backend Nodes in Detail](05-backend-nodes-in-detail.md) finishes
the ROS-side tour — the rest of what `openamr_ui_package` runs beyond the two
relays — before [Lesson 06 — The Five Pages](06-the-five-pages.md) walks
through what the Map, Route, Control, and Info pages actually show, and which
topics (relayed or direct) each one depends on.
