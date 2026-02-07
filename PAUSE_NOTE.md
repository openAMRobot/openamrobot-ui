STATUS AT PAUSE (2026-02-07):

- openamr_ui_msgs builds correctly
- MoveBase.action is generated and importable
- rosbridge_server starts but needed python3-pymongo/bson
- Flask serves React, static routing fixed
- UI loads index.html but still verifying static assets
- handler and nav start but:
  - handler depends on nav2_msgs (decision needed: required vs optional)
  - waypoint_nav design decision needed (custom MoveBase vs Nav2 NavigateToPose)

NEXT STEPS:
1) Decide Nav2 dependency policy
2) Finalize waypoint_nav implementation
3) Final UI ↔ rosbridge websocket verification
