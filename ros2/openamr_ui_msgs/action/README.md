# About

**The action folder** holds every goal based action that the UI can use. The main one: **MoveBase.action** tells the robot to move to a waypoint.

# Adding actions

An action is composed of three variables, the **request**, which defines a new goal (this could be a new
waypoint), a **result**, which is sent back when the request finishes, and **feedback**, which are updates
about the current request.

Here is the template to create an action, the 3 hyphens are necessary:
```
RequestType Request
---
ResultType Result
---
FeedbackType Feedback
```

# Further steps

For more in depth knowledge about implementing an action, the
ROS2 foundation’s [tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Creating-an-Action.html) are a great starting point.


