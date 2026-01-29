# About

**The message folder** holds every types of data that could be transmitted between the different
nodes. The main message is **ArrayPoseStampedWithCovariance.msg** which lists all of the
waypoints on a single route.

# Adding messages

A message stores variables with a fixed type.

Here is the template to create a message, the 3 hyphens are necessary (Note: There can be as many
variables in the message as one wishes, but they have to be typed properly):

```

Type1 Var

Type2 Var

Type3 Var

```

# Further steps

For more in depth knowledge on using messages, the ROS2 foundation’s [tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) are a great
starting point.


