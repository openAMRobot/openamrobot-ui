# About

**The service folder** holds every simple instruction that the different nodes could execute.

# Adding services

A service is composed of two sides: the **request** which is a list of variables given to the service and
the **response** which is the service’s output using the request’s data. This is akin to a programming
function’s **arguments and return value**

Here is the template to create a service, the 3 hyphens are necessary (Note: There can be as many variables in the request as one wishes, but they have to be typed
properly):
```
Type1 Var
Type2 Var
Type3 Var
---
ResponseType Response
```

# Further steps

For more in depth knowledge on using services, the
ROS2 foundation’s [tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html) are a great starting point.
