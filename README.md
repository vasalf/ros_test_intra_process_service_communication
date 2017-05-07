## Compiling

Supposing you use [http://vasalf.net/ros-build.html](this guide) to build ROS from git sources.

Clone this repo to your workspace source directory, then run from the root of your workspace:

```
$ . ./install_isolated/setup.bash
$ catkin_make_isolated --pkg ros_test_self_service --install
```

## Running

```
$ rosrun test_self_service test_self_service
```

It should produce some messages every 0.5 seconds.

You can check whether the service is not responding with the following command:

```
$ rosrun test_self_service call_from_outside
```

If everything is OK, it should terminate successfully.