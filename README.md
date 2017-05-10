## Compiling

Supposing you use [this guide](http://vasalf.net/au/ros-build.html) to build ROS from git sources.

Clone this repo to your workspace source directory, then run from the root of your workspace:

```
$ . ./install_isolated/setup.bash
$ catkin_make_isolated --pkg test_intra_process_service_communication --install
```

## Running

```
$ . ./install_isolated/setup.bash
$ rosrun test_intra_process_service_communication single_thread
```

It should produce some messages every 0.5 seconds.

You can check whether the service is not responding with the following command:

```
$ rosrun test_intra_proces_service_communication call_from_outside
```

If everything is OK, it should terminate successfully.
