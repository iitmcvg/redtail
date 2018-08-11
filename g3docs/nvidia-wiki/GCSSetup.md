# Ground Control Station (GCS) setup

Almost any modern laptop can be used to run GCS software and the other software components to control the drone.
We recommend using laptop running Ubuntu 16.04 or higher (Windows should work too but has not tested).

## GCS software
There are currently several popular GCS applications, including [QGroundControl](http://qgroundcontrol.com/) and [APM Planner](http://ardupilot.org/planner2/). We chose to use QGroundControl.

## Joystick support
Controllers like the NVIDIA Shield or wire Microsoft XBox gaming controller can be used to control the drone remotely. In such scenario, the GCS machine (laptop) needs to run a [ROS joystick](http://wiki.ros.org/joy) node and be able to connect to the ROS master running on the NVIDIA Jetson. The easiest way to install the required components on a laptop is to use a ROS Docker container. Using Docker allows you to isolate your host system from any changes, such as software installation and configuration updates.
To use ROS Docker, first obtain the ROS Docker image:
```sh
docker pull ros:kinetic
```
next, create and run the Docker container:
```sh
docker run -it --privileged --network=host --name=ros-joy ros:kinetic bash
```
This command will create a container named `ros-joy` that has access to USB devices (such as the joystick) (`--privileged`) and uses the host network (`--network=host` option) for simpler ROS network config in the future.
Next, while running in the container, install and verify the ROS joystick node:
```sh
apt-get update
apt-get install ros-kinetic-joy
roscore &
rosrun joy joy_node _dev:=/dev/input/js1 &
rostopic echo /joy
```
Move the joystick sticks or press buttons and you should see the output in the console.
Notes:
1. The system can have several "joystick" devices e.g. in some cases the laptop trackpad or mouse can be detected as a joystick. Use `_dev` parameter to select correct device.
2. If a joystick was not attached before the container was created/started running, then container needs to be restarted:
    ```sh
    docker start ros-joy && docker attach ros-joy
    ```

