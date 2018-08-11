This page describes how to prepare and fly a drone while running code on Jetson TX1/TX2 companion computer. Make sure you read and understand all of the steps and do a dry run **before** going out to the field in order to ensure that all of the components are working as expected.

1. [Safety](#safety-warning)
2. [Pre-launch steps](#pre-launch-steps)
3. [Running the code](#running-the-code)
4. [Flying](#flying)
5. [Troubleshooting](#troubleshooting)

# Safety warning
Please make sure that you have a safe environment to fly the drone. In particular:
1. Make sure you know and obey all relevant local laws and regulations.
2. Make sure you have a way to disable drone motors directly from the transmitter via a "kill switch". Follow the [instructions](./3DR-Iris-Setup#calibration-and-controls) to set up this switch.
3. For the very first experiments we recommend flying the drone in an open field where operating RC aircraft and drones is permitted. As an additional safety measure, we suggest tying the drone to the ground using sufficiently long and lightweight rope.
4. Make sure the drone flies stably and with no issues before running the code on Jetson and switching to autonomous mode.

# Pre-launch steps
There are several steps that need to be done before launch.

## Powering on
1. Power on the drone by connecting the battery. 
2. If you rely on **PX4FLOW module** for stabilization and visual odometry do the following:
    - Wait for Pixhawk's initialization beep and then lift the drone for 20-30cm above the ground. 
    - Wait for the drone's LED light to turn green (if Iris+ and Pixhawk are used). PX4FLOW is initialized when this happens.
    - Put the drone back on the ground
3. Close the battery compartment and arm Pixhawk by pressing and holding the arm button. The button's LED light should start producing fast and short blinks. If this happens, the drone is armed. If not, re-check your hardware and Pixhawk's firmware.

## Jetson initialization
Check that the Jetson is powered as well. Wait a minute or two for the Jetson to boot, and connect to its WiFi network. Next, SSH to the board:

`ssh nvidia@10.42.0.1`

To speed up this login process and avoid entering the password on each login, you can define the following alias:

```sh
alias sshtx="sshpass -p 'nvidia' ssh nvidia@10.42.0.1"
```

This assumes default user/password used on Jetson. Note that in previous versions of JetPack (e.g. 3.0), the default user/password combination was different: `ubuntu/ubuntu`. For better security, it is strongly recommended to change the password **before** going to the field/trail.

## Building the code
Make sure you have the latest source code installed on the Jetson. If you ran the setup script as a part of Jetson setup, then the code should be in `~/redtail` directory. Pull and build the latest version if needed (connect network cable to the Jetson carrier board):

```sh
cd ~/redtail
git pull
cd ~/ws
catkin_make
```

**TIP:** you can mount Jetson home directory to any directory on the host computer to simplify access to Jetson filesystem by using `sshfs` tool:

```sh
sudo sshfs -o idmap=user -o allow_other nvidia@10.42.0.1:/home/nvidia /home/[your_directory]/tx2-01/
```

# Running the code
Once the code is built, there are multiple options to run the ROS components. 

## Running all drone and ground station components
There are total of 3 components that run on the drone and ground station. Each component is launched via `roslaunch` or `rosrun` command.
### Navigation components
These include TrailNet DNN, YOLO DNN, MAVROS and camera ROS nodes. To run all of these components, execute the following command from `~/ws` directory on the Jetson:
```
roslaunch caffe_ros everything.launch
```
Take a look at `~/redtail/ros/packages/caffe_ros/launch/everything.launch` for more details on what parameters are supported.

### Joystick
To enable drone control with a joystick, connect the joystick (e.g. NVIDIA Shield or Microsoft XBox controller) to your ground station computer and run the `joy` ROS node on the ground station:

`rosrun joy joy_node _dev:=/dev/input/js0 _autorepeat_rate:=30`

You might need to change input device if you have multiple joysticks. Make sure to set the following environment variables (either using `export` or via `.bashrc` file):

```sh
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_IP=your_ground_station_computer_ip
```

`ROS_MASTER_URI` should point to machine where ROS master is running (e.g. Jetson) while `ROS_IP` is the IP address of the machine on which the joystick node is running (use `ifconfig` to determine this address). If you are running the joystick node from a Docker container, make sure the joystick is connected to the computer before starting the container.

### Controller
**NOTE:** it is strongly recommended to run the command below when the drone is already in the air and flying in **POSITION CONTROL** mode. In such case altitude gain should be set to 0 otherwise, the initialization procedure in px4_controller node will try to change the drone's altitude according to the setting which may lead to sudden changes in the altitude.
```
rosrun px4_controller px4_controller_node _altitude_gain:=0 _linear_speed=3 _joy_type:="shield" _obj_det_limit:=0.3
```

## Running drone and ground station components separately
Sometimes you may wish to launch components separately. To do that you will need to run MAVROS first via `roslaunch` (which will run ROS master as well as the MAVROS node) and then components of your choice via `rosrun` commands.

### MAVROS node
```
roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600" gcs_url:="udp://@10.42.0.85"
```
**NOTE:** you might need to change IP address in `gcs_url` parameter depending on your host WiFi configuration. Run the `ifconfig` command to make sure you are using correct IP address otherwise QGroundControl will not be able to connect to the drone.

### Camera node
First, make sure that Jetson board has the camera attached and it is displayed in `ls /dev/video*` output.
To enable camera feed publishing in ROS, run `gscam` node:
```sh
cd ~/ws/
rosrun gscam gscam _gscam_config:="v4l2src device=/dev/video0 ! video/x-raw, width=640, height=360 ! videoconvert"
```
If you want to enable streaming video to ground station using H.265 encoder, use a different value of the `_gscam_config` parameter:
```sh
rosrun gscam gscam _gscam_config:="v4l2src device=/dev/video0 ! tee name=t ! queue ! videoconvert ! omxh265enc ! video/x-h265, stream-format=byte-stream ! h265parse ! rtph265pay config-interl=1 ! udpsink host=10.42.0.211 port=6000 t. ! queue ! video/x-raw, width=640, height=360 ! videoconvert"
```
Change host IP address as needed.
You can also use environment variable instead of parameter to set `gscam` GStreamer pipeline:
```sh
export GSCAM_CONFIG="v4l2src device=/dev/video0 ! tee name=t ! queue ! videoconvert ! omxh265enc ! video/x-h265, stream-format=byte-stream ! h265parse ! rtph265pay config-interl=1 ! udpsink host=10.42.0.211 port=6000 t. ! queue ! video/x-raw, width=640, height=360 ! videoconvert"
rosrun gscam gscam
```

### TrailNet DNN node
To run the TrailNet DNN node, run `caffe_ros` node with the TrailNet model:
```sh
cd ~/ws/
rosrun caffe_ros caffe_ros_node __name:=trails_dnn _prototxt_path:=/home/nvidia/redtail/models/pretrained/TrailNet_SResNet-18.prototxt _model_path:=/home/nvidia/redtail/models/pretrained/TrailNet_SResNet-18.caffemodel _output_layer:=out _use_fp16:=true
```

### Object detection (YOLO) DNN node
To run the YOLO DNN node, run `caffe_ros` node with the YOLO model:
```sh
cd ~/ws/
rosrun caffe_ros caffe_ros_node __name:=object_dnn _prototxt_path:=/home/nvidia/redtail/models/pretrained/yolo-relu.prototxt _model_path:=/home/nvidia/redtail/models/pretrained/yolo-relu.caffemodel _output_layer:=fc25 _inp_scale:=0.00390625 _inp_fmt:="RGB" _post_proc:="YOLO" _obj_det_threshold:=0.2 _use_fp16:=true
```
# Flying
The steps below describe one possible way of starting autonomous flight, you may have a different sequence of steps if needed.
1. Make sure all components **except** the controller are running.
2. Take off and switch to **POSITION CONTROL** mode. Make sure the drone is flying stably (no drift etc).
    
    **NOTE:** if you don't use GPS, don't have automatic take off and use **PX4FLOW** module for stabilization and visual odometry you need to follow this procedure for taking off:
    - Take off in **STABILIZED** mode
    - Once airborne, switch to **ALT HOLD** mode and stabilize your drone in horizontal plane manually until the 
Pixhawk's LED light goes green, which indicates that PX4FLOW module is initialized and working properly. 
    - Switch to **POSITION CONTROL** mode. 

3. Position the drone in the desired location and orientation.
4. Run the controller node and wait until it switches to **NAVIGATE** mode (look at the console output).
5. Switch to the **OFFBOARD** mode from the transmitter. The light on the drone should blink and remain green. If it switches to purple then most likely controller node is not running properly.
6. Check that you can control the drone using joystick:
    - left stick: 
        - up/down: change altitude higher/lower
        - left/right: yaw
    - right stick:
        - up/down: fly forward/backward
        - left/right: strafe left/right
7. Position the drone on the trail and press and release **A** button on the joystick. The drone should start flying along the trail. You can disable autonomous mode and stop the drone using **B** button. You can also control the drone with the joystick even in autonomous mode - the joystick commands have higher priority than DNN.

# Troubleshooting
In case something is not working, try running each component separately and check if it's working as expected.
## Camera
Check that the camera is attached and working. Verify that Jetson has recognized the camera by running `ls /dev/video*` - you should see at least one video device in the output.

Run `gscam` node with H.265 streaming enabled and verify that you can receive the video on the host by running the following on the host:
```sh
gst-launch-1.0 udpsrc port=6000 ! application/x-rtp, encoding-name=H265,payload=96 ! rtph265depay ! h265parse ! avdec_h265 ! xvimagesink
```
You should see the window open with camera feed from the drone.

Next, check that the camera ROS topics are available:
```
rostopic list | fgrep camera
```
You should see 2 camera topics: `/camera/image_raw` and `/camera/camera_info`
Finally, check that the ROS topic is being published by running:
```
rostopic echo -n 1 /camera/image_raw
```
You should see (a lot!) of output.

## DNN nodes
To debug DNN nodes, run the `gscam` node first and then run the DNN node (TrailNet or YOLO depending on which you are trying to debug) using the appropriate `rosrun` command described above. Check whether the DNN node is publishing any data:
```
rostopic echo /caffe_ros/network/output
```
You should see the output of the DNN node. If not - check console output as well as ROS logs at `~/.ros/log/latest/`.

**NOTE:** if you are running DNN nodes using `everything.launch`, ROS topic names will be `/trails_dnn/network/output` for TrailNet and `/object_dnn/network/output` for the object detection net.

**YOLO DNN**: 
1. If YOLO DNN node fails with "file not found" error, make sure you have a [final model](../blob/master/models/pretrained/README.md).
2. If you are running the YOLO for the very first time, it may take up to 3-5 minutes for YOLO to be compiled and loaded. The compiled model will be cached so it's just first-time delay only.