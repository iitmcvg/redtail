This document describes the ROS nodes that are used or implemented by the project.

1. [Camera node](#camera-node)
2. [DNN node](#dnn-node)
    1. [TrailNet](#trailnet)
    2. [YOLO](#yolo)
    3. [INT8 inference](#int8-inference)
    4. [Running tests](#running-tests)
3. [Controller node](#controller-node)
3. [Image publisher node](#image-publisher)

# Camera node
The camera node (`gscam`) publishes to the `/camera/image_raw` topic using the standard ROS [Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) message. This is the master source which provides information to other nodes like the TrailNet DNN, and obstacle avoidance.

# DNN node
The `caffe_ros` node provides support for DNN inference using [TensorRT](https://developer.nvidia.com/tensorrt) library. This node currently supports Caffe models such as classification (TrailNet), YOLO-based object detection, regression and semantic segmentation. The node produces messages in different formats depending on the type of the network and post-processing options (e.g. TrailNet vs YOLO).

The timestamp in the header of the message is the same as the timestamp from the corresponding camera message. This simplifies synchronization of the image frames with DNN output and other nodes for the purpose of debugging and logging.

## TrailNet

This node publishes output of the DNN (e.g. softmax layer) using standard [Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) message.
* for `1D` output like that of the softmax layer, the dimensions of the published image are `1x1xC` where `C` is the size of the layer (e.g. 6 for the TrailNet `out` layer or 1000 for ImageNet's `softmax` layer). 
* The `encoding` field of the message is in the following format: `32FCXXX` where `XXX` is equal to number of channels. For example, for TrailNet: `32FC6`, for ImageNet: `32FC1000`. `32` means 32-bit and `F` means float. See the ROS documentation for more details. 
* The `data` field contains the output of the DNN in byte array representation so it should be cast to a 32-bit float type.

The default topic name is `/caffe_ros/network/output` but it can be changed using launch file or command line.

The node requires two parameters:
* `prototxt_path` - path to the Caffe model .prototxt file
* `model_path   ` - path to the Caffe model binary file
Additionally, make sure that other parameters, like the input/output layer names (`input_layer`/`output_layer`) are set correctly. An example of the command line for TrailNet:
```
rosrun caffe_ros caffe_ros_node __name:=trails_dnn _prototxt_path:=/data/src/redtail/models/pretrained/TrailNet_SResNet-18.prototxt _model_path:=/data/src/redtail/models/pretrained/TrailNet_SResNet-18.caffemodel _output_layer:=out
```

**Note**: when using `rosrun` remember to prefix parameters with `_` (`__` for system parameters) and use `:=` when assigning a value.

To check that the DNN node is producing some data, make sure that camera node (a real camera via `gscam` or an [image_pub](#image-publisher) node) is running, and execute `rostopic echo /trails_dnn/network/output`. An example of the output:
```
header: 
  seq: 201
  stamp: 
    secs: 1501106944
    nsecs: 796543294
  frame_id: ''
height: 1
width: 1
encoding: 32FC6
is_bigendian: 0
step: 24
data: [110, 74, 230, 58, 135, 125, 95, 60, 228, 14, 124, 63, 133, 26, 6, 61, 156, 75, 115, 63, 91, 87, 138, 60]
```
To convert the `data` byte array to float array, use this simple Python script:
```python
import struct
data = bytearray([110, 74, 230, 58, 135, 125, 95, 60, 228, 14, 124, 63, 133, 26, 6, 61, 156, 75, 115, 63, 91, 87, 138, 60])
print(struct.unpack('<%df' % (len(data) / 4), data))
```
## YOLO
In case the node is launched with `post_proc:=YOLO` argument, then a special YOLO post-processing is applied. In such case, the node publishes output of the DNN using standard [Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) message in a certain format: the output is a 2D, single-channel "image" that has the following format: `WxHx1` (so encoding == `32FC1`) where `W` is fixed and equals 6, and `H` is equal to the number of detected objects. For example, if the DNN has detected 2 objects, then the output is `6x2` image. For each detected object, the 6 values are the following:

    0  : label (class) of the detected object (e.g. person or a dog).
    1  : probability of this object.
    2,3: x and y coordinates of the top left corner of the object in image coordinates.
    4,5: width and height of the object in image coordinates.

All values are 32-bit floats, including label. Label indices correspond to 20 classes from [PASCAL VOC 2012](http://host.robots.ox.ac.uk/pascal/VOC/voc2012/htmldoc/index.html) dataset.

For example, if DNN detected a person (label:14) and a dog (label: 12) in the image with dimensions 320x180 then the output might look something like that:

| Label | Prob | X | Y  | Width | Height |
| ---- | --- | ----- | ----- | ---- | -----|
| 14.0 | 0.5 | 120.0 | 80.0  | 30.0 | 60.0 |
| 12.0 | 0.4 | 160.0 | 115.0 | 40.0 | 20.0 |

## INT8 inference
Running inference with `INT8` precision improves both performance (speed) and power efficiency on hardware that has native `INT8` support. Note that at the moment, `INT8` is supported on GP102-GP106 Pascal GPUs. Jetson TX1 and TX2 do **not** support `INT8` instructions so in this case `caffe_ros` node will use `FP32` mode. For more details on INT8 hardware support and implementation details in TensorRT see [here](https://devblogs.nvidia.com/mixed-precision-programming-cuda-8/) and [here](http://on-demand.gputechconf.com/gtc/2017/presentation/s7310-8-bit-inference-with-tensorrt.pdf).

redtail 2.0 introduces new parameter in `caffe_ros` node: `data_type` which accepts the values: `fp32`, `fp16` and `int8`. Note that existing parameter, `use_fp16`, while deprecated, is still supported.

Using `INT8` mode requires additional step to ensure no loss in model quality: model calibration. During model calibration, the network is fed with representative sample from a dataset so relevant statistics can be collected and then used to perform runtime `INT8` inference with as little loss in accuracy as possible. The sample calibration batches must be as close to what the network will "see" during runtime. The calibration needs to be performed only once and then can be stored as a file that will be used later during runtime.

`caffe_ros` node now supports 2 parameters that enable calibration:
* `int8_calib_src` provides a path (directory or a single image file) to calibration samples. If parameter is directory, all files will be read and used in calibration process. This process may take some time depending on the number of images. Once calibration is performed, this parameter can be removed (or set to empty string) to avoid re-calibration on every run.
* `int8_calib_cache` specifies path to calibration cache. If the file exists, it will be loaded and used by `caffe_ros` runtime. If it does not exist, it will be created by calibration process (this requires `int8_calib_src` parameter to be set).

Our `TrailNet` and `YOLO` models were trained as `FP16`-aware models so there is almost no difference in results produced by these networks in `FP32` and `FP16` modes. However, no special care was taken to make sure the networks also work in `INT8` mode so for these network results obtained in `INT8` might be different from results in `FP32`/`FP16` modes.


## Running tests
To verify that the DNN node is working properly, run the basic end-to-end tests that come with `caffe_ros` package. In the catkin workspace directory, run the following (replace paths if needed):
```
catkin_make caffe_ros_tests
export REDTAIL_TEST_DIR=/data/src/redtail/ros/packages/caffe_ros/tests/data
export REDTAIL_MODEL_DIR=/data/src/redtail/models/pretrained
rostest caffe_ros tests_basic.launch test_data_dir:=$REDTAIL_TEST_DIR \
    trail_prototxt_path:=$REDTAIL_MODEL_DIR/TrailNet_SResNet-18.prototxt trail_model_path:=$REDTAIL_MODEL_DIR/TrailNet_SResNet-18.caffemodel \
    object_prototxt_path:=$REDTAIL_MODEL_DIR/yolo-relu.prototxt object_model_path:=$REDTAIL_MODEL_DIR/yolo-relu.caffemodel
```
**Note**: running the tests on the device like TX2 may take about two minutes.

# Controller node

# Image publisher
The `image_pub` node is a simple ROS node that reads video or image files and publishes frames as a ROS topic as an [Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) message. There is one mandatory parameter, `img_path`, which specifies the path to an image or video file. This node supports setting custom frame rates as well as repeating an image indefinitely.
The default topic is `/camera/image_raw` which can be changed using the `camera_topic` parameter.

An example of publishing frames from the video file using the file's native frame rate:
```
rosrun image_pub image_pub_node _img_path:=/data/videos/trail_test.mp4
```

And example of publishing the same image repeatedly at 30 frames/sec:
```
rosrun image_pub image_pub_node _img_path:=/data/images/trail_right.png _pub_rate:=30 _repeat:=true
```
