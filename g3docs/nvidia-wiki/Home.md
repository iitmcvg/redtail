# Introduction

This project contains instructions, code and other artifacts that allow users to build mobile robots (drones, rovers) which can autonomously navigate through highly unstructured environments like forest trails. Our components use deep learning-based AI running on an NVIDIA Jetson embedded platform. The code implements ideas discussed in the arXiv paper, see [references](#references) section.

The project has two major parts, **modeling** and **platform implementation**.

## Modeling
The project's AI that enables autonomous navigation is based on a deep neural network (DNN) which can be trained from scratch using publicly available data. A few [pre-trained DNNs](../blob/master/models/pretrained/) are also available as a part of this project. In case you want to train TrailNet DNN from scratch, follow the steps on [this page](./Models).

## Platforms
The following platforms are currently supported:
* [TBS Discovery Platform](./Skypad-TBS-Discovery-Setup)
* [3DR IRIS+, older](./3DR-Iris-Setup)
* Differential drive rover support is coming

In general, any platform that uses the [Pixhawk](https://pixhawk.org/) autopilot should work too.

There is also an **experimental** support for APM Rover:
* [Erle Rover](./Erle-Rover-Setup)

# Getting started
Building a complete autonomous drone platform requires proper hardware and software configuration. 

## Jetson setup
The NVIDIA Jetson platform is used to run most of the components, such as DNN inference, the controller, and video streaming. The [Jetson setup guide](./Jetson-Setup) describes steps to install all required software and dependencies.

## Drone setup
Depending on the drone platform some additional steps might be required. Follow the steps in the documentation for your particular platform:
* [TBS Discovery Platform](./Skypad-TBS-Discovery-Setup)
* [3DR IRIS+, older](./3DR-Iris-Setup)

## GCS (Ground Control Station) setup
A laptop is a convenient way to run GCS software like [QGroundControl](http://qgroundcontrol.com/) as well to control the drone using an NVIDIA Shield or an XBox controller. Follow [these steps](./GCSSetup) to setup GCS machine.

## Simulation
It is usually a good idea to test your code in a simulator. Follow [these steps](./testing-in-simulator) to run simulations using Gazebo.

## Flying
Once the hardware and software setup steps are complete, it's time to take off! Follow [these steps](./Launch-Sequence-and-Flying) to fly the drone.

## References
* [arXiv paper](https://arxiv.org/abs/1705.02550)
* Our GTC 2017 talk: [slides](http://on-demand.gputechconf.com/gtc/2017/presentation/s7172-nikolai-smolyanskiy-autonomous-drone-navigation-with-deep-learning.pdf), [video](http://on-demand.gputechconf.com/gtc/2017/video/s7172-smolyanskiy-autonomous-drone-navigation-with-deep-learning%20(1).PNG.mp4)
* [Demo video showing 250 m autonomous flight, DNN activation and control](https://www.youtube.com/watch?v=H7Ym3DMSGms)
* [Demo video showing our record making 1 kilometer autonomous flight](https://www.youtube.com/watch?v=USYlt9t0lZY)
* [Demo video showing generalization to ground vehicle control and other environments](https://www.youtube.com/watch?v=ZKF5N8xUxfw)
* Stereo DNN, GTC18 talk: [arXiv paper](https://arxiv.org/abs/1803.09719), [Stereo DNN video demo](https://youtu.be/0FPQdVOYoAU)
