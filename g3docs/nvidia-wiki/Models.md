# TrailNet models
This document describes how to create TrailNet model from scratch using NVIDIA DIGITS.

## Creating a dataset
First, we need to create a dataset in DIGITS. Follow the steps in [this](./Datasets) document.

## Training a model
Once the dataset is created, the model can be trained and fine-tuned using DIGITS. Follow the steps in [this](./Training-TrailNet-model) document.

## Deploying a model
Once the model is trained, resulting `.prototxt` and `.caffemodel` files can be used by ROS nodes running on Jetson. The models can be used either in a [simulator environment](./Testing-in-Simulator) or on a [real drone](./Launch-Sequence-and-Flying).