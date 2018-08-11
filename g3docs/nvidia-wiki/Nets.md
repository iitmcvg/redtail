# Neural network scripts

Our TrailNet DNN is based on the popular ResNet-18 architecture with a few modifications (refer to the [paper](https://arxiv.org/abs/1705.02550) for more details).
The `caffe_srelu_resnet.py` script in `ResNet` directory generates a `prototxt` file which is a description of the DNN architecture that is used at the first stage of training using the original Trails dataset. At the second stage of training, the network is modified and a second "head" is added which is then trained on the translation dataset. Only the last layer of this second head is trained; all other layers (convolutional etc) are frozen.
The `srelu-resnet-dragonnet.prototxt` file contains a network architecture for the second step.
During training, both networks use custom data augmentation techniques as well as a custom loss function. This functionality is implemented as Caffe Python layers in the `python-layers.py` file. This file needs to be specified in DIGITS when launching a job.