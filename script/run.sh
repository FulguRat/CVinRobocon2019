#!/bin/bash

cd ../bin

/home/nvidia/code/robocon/CVinRobocon2019/bin/Test -m_red=/home/nvidia/code/tensorRT/mibileNet_SSD_TensorRT_Robocon2019/model/redblue/MobileNetSSD_deploy -m_blue=/home/nvidia/code/tensorRT/mibileNet_SSD_TensorRT_Robocon2019/model/blue/MobileNetSSD_deploy -com=/dev/ttyTHS2 -m_alex=/home/nvidia/code/tensorRT/mibileNet_SSD_TensorRT_Robocon2019/model/alexnet/bvlc_alexnet -m_mean=/home/nvidia/code/tensorRT/mibileNet_SSD_TensorRT_Robocon2019/model/alexnet/mean/imagenet_mean.binaryproto
