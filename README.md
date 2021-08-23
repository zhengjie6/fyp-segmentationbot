![image](resources/Cover.png)

## Introduction

This project make use of Semantic Segmentation to create a click to drive robot. The robot is implemented using only monocular cameras.

The segmentation of the project uses MMSegmentation, an open source semantic segmentation toolbox based on PyTorch.
It is a part of the OpenMMLab project. In addition, the distance algorithm of the robot makes used of homographic transformation algorithm and the robot control makes use of Robot Operating System (ROS).

The master branch works with **PyTorch 1.3+** and the ROS on the segmentation computer is **ROS Melodic**.

You may read the report of this project at [this link](https://drive.google.com/file/d/1lnmtmhn0mRjQqjQMD1tPpiQRdENOSLKn/view?usp=sharing).
![demo image](resources/seg_demo.gif)

## System Overview

This project has 3 main parts: Segmentation of Ground Images using the MMSegmentation Library, Estimation of Distance of a Point from a Robot and Robot Control Algorithm. The final product is click-to-drive robot that can drive to a particular location when the use selects a point.

### Segmentation of Ground Images
The key objective is to obtain a clear segmentation of ground level images using a deep learning based solution. For this task, creation and training of custom dataset is done to allow for the segmentation to be accurate. This is so that by using only monocular cameras, the autonomous robot can detect obstacles in its path of travel.

### Estimation of Distance of a Point from Robot
The key objective is to transform camera coordinate to real world coordinates


## Installation

Please refer to [get_started.md](docs/get_started.md#installation) for installation and dataset preparation.

## Get Started

Please see [train.md](docs/train.md) and [inference.md](docs/inference.md) for the basic usage of MMSegmentation.
There are also tutorials for [customizing dataset](docs/tutorials/customize_datasets.md), [designing data pipeline](docs/tutorials/data_pipeline.md), [customizing modules](docs/tutorials/customize_models.md), and [customizing runtime](docs/tutorials/customize_runtime.md).
We also provide many [training tricks](docs/tutorials/training_tricks.md).

A Colab tutorial is also provided. You may preview the notebook [here](demo/MMSegmentation_Tutorial.ipynb) or directly [run](https://colab.research.google.com/github/open-mmlab/mmsegmentation/blob/master/demo/MMSegmentation_Tutorial.ipynb) on Colab.

## Citation

If you find this project useful in your research, please consider cite:

```latex
@misc{mmseg2020,
    title={{MMSegmentation}: OpenMMLab Semantic Segmentation Toolbox and Benchmark},
    author={MMSegmentation Contributors},
    howpublished = {\url{https://github.com/open-mmlab/mmsegmentation}},
    year={2020}
}
```

## Acknowledgement

This project is make use of MMSegmentation created by MMSegmentation Contributors. 
The original project can be found at https://github.com/open-mmlab/mmsegmentation.
We would like to thank the original authors of MMSegmentation for creating this project.

In addition, the project is made possible my NP Final Year Project Supervisor and TSO Staff.
