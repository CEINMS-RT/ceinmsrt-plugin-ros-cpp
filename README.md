<img src="https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/CEINMS-RT_V2_ICON.png" width="50%" alt="CEINMS-RT logo">

[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)]()

CEINMS-RT [installation](https://ceinms-docs.readthedocs.io/en/latest/Installation%20%5BWindows%5D.html) and [use](https://ceinms-docs.readthedocs.io/en/latest/Tutorial%20%5BWindows%5D%5BUbuntu%5D.html).
Plugin [installation](#installation) and [compilation](https://ceinms-docs.readthedocs.io/en/latest/Compilation%20%5BWindows%5D.html). How to use a [plugin](#getting-started).

# ROS Plugin

The goal of this plugin is to offer a way to transmit information of CEINMS-RT from and to ROS.
Current version is using ROS1 (Noetic) on Ubuntu 20.04.

## Installation

Before compilation the following step is needed:

``` bash
. /opt/ros/noetic/setup.sh
```

This will allow CEINMS-RT to find all the ROS dependencies.

## Getting Started

Plugin name for the EMG:

``` xml
PluginEMGROS
```

For torque/position:

``` xml
PluginROS
```

The name of the node for the EMG plugin is "EMG_CEINMS" and for the Torque/position "nms_model".

The inputs/outputs are the following:
The EMG plugin only subscribes to a topic name "emg" and expects a vector with a size of the number of channels described in the model xml. \
For the torque/position plugin, the plugin creates the following topics:

* joint_state (type sensor_msgs::JointState) \
The size of the vector is the same as the number of joints in the model;
* muscle_state (type sensor_msgs::JointState) \
The size of the vector is the same as the number of muscles in the model;
* muscle_state_active (type sensor_msgs::JointState) \
The size of the vector is the same as the number of muscles in the model;
* muscle_state_passive (type sensor_msgs::JointState) \
The size of the vector is the same as the number of muscles in the model;
* tendon_strain (type sensor_msgs::JointState) \
The size of the vector is the same as the number of muscles in the model;

And subscribe to the following topics:

* joint_position (type sensor_msgs::JointState) \
The size of the vector does not matter, but it expect two joints: "jt_L5_S1_y" (translated to "L5_S1_Flex_Ext") and "jt_L5_S1_z" (translated to "L5_S1_axial_rotation").\
To change it to your needs [see](src/PluginROS.cpp#L45);
* joint_torque_ext (type sensor_msgs::JointState)\
The size of the vector does not matter since the ID torque is only used for visualization in the GUI.\
To change it to your needs [see](src/PluginROS.cpp#L83);

## Citation

If you find this repository useful in your research, please consider giving a star ⭐ and cite our [IEEE TBME paper](https://spiral.imperial.ac.uk/bitstream/10044/1/48309/2/durandau%202017.pdf) by using the following BibTeX entrys.

```BibTeX
@article{durandau2017robust,
  title={Robust real-time musculoskeletal modeling driven by electromyograms},
  author={Durandau, Guillaume and Farina, Dario and Sartori, Massimo},
  journal={IEEE transactions on biomedical engineering},
  volume={65},
  number={3},
  pages={556--564},
  year={2017},
  publisher={IEEE}
}
```
