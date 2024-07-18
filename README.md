<img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcQ1vHDQMUcbXRoh_hAcOvHvIXIQVk2dtlak3QBu-KU_PnGjMAwr6yHy9VdkSe04BuIF9_w&usqp=CAU" width=200>

[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)]()

[CEINMS-rt]() installation and [use]().
Plugin [installation]() and [compilation](). How to use a [plugin]().

# ROS Plugin for CEINMS-RT

Current version is using ROS1 (Noetic) on Ubuntu 20.04. 
The goal of this plugin is to offer a way to transmit information of CEINMS from and to ROS.

## Compile CEINMS with ROS

The compilation of the plugin is done in CMake like CEINMS but before compilation the following step is needed:

In the terminal run first:
``` bash
. /opt/ros/noetic/setup.sh
```

then
``` bash
cmake-gui
```
This will allow CEINMS to find all the ROS dependencies.

## Getting Started

Plugin name for the EMG:
``` xml
PluginEMGROS
```

For torque/position:
``` xml
PluginROS
```

The name of the node for the EMG plugin is "EMG_CEINMS" and for the Torque/position "nms_model"

The inputs/outputs are the following:
The EMG plugin only subscribe to a topic name "emg" and expect a vector with a size of the number of channel described in the model xml.
For the torque/position plugin, the plugin creates the following topics:

    joint_state (type sensor_msgs::JointState) the size of the vector is the same as the number of joint in the model;
    muscle_state (type sensor_msgs::JointState) the size of the vector is the same as the number of muscle in the model;
    muscle_state_active (type sensor_msgs::JointState) the size of the vector is the same as the number of muscle in the model;
    muscle_state_passive (type sensor_msgs::JointState) the size of the vector is the same as the number of muscle in the model;
    tendon_strain (type sensor_msgs::JointState) the size of the vector is the same as the number of muscle in the model;

And subscribe in the following topics:

    joint_position (type sensor_msgs::JointState) The size of the vector does not matter but it expect two joint ("jt_L5_S1_y" (translated to "L5_S1_Flex_Ext") and "jt_L5_S1_z"(translated to "L5_S1_axial_rotation")). This part to should be changed for your need see PluginROS.cpp line 50;
    joint_torque_ext (type sensor_msgs::JointState) The size of the vector does not matter since the ID torque is only use for visualization in the GUI. This part to should be changed for your need see PluginROS.cpp line 83;

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

## License

CEINMS-rt and plugins are licensed under the [Apache License](LICENSE).
