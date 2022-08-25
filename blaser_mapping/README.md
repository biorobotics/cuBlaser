# Blaser Mapping

This package contains the mapping software package developed for confined space.

In this work, a unified sensor framework based on laser profiling is proposed.
The propsed framework contains a hardware structure, a software pipeline, and a
Simultaneous Localization and Mapping (SLAM) method. The hardware consists of a
camera, a laser projector, an inertial measurement unit (IMU), and other
optional sensors. Software for mapping as well as for calibration and
sensitivity analysis are developed. The SLAM software can generate
photorealistic 3D point cloud maps of scanned scenes.

Two sensor prototypes are created using this unified sensor framework for two
different confined space applications. The Blaser prototype is for general
confined spaces and is designed to be as compact as possible. The PipeBlaser
sensor is created for in-pipe mapping for 12-inch diameter natural gas pipes.

This Git repository is organized as follows.

0. Installation ReadMe guidebook: [blaser_mapping/blaser_ros](https://github.com/biorobotics/blaser_mapping/blob/master/blaser_ros/README.md).

1. `blaser_slam` contains the SLAM software. It has a `feature_tracker` as
   visual front-end and a `slam_estimator` for factor graph backend. This SLAM
   software is a stand-alone tool and do not require any modification for
   different sensors.
2. `blaser_ros` contains the software for the Blaser prototype. It contains
   software including laser detection, laser triangulation, sensitivity
   analysis, laser plane calibration, etc. It also accommodates all
   sensor-specific data, configuration files, and launch files. To use the
   software package with a new sensor prototype, you mainly need to work with
   the `blaser_ros` folder. Refer to the `README.md` file in `blaser_ros` folder
   for a tutorial for calibrating and running SLAM with Blaser. To use Blaser
   3.0, you also need the driver package
   [blaser-pcl-core](https://github.com/biorobotics/blaser-pcl-core).
3. `pipe_blaser_ros` has similar structure and functionalities to `blaser_ros`,
   but it is for PipeBlaser prototypes.

For a detailed description on this work, please refer to Daqian Cheng's 
[master's thesis](https://www.ri.cmu.edu/publications/slam-with-laser-profilers-for-high-definition-mapping-in-confined-spaces/).

Papers:
```
@inproceedings{2020blaser_sensors,
  title={A Compact and Infrastructure-free Confined Space Sensor for 3D Scanning and SLAM},
  author={Cheng, Daqian and Shi, Haowen and Schwerin, Michael and Li, Lu and Choset, Howie},
  booktitle={2020 IEEE SENSORS},
  year={2020},
  organization={IEEE}
}
```

```
@inproceedings{2021blaser_icra,
  title={visual-laser-inertial slam using a compact 3d scanner for confined space},
  author={Cheng, Daqian and Shi, Haowen and Xu, Albert and Schwerin, Michael and Crivella, Michelle and Li, Lu and Choset, Howie},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={},
  year={2021},
  organization={IEEE}
}
```
