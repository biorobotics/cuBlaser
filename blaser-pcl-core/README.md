# blaser-pcl-core
Linux reference implementation of Blaser core computer vision and depth
recovery algorithms.

Some core functionalities are written with portability in mind but
it is not realistic to cross compile to embedded targets. It is recommended
that the embedded implementation follow the similar structure to make it
easier to receive updates from this reference implementation.

## Dependencies
- OpenCV 3.0.0+
- Boost
- PCL
- OpenMP

## Optional Dependencies
- Python 2.7
- Matplotlib
- NumPy
- XIMEA

Python 2.7 and Matplotlib are needed to enable various visualizations.
```sh
sudo apt-get install python-matplotlib python-numpy python2.7-dev
```

To enable optional XIMEA camera support, follow driver installation
instructions here:

https://www.ximea.com/support/wiki/apis/XIMEA_Linux_Software_Package

## Getting Started
This repository can be built as a ROS package. Put this package under
your_ros_workspace/src and build using `catkin build blaser_pcl_core`.
```sh
cd <your_ros_workspace>/src
git clone https://github.com/biorobotics/blaser-pcl-core
catkin build blaser_pcl_core
```

## Driver for TRL-6 Blaser
First make sure you are not using simulated ROS time:
```sh
rosparam set use_sim_time true
```
To enable high transmission speed of USB3.0, use the following command. You
will need to do this every time upon rebooting your Linux computer.
```sh
sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb > /dev/null <<< 0
```
Everytime you plug in the microcontroller USB, make sure your user can access
the mounted device. You will need to do this every time you reconnect the
microcontroller. Alternatively, assign your user to the proper device group
for device read/write privileges.
```sh
sudo chmod 666 /dev/ttyACM0
```
Make sure XIMEA camera is connected to a dedicated USB3.0 port without a hub,
and start the handheld Blaser driver node:
```sh
rosrun blaser_pcl_core handheld_blaser_v30_node
```

See more command line options by using `--help`:
```sh
rosrun blaser_pcl_core handheld_blaser_v30_node --help

Allowed options:
  --help                 Produce help message
  --device_test          Test basic device functionalitites and quit
  --camera_only          Do not connect to embedded controller
  --disable_alt_shutter  Disable alternating shutter speed
  --serial_port arg      Blaser device serial port
  --lexp_val arg         Low exposure time (us)
  --hexp_val arg         High exposure time (us)
  --laser_brightness arg Laser brightness percentage
  --gain arg             Camera gain factor
```

Then, start the computer vision and point cloud generation pipeline:
```sh
rosrun blaser_pcl_core pcl_gen_node
```

See more command line options by using `--help`:
```sh
rosrun blaser_pcl_core pcl_gen_node --help

Allowed options:
  --help                  produce help message
  --laser_width arg       Width of laser stripe
  --laser_color arg       Color of laser stripe
  --extraction_method arg Extraction method
  --input_source_type arg Input source type
  --input_name arg        Option for selected input source
  --output_topic arg      Output point cloud topic
  --frame_id arg          Point cloud frame name
  --calib_filename arg    Calibration file path
```
For Blue laser Blaser, the recommended options are:
```--laser_color BLUE --laser_width 8```

To calibrate the TRL-6 Blaser, you can use the following setting and record
/blaser_camera/image_hexp images for calibration dataset:
```sh
rosrun blaser_pcl_core handheld_blaser_v30_node --disable_alt_shutter --hexp_val 5000 --laser_brightness 10
rosrun image_view image_view image:=/blaser_camera/image_hexp
```

## Computer Vision & Depth Recovery
```sh
rosrun blaser_pcl_core pcl_gen_node --laser_color BLUE --laser_width 8 --calib_filename /path_to_your/calibration.yaml
```

GGM Method:
```sh
rosrun blaser_pcl_core pcl_gen_node --laser_color BLUE --laser_width 40 --calib_filename /home/haowensh/Projects/blaser/blaser_ws/src/blaser-pcl-core/src/pcl_gen/calibration_TRL6.yaml --extraction_method GGM
```


## Plugins
Input video capture source and output type are implemented as plugins which
increases modularity.

Example input plugins:
- Read from file
- V4L RGB565 special handlingAllowed options:
  --help                  produce help message
  --laser_width arg       Width of laser stripe
  --laser_color arg       Color of laser stripe
  --extraction_method arg Extraction method
  --input_source_type arg Input source type
  --input_name arg        Input image topic
- OpenCV Capture Device (using libv4l)
- ROS image topic

Example output plugins:
- Single line PCL generation
- Multi line PCL generation
- Object classification result
