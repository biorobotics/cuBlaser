# BLASER_ROS

This ROS package contains software for hand-held Blaser, including

* Laser plane calibration
* Laser points detection and triangulation
* Blaser resolution (sensitivity) analyzation
* Laser point cloud coloring (RGB value estimation), which is also implemented
  in the SLAM package (`blaser_slam`)

The library `camera_model` (forked from **camodocal**) is used extensively in this
package.

This tutorial contains three sections

0. Installing dependencies and Blaser software
1. Running SLAM (involves calibration & running triangulation)
2. Performing sensitivity evaluation

## 0. Installation
This section is about how to set up Blaser software on a new computer.

### 0.0 Environment
Blaser uses Ubuntu 18.04 and ROS Melodic ([Installation](http://wiki.ros.org/melodic/Installation/Ubuntu)).

### 0.1 Install Dependencies

1. First make a directory for all Blaser dependencies
```shell
mkdir ~/blaser_dependencies
```

2. Install [Matio](https://github.com/tbeu/matio)
```shell
cd ~/blaser_dependencies
git clone git://git.code.sf.net/p/matio/matio
cd matio
./autogen.sh
./configure
make
sudo make install
```
3. Install [Ceres Solver](http://ceres-solver.org/installation.html)
```shell
cd ~/blaser_dependencies
wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
sudo apt-get install libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev
tar zxf ceres-solver-1.14.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-1.14.0
make -j8
sudo make install
```
4. Install [XIMEA Linux Software packge](https://www.ximea.com/support/wiki/apis/XIMEA_Linux_Software_Package)
```shell
cd ~/blaser_dependencies
wget https://www.ximea.com/downloads/recent/XIMEA_Linux_SP.tgz
tar xzf XIMEA_Linux_SP.tgz
mv package ximea_linux_sp
cd ximea_linux_sp
sudo ./install
```
5. Python libraries
```sh
sudo apt-get install python-matplotlib python-numpy
```
6. Enable high speed USB and USB permission
```shell
sudo adduser $USER dialout
# if file /etc/rc.local already exists
echo 'echo 0 > /sys/module/usbcore/parameters/usbfs_memory_mb' | sudo tee -a /etc/rc.local
# if file /etc/rc.local does not exist 
printf '%s\n' '#!/bin/bash' 'echo 0 > /sys/module/usbcore/parameters/usbfs_memory_mb' | sudo tee /etc/rc.local; sudo chmod +x /etc/rc.local
# reboot for the changes to take effect
```

### 0.2 Install Blaser software
1. Create a catkin workspace for blaser:
```sh
mkdir -p ~/blaser_ws/src && cd ~/blaser_ws/src
```
2. Then clone the repo `blaser_mapping`:
```sh
git clone https://github.com/biorobotics/blaser_mapping.git
```
3. Clone the Blaser driver called `blaser-pcl-core` located in another Git repo.
```sh
git clone https://github.com/biorobotics/blaser-pcl-core.git
```
4. build
```shell
cd ~/blaser_ws/
catkin_make -j1 
```
The compilation will take some time. Please wait. 

Once the compilation is done, you would be required to add a line to `~/.bashrc` file. 
Open the the bashrc file by running the following command in a terminal.

`gedit ~/.bashrc`

This will open the text editor and add the following line at the end of the file. 

 ```
 source ~/blaser_ws/devel/setup.bash
 ```
 
 Save and close the text editor and run `source ~/.bashrc` for the changes to take effect. 
 
 You can test your installation by running 
 
 ```
 roscd blaser_ros
 ```
 Upon running the above commnand, your current directory should change to  - `~/blaser_ws` .
 
## Run Blaser
Now you can run Blaser following [this tutorial](https://github.com/biorobotics/blaser_mapping/blob/master/blaser_ros/README.md).\
If the Blaser is already calibrated, you can skip Step 1.2 Sensor Calibration in the linked tutorial.

## 1. Running SLAM

Check out our [video tutorial](https://drive.google.com/file/d/15whcikrRbo9H2MsuE5pYbJ87K-Ah5hBY/view).

To run SLAM, there are three steps involved. Firstly, you should run the sensor
driver for your specific Blaser prototype and acquire raw sensor topics. Next,
calibrate the camera intrinsics, camera-IMU extrinsics, and camera-laser extrinsics and load the 
calibration results into a yaml file. Finally,
execute SLAM by running both the SLAM front-end and the back-end.

### 1.1 Running sensor driver

Run the driver for your specific Blaser prototype.

For the Blaser 3.0 (sent to Boeing version), see [`blaser-pcl-core`](https://github.com/biorobotics/blaser-pcl-core).

Make sure you have three ROS topics:

1. Visual frames (sensor_msgs/Image, image stream observing the environment with
   the laser turned off, at least 20 hz)
2. Profiling frames (sensor_msgs/Image, image stream observing the laser stripe,
   at least 20 hz)
3. IMU data (sensor_msgs/Imu, at least 100 hz)

Visualize these topics (use rqt_image_view and rqt_plot) to check if they are
valid. Adjust camera exposure, LED brightness, and laser brightness to make sure
that 

- Visual frames are neutrally exposed.
- Profiling frames are under-exposed (dark) while the laser stripe is clearly
visible but not over-exposed (RGB channel values should not reach 255). If laser
is over-exposed, shorten camera exposure time.
- The clock of the camera and the IMU are synced.

Generally, short camera exposure time is preferred to reduce motion blur.
Therefore, it is good practice to use maximum LED and laser brightness for
SLAM. 

### 1.2 Sensor calibration

There are three calibration tasks: camera intrinsics, camera-IMU extrinsics, and
camera-laser extrinsics.

prepare a folder for this calibration trial and run some basic commands:

Say you want to name your trial folder TRIAL_FOLDER (e.g. boeing-demo)
```
$    roscd blaser_ros; mkdir -p calib_data/TRIAL_FOLDER; cd calib_data/TRIAL_FOLDER

$    mkdir intrinsics; mkdir laser_plane; scp ~/catkin_ws/blaser_ws/src/blaser_ros/config/laser_calib.yaml .   (don't forget the "." at the end!!)

$    scp -r ~/catkin_ws/blaser_ws/src/blaser_ros/config/kalibr_data .   (don't forget the "." at the end!!)

$    scp ~/catkin_ws/blaser_ws/src/blaser_ros/config/trl6_slam.yaml .; mv trl6_slam.yaml slam.yaml
```

These commands create a calibration folder and copy sample yaml files, which will
be loaded with your own calibration data later.

#### Camera intrinsics

**First collect images observing a checkerboard from various angles.**
It is a good idea to use ROS tool **camera_calibration** to collect images.

1. Run the Blaser driver
2. `cd ~/catkin_ws/blaser_ws/src/blaser_ros/calib_data/TRIAL_FOLDER/intrinsics`
3. Run the ros calibration tool  
`rosrun camera_calibration cameracalibrator.py --size 10x7 --square 0.005 image:=/image_topic`
4. Follow its instructions to collect images to satisfy all its score bars
5. Click calibrate, and wait until it finishes
6. Click save to save the images and the calibration results as a tarball. The
   terminal will say the tarball's location, usually in `/tmp/`.
7. Move the tarball to our data folder and extract the tarball  
`mv /tmp/calibrationdata.tar.gz .; tar xvf calibrationdata.tar.gz; rm calibrationdata.tar.gz; rm ost.*`

**Secondly, use package `camera_model` to perform calibration.**
`rosrun camera_model Calibration -w 7 -h 10 -s 5 -i ./ -v --camera-model mei`

The camera model parameters will be saved to `camera_camera_calib.yaml`.

Use help (`rosrun camera_model Calibration --help`) to see all the parameters.

Be careful about what camera model you use. For a narrow FoV camera without huge
distortion, use `pinhole` model should suffice. If you use a large FoV camera
with a lot of distortion, use `mei` or `kannala-brandt`. Typically you can try
all the camera models and choose the one with the smallest reprojection error.

**Finally, examine the intrinsics parameters and result**

1. Is the reprojection error reasonable? (should be at least < 1.0 px)
2. Is fx and fy similar?
3. Is cx and cy reasonable? cx ~ width / 2, cy ~ height / 2
4. Is image height and width correct?

You can also generate undistorted images. All the straight lines in real world
should appear as perfectly straight on the image.

#### Camera-IMU extrinsics

Now we have to calibrate the Euclidean transformation between the IMU frame and
the camera frame.

We use **Kalibr** to perform this calibration. It is a powerful calibration tool
that can calibrate camera intrinsics, camera-IMU extrinsics, multi-camera
extrinsics, etc. Follow its instructions for installation and usage.

A few tips:

* Use a different catkin workspace for **Kalibr**, since it uses `catkin build`.
* Since extrinsics calibration requires camera instrinsics, you may need to redo
  camera intrinsics calibration using Kalibr, if the camera model used in
  `camera_model` is not compatibale with Kalibr.
* You need IMU parameters (random walk and noise for accelerometer and
  gyroscope)
  for both extrinsics calibration and SLAM. `Kalibr`'s wiki has a tutorial on
  how to determine these four parameters. You can also use `imu_utils` package
  on Github to calibrate them.
* **Kalibr**'s wiki has some great tutorials on IMU modeling.

Trouble shooting for installing Kalibr:
1. Issue 1: Cannot find package when enter `$sudo pip install python-igraph --upgrade`  
Solution: Install older version of python-igraph. Still may need to `apt install bison byacc flex`.
2. Issue 2: [Cannot reach TAMU server when building Kalibr](https://github.com/h2r/kuka_brown/issues/1)  
Solution: Modify CMakeList file in SuiteSparse package and add --no-check-certificate between wget and TAMU URL

Use the following steps to calibrate
```
$    cd ~/catkin_ws/blaser_ws/src/blaser_ros/calib_data/TRIAL_FOLDER/intrinsics

(We recalibrate the camera intrinsics using the pinhole model. However, we only use it for imu-camera extrinsics calibration, NOT in SLAM!)
$    rosrun camera_model Calibration -w 10 -h 7 -s 5 -i ./ -v --camera-model pinhole

$    cd ~/catkin_ws/blaser_ws/src/blaser_ros/calib_data/TRIAL_FOLDER/kalibr_data; mkdir kalibr_results

Then update the camchain.yaml file with the new intrinsics from ~/catkin_ws/blaser_ws/src/blaser_ros/calib_data/TRIAL_FOLDER/intrinsics/camera_camera_calib.yaml

Then collect a bag file using aprilgrid of either 8.8mm or 5.5mm grids:

$    rosbag record --duration=100 -O data.bag /blaser_camera/image_hexp /imu

$    cd kalibr_results; activate_kalibr

$    rosrun kalibr kalibr_calibrate_imu_camera --bag ../data.bag --cam ../camchain.yaml --imu ../imu.yaml --target ../april_8mm.yaml
```

The calibration may take a long time. After it finishes, examine the rotation
and translation result. You should have a pretty good guess from the mechanical
design CAD file or by just looking at the sensor.

#### Camera-laser extrinsics

The extrinsics parameters between camera and laser is the 3D position of the
laser plane in the camera reference frame, defined as $ax + by + cz + d = 0$. In
order to determine the plane's position, we take sample points from this plane
and then try to fit a 3D plane to these 3D points. We obtain sample points from
images of a checkerboard where the laser stripe is projected onto the
checkerboard. Since the checkerboard defines a 3D plane, we can get a 3D point
position for each 2D laser point on the image, which is the intersection between
the checkerboard plane and the line-of-sight ray.

We first need to make sure that the laser stripe detection is working. The laser
stripe detector basically performs an HSV filter with five parameters hue_min,
hue_max, sat_min, val_min, and val_ratio, defining a 3D range filter H in \[hue_low,
hue_high\], S in \[sat_low, 255\], and V in 
\[max(val_low, image_max_val * val_ratio), 255\]. Note that V range is dynamic 
and is set with every image.

When using red laser and you want a hue range containing the hue=180 value,
set hue_min > hue_max and the program will generate two separate hue ranges: 
one is \[hue_min, 180\] and the other is \[0, hue_max\]. 

To set these parameters, first use `python scripts/im_saver.py [image_topic]` to
save a couple of sample images, then use `python scripts/im_hsv_viewer.py
[image_file]` to determine the HSV value of the laser stripe and set appropriate
values for the threshold parameters. Load these values in a config file (todo
give example config and dir), which will be used in the calibration later.

To test these parameters, run `laser_stripe_detector` node with sample images
and the config file. (todo turn on lsd visualization).

Perform laser calibration using the following steps:

$    cd ~/catkin_ws/blaser_ws/src/blaser_ros/calib_data/TRIAL_FOLDER; cd laser_plane      (Stay in this folder throughout Step 2!)

$    rosrun blaser_ros im_saver.py /blaser_camera/image_lexp   

Then press 's' to save the current image. Use a [foot switch](https://www.amazon.com/s?k=foot+switch+usb) if it helps.
Collect 10~15 images with varying checkerboard depth. 
Avoid collision between laser line and checkerboard corners.

Then update the laser_calib.yaml (copy from blaser_ros/config/) file with the new camera intrinsics you got in Step 1.
The intrinsics yaml file was generated in ~/catkin_ws/blaser_ws/src/blaser_ros/calib_data/TRIAL_FOLDER/intrinsics/camera_camera_calib.yaml

$    rosrun blaser_ros laser_calib -i ./ -c laser_calib.yaml   (Press 'y' to accept, 'n' to reject current image) 

$    python ../../../scripts/vis_laser_calib_result.py calib_results.txt

#### Finally, load your calibrated parameters to the SLAM yaml file.

1. The big yaml file is: ~/catkin_ws/blaser_ws/src/blaser_ros/calib_data/TRIAL_FOLDER/trl6_slam.yaml
2. Camera intrinsics result is in: ~/catkin_ws/blaser_ws/src/blaser_ros/calib_data/TRIAL_FOLDER/laser_plane/laser_calib.yaml
3. Laser-camera extrinsics is in : ~/catkin_ws/blaser_ws/src/blaser_ros/calib_data/TRIAL_FOLDER/laser_plane/calib_results.txt
4. Camera-imu extrinsics is in   : cd ~/catkin_ws/blaser_ws/src/blaser_ros/calib_data/TRIAL_FOLDER/kalibr_data/kalibr_results/results-imucam-..data.txt (Please copy the T_ic!)

After updating the yaml file, here's the final command before actually running slam!
$    cd ~/catkin_ws/blaser_ws/src/blaser_ros/config; rm trl6_slam.yaml; scp ~/catkin_ws/blaser_ws/src/blaser_ros/calib_data/TRIAL_FOLDER/trl6_slam.yaml . (Don't forget the dot!!)

If your SLAM yaml file has a different name, simply update the name in `trl6_sensor_proc.launch` and `trl6_slam.launch`.

### 1.3 Run SLAM
1. Device driver  
`rosrun blaser_pcl_core handheld_blaser_v30_node --lexp_val 50`
2. SLAM frontend  
`roslaunch blaser_ros trl6_sensor_proc.launch`
3. SLAM backend  
`roslaunch blaser_ros trl6_slam.launch`
4. Use `rqt_reconfigure` to control the mapping process (start / stop / clear) mapping. Mapping status:
    - 0: Stop
    - 1: Start
    - 2: Clear  
`rosrun rqt_reconfigure rqt_reconfigure`
5. Save pointcloud (colorized, accumulated)  
For 3D map: `rosrun pcl_ros pointcloud_to_pcd input:=/slam_estimator/laser_pcd`
For raw laser scans `rosrun pcl_ros pointcloud_to_pcd input:=/laser_detector/laser_points`

## 2. Perform sensitivity analysis
The sensitivity of the Blaser sensor is defined as the shift of the laser stripe
on the image in response to a 1 mm depth change. It as a unit of pixel/mm. The
higher the sensitivity, the higher the 3D measurement precision. Assuming that
the laser detection on the image has an absolute error upper bound of 0.5 pixels,
then the sensor's 3D measurement precision is 0.5 pixel / sensitivity. E.g., if
the sensitivity is 5 pixel/mm, then the precision is 0.1 mm.

The sensitivity analysis tool can be used to theoretically analyze the sensor's
precision, but also to determine the best sensor design in terms of laser leaning
angle (angle between the laser plane and the camera optical axis, 0 being parallel).

To perform the sensitivity analysis, run  
`rosrun blaser_ros resolution_analyser <sensitivty_analysis_yaml_file>`.

A sample sensitivty_analysis_yaml_file can be found at 
`blaser_ros/config/resolution_analyser/resolution_analyser.yaml`.

## Using ROS Bag with Blaser

If you want to record sensor data and run SLAM with recorded data offline, use
compressed image topics instead of raw image topics. This can save you GBs of
storage space. Here's what I typically do:

1. Record: record raw sensor data only, including raw imu data and compressed
image topics. The compressed image topics are already published by the driver.  
`rosbag record /imu /blaser_camera/image_hexp/compressed /blaser_camera/image_lexp/compressed`
2. Replay: since the image topics are compressed, you need to *republish* the
image topics. This is implemented in the `sensor_proc.launch` file using the
`republish_compressed_image` argument, whose default value is 0. Use the
following command when using rosbags.  
`roslaunch blaser_ros xx_sensor_proc.launch republish_compressed_image:=1`
