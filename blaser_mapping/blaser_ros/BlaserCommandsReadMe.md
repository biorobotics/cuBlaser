
# Blaser Commands Readme

## -------Quick start Method--------

	### Step 0: Plug in Blaser's USB3 and USB2 cable to NUC
	You will see the LED and Blue laser pulse once, then Blue laser turns on, then the sensor is ready!

	### Short-Step 1: Bring up Blaser sensor
	`roslaunch blaser_ros bring_up_blaser.launch` 

	### Short-Step 2: Start up Blaser SLAM backend and Mapping GUI
	`roslaunch blaser_ros trl6_slam.launch`

## -------Step-by-Step Method--------

	### Step 0: Plug in Blaser's USB3 and USB2 cable to NUC
	You will see the LED and Blue laser pulse once, then Blue laser turns on, then the sensor is ready!

	### Step 1: open roscore at terminal #1
	`roscore`

	### Step 2: Allow port access for Blaser low-level control
	`sudo chmod 666 /dev/ttyACM0`

	### Step 3: Turn on Blaser Camera device driver
	`rosrun blaser_pcl_core handheld_blaser_v30_node --lexp_val 50`

	### Step 4: Start up Blaser Machine Vision frontend
	`roslaunch blaser_ros trl6_sensor_proc.launch`

	### Step 5: Start up Blaser SLAM backend and Mapping GUI
	`roslaunch blaser_ros trl6_slam.launch`

## -------How to stitch point cloud--------

	### To use mapping GUI:
	### You need to change the "Mapping status" Slider/Parameter:
	###     - 0: Stop Map Generation
	###     - 1: Start Stitching Point Cloud Map
	###     - 2: Clear Map

	### Always start from 0(stop) stage, when Blaser initialized, then trun on 1(start) to stitch point cloud, and end with move back to 0 to stop.
	### you can also slide it to 2(Clear map).

## -------How to Save point cloud--------

	### Save pointcloud (colorized, accumulated)
	`rosrun pcl_ros pointcloud_to_pcd input:=/slam_estimator/laser_pcd`
	`rosrun pcl_ros pointcloud_to_pcd input:=/laser_detector/laser_points`
