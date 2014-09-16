libvisensor
=============

Sensor driver library to the Visual-Inertial Sensor developed by the [Autonomous Systems Lab (ASL), ETH Zurich](http://www.asl.ethz.ch) and [Skybotix AG](http://www.skybotix.com). This sensor provides fully time-synchronized and factory calibrated IMU- and stereo-camera datastreams. A detailed spec sheet of the sensor can be found [here](http://www.skybotix.com/skybotix-wordpress/wp-content/uploads/2014/03/VISensor_Factsheet_web.pdf). The ROS frontend of the VI-Sensor library can be found [here](https://github.com/ethz-asl/visensor_node).

![alt text](http://wiki.ros.org/vi_sensor?action=AttachFile&do=get&target=vi-sensor-front.jpg "Sensor Photo")

A detailed description on how the sensor can be used in ROS is found in the corresponding [ROS-Wiki](http://wiki.ros.org/vi_sensor).


## Installation Instructions ROS

Check out the sensor library and this node to your catkin workspace:

```
cd your_catkin_workspace
git clone https://github.com/ethz-asl/libvisensor.git
```

Make sure that you installed all necessary Ubuntu packages

```
sudo apt-get install libeigen3-dev libboost-dev
```

Build the package using catkin_make

```
catkin_make
```

 
## Installation Instructions Stand-alone (No ROS)
Check out the sensor library
```
git clone https://github.com/ethz-asl/libvisensor.git
```
Make sure that you installed all necessary Ubuntu packages

```
sudo apt-get install libeigen3-dev libboost-dev
```

Build the package and install it system-wide.

```
cd libvisensor
./install_libvisensor
```

In case you don't want to install the library system-wide, you can also simply build it using
```
cd libvisensor
mkdir build
cd build
cmake ..
make
```
In this case, you have to ensure that your applications can find the library files and headers.

To see sample (stand-alone) projects, please refer to https://github.com/skybotix/visensor_sample_applications 

This repo contains a few sample standalone applications for the VI-Sensor

* **vi_sensor_interface**: A very basic interface to the VI-Sensor. Receives images and IMU messages and displays it using openCV.
* **vi_sensor_stereo_block_matcher**: Applies rectification to Vi-Sensor images and subsequently computes the disparity image using standard openCV block matcher. Intrinsic & extrinsic calibration parameters for rectification are downloaded from VI-Sensor.
