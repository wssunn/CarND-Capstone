# Udacity Self-Driving Car Nanodegree Capstone: Programming a Real Self-Driving Car
In this project, core functions of an autonomous vehicle are implemented using ROS. Functions include traffic light detection, control and waypoint following. 

## Implementation

### Core steps
1. Waypoint Updater Node (Partial): Complete a partial waypoint updater which subscribes to `/base_waypoints` and `/current_pose` and publishes to `/final_waypoints`.
2. DBW Node: Once waypoint updater is publishing `/final_waypoints`, the waypoint_follower node will start publishing messages to the `/twist_cmd` topic. After completing this step, the car should drive in the simulator, ignoring the traffic lights.
3. Traffic Light Detection: This can be split into 2 parts:
	* Detection: Detect the traffic light and its color from the `/image_color`. The topic `/vehicle/traffic_lights` contains the exact location and status of all traffic lights in simulator, use it to test the output.
	* Classification: Train a deep learning classifier to classify the entire image as containing either a red light, yellow light, green light, or no light.
	* Waypoint publishing: Once correctly identified the traffic light and determined its position, convert it to a waypoint index and publish it.
4. Waypoint Updater (Full): Use `/traffic_waypoint` to change the waypoint target velocities before publishing to `/final_waypoints`. Your car should now stop at red traffic lights and move when they are green.

[//]: # "Image References"

[image1]: ./imgs/ros_graph.png
[image2]: ./imgs/tl-detector-ros-graph.png
[image3]: ./imgs/waypoint-updater-ros-graph.png
[image4]: ./imgs/dbw-node-ros-graph.png


### System Architecture

The following is a system architecture diagram showing the ROS nodes and topics used in the project.

![alt text][image1]

### Code Structure

Find the ROS packages for this project under the `/ros/src/` directory.

#### `/ros/src/tl_detector/`

This package contains two nodes:

1. `tl_detector.py`: traffic light detection node
2. `light_classification_model/tl_classfier.py`: traffic light classification node

The purpose is to publish locations to publish locations to stop for red traffic lights.

![alt text][image2]

#### `/ros/src/waypoint_updater/`

This package contains the waypoint updater node: `waypoint_updater.py`. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. 

![alt text][image3]

#### `/ros/src/twist_controller/`

This package contains the drive-by-wire (dbw) node that responsible for control the vehicle. Files worth noting:

1. `dbw_node.py`
2. `twist_controller.py`
3. `pid.py`
3. `lowpass.py`

![alt text][image4]

#### `/ros/src/styx/`
A package that contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.

#### `/ros/src/styx_msgs/`
A package which includes definitions of the custom ROS message types used in the project.

#### `/ros/src/waypoint_loader/`
A package which loads the static waypoint data and publishes to `/base_waypoints`.

#### `/ros/src/waypoint_follower/`
A package containing code from [Autoware](https://github.com/CPFL/Autoware) which subscribes to `/final_waypoints` and publishes target vehicle linear and angular velocities in the form of twist commands to the `/twist_cmd` topic.

## Installation

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

