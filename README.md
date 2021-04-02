# **Programming a Self-Driving Car in a Simulator **

The goals / steps of this project are to write ROS nodes to implement core functionality of the autonomous vehicle, including the followings:
* Traffic light detection
* Control
* Waypoint followings

[//]: # (Image References)

[system_architecture]: ./image/system_architecture.png "System Architecture"
[waypoint_loader_node]: ./image/waypoint_loader_node.png "Waypoint Loader Node"
[waypoint_follower_node]: ./image/waypoint_follower_node.png "Waypoint Follower Node"
[waypoint_updater_node]: ./image/waypoint_updater_node.png "Waypoint Updater Node"
[drive_by_wire_node]: ./image/drive_by_wire_node.png "Drive by Wire Node"
[traffic_light_detection_node]: ./image/traffic_light_detection_node.png "Traffic Light Detection Node"
[model_accuracy]: ./image/model_accuracy.png "Model Accuracy"
[model_loss]: ./image/model_loss.png "Model Loss"

## System Architecture

The following diagram illustrate the system architecture showing the ROS nodes and tpics in the project.

![System Architecture][system_architecture]

## Waypoint Loader Node

Waypoint loader node is resposible to loads the static waypoint data and publishes to `/base_waypoints`.

They are located in `./ros/src/waypoint_loader/`.

![Waypoint Loader Node][waypoint_loader_node]

## Waypoint Follower Node

Waypoint follwer node containing code from Autoware is resposible to subscribes to `/final_waypoints` and publishes target vehicle linear and angular velocities to the `/twist_cmd` topic.

They are located in `./ros/src/waypoint_follower/`.

![Waypoint Follower Node][waypoint_follower_node]

## Waypoint Updater Node

Waypoint updater node is responsible to compute the target velocity property of each waypoint based on the traffic light.
The node subscribes to the `/base_waypoints`, `/current_pose`, `/obstacle_waypoint`, and `traffic_waypoint` topics.
It publishes a list of waypoints, containing velocities, ahead of the car to the `/final_waypoints` topics.

They are located in `./ros/src/waypoint_updater/`.

![Waypoint Updater Node][waypoint_updater_node]

## Drive-by-wire (DBW) Node

![Drive by Wire Node][drive_by_wire_node]

Driver-by-wire node is responsible to control of the vehicle by computing throttle, steering, and brake.
It subscribes to `/current_velocity`, `/twist_cmd`, and `/vehicle/dbw_enabled`.
The throttle, steering, brake commands are computed and publish to `/vehicle/throttle_cmd`, `/vehicle/steering_cmd`, and `/vehicle/brake_cmd` topics.

They are located in `./ros/src/twist_controller/`.


## Traffic Light Detection Node

Traffic light detection node is responsible to publishes the locations to stop for red traffic lights to the `/traffic_waypoint` topic given the data from `/image_color`, `/current_pose`, and `/base_waypoints`.

They are located in `./ros/src/tl_detector/`.

![Traffic Light Detection Node][traffic_light_detection_node]

A traffic light classification model is built to classify if there is red traffic light ahead of the car.
The car would stop in front of the stop line.

### Classification

The traffic light classification model is built using convolution neutral network. Details are described below.

#### 1. Dataset Summary.

4000 images are gathered by running the car in the simulator. 

|Label                 |No of images |
|:---------------------|:-----------:|
|Green traffic light   |1000         |
|Yellow traffic light  |1000         |
|Red traffic light     |1000         |
|Without traffic light |1000         |

80% of the images are used for training.
20% of the images are used for validation.

#### 2. Model Architecture

The model consists of the following layers. The model and its training is implemented in `detection/training.py`.

|Layer             |Description                 |Output         |Parameter|
|:-----------------|:---------------------------|:-------------:|:-------:|
|Input             |RGB image                   |400 x 300 x 1  |         |
|Convolution 3 x 3 |1 x 1 stride, no padding    |398 x 298 x 16 |448      |
|RELU              |Relu Activation             |398 x 298 x 16 |0        |
|Max pooling 2 x 2 |2 x 2 stride                |199 x 149 x 16 |0        |
|Convolution 3 x 3 |1 x 1 stride, no padding    |197 x 147 x 16 |2320     |
|RELU              |Relu Activation             |197 x 147 x 16 |0        |
|Max pooling 2 x 2 |2 x 2 stride                |98 x 73 x 16   |0        |
|Convolution 3 x 3 |1 x 1 stride, no padding    |96 x 71 x 16   |2320     |
|RELU              |Relu Activation             |96 x 71 x 16   |0        |
|Max pooling 2 x 2 |2 x 2 stride                |48 x 35 x 16   |0        |
|Convolution 3 x 3 |1 x 1 stride, no padding    |46 x 33 x 32   |4640     |
|RELU              |Relu Activation             |46 x 33 x 32   |0        |
|Max pooling 2 x 2 |2 x 2 stride                |23 x 16 x 32   |0        |
|Convolution 3 x 3 |1 x 1 stride, no padding    |21 x 14 x 32   |9248     |
|RELU              |Relu Activation             |21 x 14 x 32   |0        |
|Max pooling 2 x 2 |2 x 2 stride                |10 x 7 x 32    |0        |
|Convolution 3 x 3 |1 x 1 stride, no padding    |8 x 5 x 32     |9248     |
|RELU              |Relu Activation             |8 x 5 x 32     |0        |
|Max pooling 2 x 2 |2 x 2 stride                |4 x 2 x 32     |0        |
|Flatten           |Flatten                     |256            |0        |
|Dense             |Dense network               |256            |32896    |
|Dense             |Dense network               |128            |516      |
|Softmax           |Softmax                     |4              |0        |

#### 3. Model Training

The model was trained using RMSprop with the following parameters.

|Item            |Value|
|:---------------|:---:|
|Batch size      |32   |
|Number of epochs|20   |
|Learning rate   |0.001|

The accuracy and the loss for training and validation are shown in the following figures. 

![Model Accuracy][model_accuracy]
![Model Loss][model_loss]

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

## Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

## Usage

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
