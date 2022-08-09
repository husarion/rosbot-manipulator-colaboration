---
sidebar_label: 5. Rosbot manipulator colaboration
id: rosbot-manipulator-colaboration
title: 5. Rosbot manipulator colaboration
---

## Abstract
In most of industrial cases, application of mobile and manipulation robots significantly speeds up and improves a given process.
Sometimes a combination of the advantages of transport robots and robotic arms is needed to acheive a given goal.
This project, based on ROS2, shows an example of cooperation between the ROSbot 2.0 PRO mobile robot and a stationary stand copmosed of the OpenMANIPULATOR-X robotic arm and the Intel Realsense D435 depth camera. The software layer cosists of 3 Python nodes (`/grabber_from_image_cords`, `tracker` and `rosbot_control`), which are discribed in [ROS Description](https://husarion.com/tutorials/ros-projects/rosbot-manipulator-colaboration/#ros-description) section. 

![rosbot_with_plate](rosbot_with_plate.jpg)

## Description
As said in the abstract above, this project is an application based on the cooperation of [ROSbot 2.0 PRO](https://store.husarion.com/products/rosbot-pro), [OpenMANIPULATOR-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/) and [Intel Realsense D435](https://www.intelrealsense.com/depth-camera-d435/).
ROSbot with custom 3D printed [plate](https://github.com/husarion/rosbot-manipulator-colaboration/tree/master/CAD) transports [yellow cylinders](https://github.com/husarion/rosbot-manipulator-colaboration/tree/master/CAD) (more less with a diameter of 35 mm), which can be also 3D printed, from starting point to the manipulator's area. Above the robotic arm there is a Realsense depth camera which detects the cartesian position of objects and their height. The manipulator picks up the yellow elements one by one putting them on top of each other in a safe place called storage. Then, the empty ROSbot returns to its starting point and waits for new objects to be reloaded in order to start new sequence.

The final effect of this project can be seen in the video below.

#### Project Video

<div align="center">
<iframe width="???" height="???" src="https://www.youtube.com/???" frameborder="0" gesture="media" allowfullscreen></iframe>
</div>

### Features and capabilities
* Whole system runs with Docker Compose 
* Laptop, manipualtor and camera communicate with the ROSbot via wifi network
* Application launches Rviz2 software, which shows the visualization of the ROSbot's movement in the room
* Rviz2 also shows images from the camera: color image, depth image and markers on detected objects
* The color of detected objects depends on the HSV ranges set in the code
* System counts collected objects

### ROS Description
OpenMANIPULATOR launch file offers [a lot of services](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controller_msg/#topic). This project (specifically `/grabber_from_image_cords` node) uses some of these to control the manipulator: `/goal_task_space_path`, `/goal_joint_space_path`, `/goal_tool_control`.

The realsense node `/realsense2_camera_node` publishes [a lot of topics](https://github.com/IntelRealSense/realsense-ros/tree/ros2). This project (specifically `/tracker` node) uses only two: `/color/image_raw` and `/depth/image_rect_raw`.

Rviz2 software shares the feature of publishing data about the point, clicked on the displayed map, on topic: `/clicked_point`. Node `rosbot_control` subscribes to that topic.

Also some custom ROS2 Topics were made:

Topic | Message type | Publisher node name | Description
--- | ---| --- | ---
`/target_on_image` | `Point` | `/tracker` | The camera checks for the presence of object(s) and then publishes their XYZ location.
`/output/color` | `Image` | `/tracker` | Color image from the camera with indication of the detected color
`/output/black` | `Image` | `/tracker` | Black image with circle(s) graphically highlighting the position(s) of the object(s)
`/output/depth` | `Image` | `/tracker` | Depth image from the camera
`/manip_finished` | `Bool` | `/tracker` | The camera checks if the manipulator finished picking object(s) and publishes True of False.
`/rosbot_arrived` | `Bool` | `rosbot_control` | The ROSbot publishes True upon arrival to the camera and manipulator. 

System schematic diagram:

![diagram](schematic.png)
'*' More about topics published by ROSbot to rviz You can fine [here](https://github.com/husarion/rosbot-docker#publishes)


## Building the project
### Docker installation
Whole system runs on Docker Compose to make it as easy as possible to launch on different devices. To install Docker please refer to [Docker installation manual](https://docs.docker.com/engine/install/ubuntu/)

### Connecting to ROSbot via ssh
ROSbot is basically a computer running Ubuntu, so plug in a display with HDMI, mause and keyboard into USB port in the rear panel of ROSbot. Proceed step by step with [Connecting ROSbot to your Wi-Fi network](https://husarion.com/manuals/rosbot/#connect-rosbot-to-your-wi-fi-network) (e.g. ssh husarion@192.168.8.191)

### Cloning GitHub repository

##### Both on PC and on ROSbot:

Create new foler and clone this repository:
```bash
mkdir rosbot_manipulator_colaboration
git clone https://github.com/husarion/rosbot-manipulator-colaboration.git rosbot_manipulator_colaboration
```

### Mapping

<div align="center">
<iframe width="???" height="???" src="https://www.youtube.com/???" frameborder="0" gesture="media" allowfullscreen></iframe>
</div>

First You need to map rooms and surroundings with ROSBot and than save the map.
All you need to do is place robot on starting point and:

ON LAPTOP:
```bash
xhost local:root
cd rosbot_manipulator_colaboration/docker_stuff/
docker compose -f compose.rviz.mapping.yaml -f compose.rviz.lan.yaml up
```
ON ROSBOT:
```bash
cd rosbot_manipulator_colaboration/docker_stuff_rosbot/
docker-compose -f compose.rosbot.hardware.yaml -f compose.rosbot.mapping.yaml -f compose.rosbot.lan.yaml up
```

Navigate in your room using `2D Goal Pose` on Rviz. When map is finished open new terminal and again connect to ROSBot via ssh [like previously](https://husarion.com/tutorials/ros-projects/rosbot-manipulator-colaboration/#connecting-to-rosbot-via-ssh).
Then execute commands on ROSBot:
```bash
cd rosbot_manipulator_colaboration/docker_stuff_rosbot
./map-save.sh
```
Your map is now saved in the 'maps/' folder!

Now transfer the maps folder to your laptop using for example [sftp](https://linuxize.com/post/how-to-use-linux-sftp-command-to-transfer-files/) (e.g. sftp husarion@192.168.8.191)

Example result of the map:
![map](map.png)

### Launching project

ON LAPTOP:
```bash
xhost local:root
cd rosbot_manipulator_colaboration/docker_stuff/
docker compose -f compose.main.yaml -f compose.rviz.localization.yaml -f compose.rviz.lan.yaml up
```
ON ROSBOT
```bash
cd rosbot_manipulator_colaboration/docker_stuff_rosbot/
docker-compose -f compose.rosbot.control.yaml -f compose.rosbot.hardware.yaml -f compose.rosbot.localization.yaml -f compose.rosbot.lan.yaml up
```

Setup the hardware for example like this:

![setup](setup.jpg)

In addition, you can add some obstacles or build small maze:

![maze](maze.jpg)

Set your ROSbot on the starting point and put objects on its plate. Now, using `Publish Point` on Rviz chose starting point and destination point on the loaded map. Everything should look like in previously linked video:

<div align="center">
<iframe width="???" height="???" src="https://www.youtube.com/???" frameborder="0" gesture="media" allowfullscreen></iframe>
</div>

## Possible issues

#### 1. No image from Realsense camera

Every time you stop and rester the project first you need to unplug and plug USB cable of Realsense.

#### 2. Manipulator doesn't move

There are [some positions](https://github.com/ROBOTIS-GIT/open_manipulator/pull/46) of robotic arm which exceed its joint's limits.
Try to launch project with the initial pose like this:

![manip_initial_pose_example](manip_initial_pose_example.jpg)
