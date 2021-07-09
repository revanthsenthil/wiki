---
layout: article
date: 2021-06-17
title: Software Documentation
author: Raghava Uppuluri
---

## System Overview

### Protoarm

```mermaid
graph
  subgraph Low Level
    direction LR
    a1[driver] -- servo position PWM --> a2[servos] 
    a2 -- potentiometer feedback \- not done --> a1
    a3[Protoarm ROS Controller]  --> a1[driver] 
  end
  subgraph Simulation
    direction LR
    b1[Gazebo ROS Controller] --> b2[Simulated robot joints from URDF] 
  end
  subgraph Mid Level 
    direction LR
    c1[MoveIt] --> a3 
    c2[MoveTo C++ ROS node] --> c1 
    c3[Visual Servoing ROS node] --> c2 
    c4[YOLOv5 Object Detector ROS node] --> c3 
    c5[Chessboard Detector] 
  end
  subgraph High Level 
    d1[Task Planner]-->c3
    d2[RL Policy]
  end
  c1 --> b1 
  c5 --> d1
  click c1 "https://moveit.ros.org/assets/images/diagrams/moveit_pipeline.png" "Open this in a new tab" _blank
  classDef not_started fill:#ff8181
  classDef in_progress fill:#ffba82
  classDef done fill:#81ff9b
  class c5,d1,d2,a3 not_started
  class c3,a1 in_progress
  class c1,c4,c2,c1,a2,b2,b1 done
```
```mermaid
graph
  l1[Not Started]
  l2[In Progress]
  l3[Done]
classDef not_started fill:#ff8181
classDef in_progress fill:#ffba82
classDef done fill:#81ff9b
class l1 not_started
class l2 in_progress
class l3 done
```

**NOTE:** The quick start code snippets in the following docs assume you have [setup](https://github.com/purdue-arc/arc_robot_arm) the `arc_robot_arm` repo.

## High level

### Task Planner
After the robot turns on, what does it do? This is the job of the task planner. Take inputs from the world using other tools in the stack, analyze, and make a decision. The decision flowchart that a robot can make is modeled here:
```mermaid
graph
  a1[Scanning for change in board state]
  p1[Virtual Human]
  p2[Engine]
  a3[Identify and pick up piece]
  a4[Identify destination and place piece]
  a1 -- 2d picture of board --> p1
  a1 -- FEN Notation of board--> p2
  p1 -- Next move --> a3
  p2 -- Next move --> a3
  a3 --> a4 --> a1
```

Some resources we can try to use to complete the above:
1. [FEN to stockfish chess engine](https://github.com/zhelyabuzhsky/stockfish)
2. 2D chess -> virtual human can be done using a text message API of some sort, where images of the board are sent over text and replies with the move can be recieved and played.

## Mid level

### Chess Piece Detection with YOLOv5

The object detection stack consists of the [YOLOv5](https://github.com/ultralytics/yolov5) object detection model trained on a custom dataset and outputs 2d bounding boxes. 

For chess, the model is trained with a custom large chess piece dataset of 500+ images stored on [roboflow](https://app.roboflow.com/). [Here](https://colab.research.google.com/drive/1XJ82eVA0cEfTczXFMtV1sunqvsfiJWQ0?usp=sharing) is a simple colab that walks through the training process using a Roboflow dataset.

Inference is using the `yolov5_pytorch_ros` ROS package using the `detector` ROS node using Python/PyTorch, allowing it to communicate detections and classes to other nodes such as for visual servoing. It reaches around 15-20 FPS without a GPU on a Mac.
> See [yolov5_pytorch_ros](https://github.com/raghavauppuluri13/yolov5_pytorch_ros) for more details and usage instructions.

#### Quick Start

1. Run the detector node
```
roslaunch chess_piece_detector detector.launch
```
2. In another window, run rqt by running `rqt`, click image to view images, and select `detections_image_topic`
3. If you have some chess pieces, put them in the camera view and see the detections

### Visual Servoing (WIP) 
We are using image-based visual servoing to localize the robot arm hand over an object in a graspable configuration, using images/2d bounding boxes as inputs to the system and servo commands as outputs.

<img src="assets/images/vs-diagram.png" alt="Diagram of controller" width="400"/>
> Visual servoing system diagram ([source](https://www.researchgate.net/figure/Visual-servoing-closed-loop_fig1_265436696))

See the `protoarm_visual_servoing` package for more details.

### Kinematics and Planning 
As of now, all the kinematics and planning heavy lifting is done by MoveIt. The `protoarm_kinematics` package houses a wrapper written in C++ that abstracts the process of sending [JointState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html) or [Pose](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html) goals. 

The wrapper is interfaced externally using the `move_to` node. Refer to the [test_kinematics](https://github.com/purdue-arc/arc_robot_arm/blob/main/protoarm_kinematics/src/test_kinematics) rospy file in `protoarm_kinematics/src` for usage of the `move_to` node. 

This package is also where we would keep our custom kinematics plugin and planning library if we choose to make it from scratch, instead of the default [KDL](https://ros-planning.github.io/moveit_tutorials/doc/kinematics_configuration/kinematics_configuration_tutorial.html#the-kdl-kinematics-plugin) kinematics plugin and [OMPL](https://ros-planning.github.io/moveit_tutorials/doc/ompl_interface/ompl_interface_tutorial.html) motion planning library. 

### Chessboard Detection
Playing chess is more than just picking and placing pieces. The robot needs to actually beat the human. We need to know the exact state of the chessboard, so another person or an overpowered engine can say what move to play next.

```mermaid
graph LR
  a1[Real chessboard] --> a2[2D chessboard] 
  a2 --> a3[FEN Notation] 
  a3 --> a4[Chess Engine]
  a2 --> a5[Virtual human player]
```
Some external projects we plan to use to complete the above:
- [Real chessboard -> 2D chessboard](https://github.com/maciejczyzewski/neural-chessboard )
- [2D chessboard -> FEN Notation](https://github.com/Elucidation/tensorflow_chessbot)

## Low level

### Controller 
`protoarm_control` is the ROS package that ensures that MoveIt execution commands are executed exactly as expected and to a degree of certainty in the real world. 

Right now, the driver communicates directly with MoveIt as the `protoarm_control` package doesn't exist yet.

Given that the protoarm uses some of the cheapest servos on the market, how can we still get dependable sub-millimeter precision to do tasks reliably? One way to do so is to hack our servos to add encoders, and use a control scheme that can take velocity and torque into account.

Inspired by Adam's [Servo Project](https://github.com/adamb314/ServoProject).

### Driver
The `protoarm_driver` is written in Arduino that actually interfaces with the servos and encoders. It sets joint limits, does coordinate frame conversions from the URDF to the actual robot, converts MoveIt angles to servo joint angles $$(-\pi,\pi)$$ to $$(0^\circ,180^\circ)$$, and executes servo commands using PWM to the servos.

## Simulation

### Gazebo

To speed up development and test software, our arm is simulated in Gazebo with a Realsense D435 camera (`realsense_ros_gazebo`), chessboard, and chess pieces (`chessboard_gazebo`). The robot arm and the camera are represented in [URDF](http://wiki.ros.org/urdf/XML/model) using XML. 

Sensors, actuation, gazebo plugins, and more are [XML specifications](http://wiki.ros.org/urdf/XML) that can be added. For the arm, these specifications exist in the `.xacro` files in the `urdf` folder of the `protoarm_description` ROS package.

Gazebo uses SDF models (in `models` folder of `chessboard_gazebo`) to represent static assets in the simulation like the chess pieces and chessboard. These objects are then spawned into a Gazebo world (along with the robot URDF), represented with a `.world` file.

#### Quick Start

1. Run the arm without the camera or chessboard
```
roslaunch protoarm_bringup sim.launch
```
Now, you can send kinematics commands using the `move_to` ROS node and the Gazebo arm will reflect the commands.

2. Run with both the camera and chessboard
```
roslaunch protoarm_bringup sim.launch realsense:=true chess:=true
```

Now, the camera topics are running, so you can run visual servoing, an RL algorithm, that subscribe to those exposed topics like the following demo.

{% include googleDrivePlayer.html id="19FZ7lsqCn6DEChjjdBgsTy0feUANYaVO/preview" %}

## To-do

#### Task Planner
- [ ] Implement Planner 
- [ ] FEN -> Stockfish 
- [ ] 2D chessboard -> virtual human 

#### Chess Piece Detection
- [ ] Open-source dataset 

#### Visual Servoing
- [ ] Implement feature extraction using OpenCV
- [ ] Implement controller

#### Chessboard Detection
- [ ] ROS node that can output 2d chessboard image and/or FEN

#### Controller 
- [ ] Implement controller

#### Driver
- [ ] Purchase and assemble encoder hardware on servos
- [ ] Write driver for encoder 
