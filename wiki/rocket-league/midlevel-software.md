---
date: 2021-07-11
title: Midlevel Software
show_author_profile: true
author: Harrison McCarty, Adrian Teigen
---

This article details the inner workings of the midlevel software, which includes the following components:

- Waypoint controller
- Trajectory planner
- Simulator

## Usage

<!-- TODO: fill in section -->

## How it works

### Waypoint controller

<!-- TODO: fill in section -->

### Trajectory planner

<!-- TODO: fill in section -->

### Simulator

**Summary**

<!-- TODO: fill in section -->

The simulator uses Box2d for python. It allows us to test the High Level Planner, the Waypoint Controller, and the Trajectory Planner.

The simulator subscribes to the car's desired velocity (from the Waypoint Controller).

The simuilator publishes the car and field poses.



**Constants**

The simulator has multiple constants which can be found in the racersim/YAML_files/sim_info.yaml file. All non-abstract constants are in SI (metric) unless otherwise specified. E.g. metres, kilograms, and newtons. Some numbers have been measured, others are made up.



**ROS Interface**

The racersim/nodes/racersim_node file interfaces with ROS. It does the following:

- Reads constants from the YAML file (racersim/YAML_files/sim_info.yaml)

- Subscribes to car0/{velocity, path, lookahead_pnt} which are sent by the Waypoint Controller.

- Publishes to {car0, ball}/odom.

- Initializes the Sim class (found in sim.py)

- Performs a sim step (with some Δt)



**sim.py**

The racersim/src/racersim/sim.py is the most important file for the simulator. It contains the following.

- An initializer.
- The step function.



When the Sim class is initialized, it will also initialize the world (the field), the tires, the car, and the ball. Each object has it's own file.

The initializer will also create a random/semi-random/deterministic start position for the ball and the car, as well as a starting angle for the car.



This simulator uses timesteps to calculate the relevant physics. The Sim class contains a step function. This function will call 2 other step functions. 



The self.car.step() function calls the step function in car.py. This functions does things such as turning the tires towards the ball, and calling updateFriction() and updateDrive() in tire.py.



The self.world.Step() function does something else. I imagine that it calls some function defined in Box2D (our physics engine).



It will also call the self.\_render() function which draws every object in a new window. Make sure that x-forwarding works if you cannot see this window when running WSL 2. 



**Common Mistakes**

The x-coordinates are intuitive. Larger x-coord ==> more to the right.

The y-coordinates are not intuitive. Larger y-coord ==> further down.

The coordinate (0,0) is found in the top-left of the window.

This causes a lot of confusion during angular velocity calculations.
