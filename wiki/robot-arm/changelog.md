---
date: 2021-06-01
title: Changelog 
---
## 2021
- 
- 
-
## Fall 2020 

### November/Early December
- Final design of a 6DOF arm was made
- A HACKY solution was made through porting that python3 code into ROS melodic (python2), but ended up working to accomplish our goal. 
- To solve the previous issue, generated the Denavit Hartenburg parameters for the arm and used [Peter Corke's Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/), which suprisingly converged every time. 

### October
- Found that using the MoveIt solver in ROS for our 5DOF arm never converged on a solution. 

- Found that the 5 DOF arm gripper was too heavy for the motors we had. Took 2-3 weeks to make the new gripper due to 3D printing reliability. The [new gripper]() was lighter, but was misaligned and frail, but we used it in the end with the power of duct tape.

### September
- Robot arm project was created to be a robotics educational platform for students to learn robotics. In the semester (Sept - Dec), we aimed to design a custom 6 DOF arm that can play Jenga and develop software using ROS to control it. At this point, a 5 DOF robot arm was 3D printed, assembled, and wired using [this](https://www.youtube.com/watch?v=_B3gWd3A_SI) tutorial. 
