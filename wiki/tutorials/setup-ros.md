---
title: Tutorial - Setup ROS
date: 2021-06-01 00:00:00
---

The following tutorial has been tested for **MacOS (non-M1) and Linux** systems.

For **Windows**, testing is currently in progress.

For **Mac with the M1 chip**, you can try to use [UTM](https://eshop.macsales.com/blog/72081-utm-virtual-machine-on-m1-mac ) to run a virtual machine that runs Linux. 

Prerequisites: 
- Basic knowledge of using the command line
  - Tutorials: ([Windows](https://www.youtube.com/watch?v=MBBWVgE0ewk), [Mac](https://www.youtube.com/watch?v=x3YfYVVTYvw), [Linux](https://www.youtube.com/watch?v=cBokz0LTizk))
- High level understanding of Conda ([Conda Docs](https://docs.conda.io/en/latest/))

**Important notes:**

- Not all ROS packages are available using RoboStack. [Here](https://robostack.github.io/noetic.html)'s a list of all the supported packages for each platform. If the packages that you need aren't available, try opening an issue in the GitHub repo. 
- If all else fails, use the official [ROS tutorials](https://www.ros.org/) using a Linux system (not using a VM or docker if you want access to hardware).

### 1. Setup ROS Noetic using RoboStack

Use the following [tutorial](https://robostack.github.io/GettingStarted.html).

### 2. Setup your ROS workspace 
By now you have ROS installed in your conda environment. You can now create a ROS catkin workspace and add some packages with it, and run it to test to see if things work.

#### 1. Create the catkin workspace
In your terminal, run the following commands to create the catkin workspace in your home directory, build, and initialize the workspace:

Creates the workspace filestructure (It is simply just a folder in your home directory called `catkin_ws`, although can be called anything, with an empty `src` folder in it)
```
# Goes to home directory
cd

# Creates catkin workspace folder
mkdir catkin_ws

# Navigates into the workspace folder and creates the src folder
cd catkin_ws
mkdir src
```

**Important:** Always build when you add new packages, create a new workspace, compiles C++ code, or add custom ROS message or ROS service files.
```
# Builds the new workspace (Make sure you are somewhere in the catkin_ws directory when your run this)
catkin build
```

**Important:** Run this to activate your workspace on start and after running catkin build to allow ROS to find any newly built packages
```
# Run the setup file (devel folder is always directly in the catkin workspace directory)
source devel/setup.bash # or setup.zsh if you use zsh
```

#### 2. Add a ROS package in your workspace and build
If you have a ROS package in mind to add to your workspace, add it to your `src` folder in your catkin workspace using the `git clone` command, then build and source.

If not, add this robot car test ROS package to your `src` folder, build, and source.

```
cd src
git clone https://github.com/raghavauppuluri13/robot_car_description.git
catkin build
source ../devel/setup.bash # or setup.zsh
```

**Helpful Debugging Command:**

To clean and rebuild your entire catkin workspace run this:

```
# Removes build, devel folders
catkin clean

# Builds all packages
catkin build

# Source catkin workspace ('~' means "relative to your home directory")
source ~/catkin_ws/devel/setup.bash # or setup.zsh
```

#### 3. Run roslaunch
If everything so far suceeeds, roslaunch the launch file in your own ROS package to test to see if things work.

If you added the `robot_car_description` package, run the following command:
```
# in general, roslaunch name_of_package name_of_launch_file.launch
roslaunch robot_car_description display.launch
```

You should get the following window:
![rviz demo image](assets/images/rvizdemo.png)

### Conclusion

At this point, you should
**Have:**
- A working ROS install using conda/RoboStack
- A catkin workspace

**Know how to:**
- Install packages using conda/mamba
- Create a catkin workspace
- Build a catkin workspace
- Add new packages to a catkin workspace
- Use roslaunch to run a ROS project
