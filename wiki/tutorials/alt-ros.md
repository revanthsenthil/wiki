---
title: Tutorial - Alternative ROS Setup
date: 2021-06-01 00:00:00
---

> DEPRECATED: ARC now recommends using Docker for your ROS development environment. Installation using current recommendations can be found [here](ros)

The following tutorial has been tested for **MacOS (non-M1) and Linux** systems.

For **Windows**, use [ConstructSim](https://app.theconstructsim.com/): virtual, browser-based ROS environment with zero setup. Although, doesn't have access to hardware or networking, but is a good enough for beginners to get used to ROS. 

For **Mac with the M1 chip**, you can try to use [UTM](https://eshop.macsales.com/blog/72081-utm-virtual-machine-on-m1-mac ) to run a virtual machine that runs Linux. 

Prerequisites: 
- Basic knowledge of using the command line
  - Tutorials: ([Mac](https://www.youtube.com/watch?v=x3YfYVVTYvw), [Linux](https://www.youtube.com/watch?v=cBokz0LTizk))
- What is Conda? ([Conda Docs](https://docs.conda.io/en/latest/))

**Important notes:**

- Not all ROS packages are available using RoboStack. [Here](https://robostack.github.io/noetic.html)'s a list of all the supported packages for each platform. If the packages that you need aren't available, try opening an issue in the GitHub repo. 
- If all else fails, use the ConstructSim or official [ROS tutorials](https://www.ros.org/) using a Linux system (not using a VM or docker if you want access to hardware).

## Step 1. Setup ROS Noetic using RoboStack

### Install mambaforge
<details>
	<summary>Why am I doing this?</summary>
  Conda-forge, mamba-forge, mini-forge are infrastructure that allow you to use package managers such as conda and mamba that allow you download packages developed by a huge community of developers with a simple command on your terminal. The Robostack team put ROS packages on conda-forge using "recipes". 
  <br/>
  <br/>
  Conda also has added benefits of having a virtual environment system, where packages downloaded onto that virtual environment do not interfere with your normal packages on your computer, making it simple to delete an environment when something isn't working and start on a new slate with a new conda environment. You will be making some virtual environments in this tutorial. Mamba is basically like conda except it has extremely fast download speeds compared to conda, making it default in our tutorial.
</details>

Download mambaforge [here](https://github.com/conda-forge/miniforge#mambaforge). For Mac, choose the `x86_64` one.

### Run the installer

Open your terminal in MacOS/Linux or Windows and navigate to the directory with the installed file. Then run the following command:

**MacOS/Linux:**
```bash
bash <installer-you-just-downloaded>.sh 
```

After accepting terms and conditions, **select yes** to the option to run `conda init`, which will activate the miniforge conda base environment for you once the installer exits.

Make sure that the `mambaforge` folder exists in your home directory as the following conda setup assumes you do.

#### Configure conda setup behavior
<details>
	<summary>Why am I doing this?</summary>
  While conda is helpful, it may be a source of an unexpected error if you are doing something different and accidentally have it activated. This step makes it that you can activate it yourself only when you need it, avoiding this problem.
</details>

To avoid conflicts with pip or other installations, only activate your conda environment only when you need it. To disable the auto activation, run 
```
conda config --set auto_activate_base false
```

To manually activate your miniforge conda base environment, run:
```
source ~/mambaforge/bin/activate
```

To save yourself from typing that every time you open a new shell. Add this alias to your .bashrc or .zshrc:
```
echo "alias conda_init='source ~/mambaforge/bin/activate'" >> ~/.bashrc # Replace with .zshrc if using zsh
source ~/.bashrc
```

Then, just type `conda_init` in your terminal to automatically activate your base conda env. Make sure that you see `(base)` pop up.
```
conda_init
```

## Step 2: Setup conda environment with RoboStack

### Create the ros_env conda environment
<details>
	<summary>Why am I doing this?</summary>
  By doing all your installations in a separate virtual environment, it allows you to delete, copy all your package configurations very easily and even debug them if things go wrong. 90% of the time errors are because you are missing a package or have an incompatable version of a package, something easily debuggable with a virtual environment.
</details>
Ensure that your base conda environment is activated (should see `(base)` in your command line tool). Then run:
```
conda create -n ros_env python=3.8
conda activate ros_env
```

### Add channels and set channel priority
<details>
	<summary>Why am I doing this?</summary>
  This tells conda where to look for your packages. The robostack channel is important as it is where all the ROS packages are located. 
</details>

This adds the conda-forge and robostack channel to your persistent configuration in ~/.condarc.
```
conda config --add channels conda-forge 
conda config --add channels robostack
conda config --set channel_priority strict
```

## Step 3: Install ROS
### Install ROS using conda or mamba
> Do NOT install ROS packages in your `base` environment **make sure** that you see `ros_env`.

```
mamba install ros-noetic-desktop
```

### Install additional packages and reload environment
Install some compiler packages if you want to e.g. build packages in a catkin_ws - with conda:
```
mamba install compilers cmake pkg-config make ninja catkin_tools
```

> You can install any ROS Noetic packages that are [on this list](https://robostack.github.io/noetic.html) using `mamba install ros-noetic-name-of-ROS-package-with-dashes`

### Reload environment
Reload environment to activate required scripts before running anything,
 
```
conda deactivate
conda activate ros_env 
```

### (Optional) Install rosdep
<details>
	<summary>Why am I doing this?</summary>
  ROS packages all have a package.xml file that can define all the ROS packages that it depends on. This step initializes rosdep that allows you in the future to just do the following to install all the dependencies in your workspace. 
  <pre>
  # Installs all dependencies
  cd ~/catkin_ws # must be in workspace root dir
  rosdep install --from-paths src --ignore-src --rosdistro noetic -y
  </pre>
</details>

Make it simple to download all ROS packages using rosdep
```
mamba install rosdep
rosdep init  # note: do not use sudo!
rosdep update
```

## Step 4. Setup your ROS workspace 
By now you have ROS installed in your conda environment. You can now create a ROS catkin workspace and add some packages with it, and run it to test to see if things work.

### Create the catkin workspace
<details>
	<summary>Why am I doing this?</summary>
Your catkin workspace is where all your ROS packages will live and where all the action happens! 
</details>

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

### Add a ROS package in your workspace and build
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

### Run roslaunch
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

### Next Steps

If you're new to ROS and want to get a quick deep dive, check out the [ROS snake game tutorial](https://wiki.purduearc.com/wiki/tutorials/snake-tutorial)