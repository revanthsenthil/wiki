---
date: 2021-06-01
title: Tutorial - Setup ROS with RoboStack (easy)
---

**PREFERRED FOR MAC AND WINDOWS USERS**

## Introduction
[ROS](https://www.ros.org/) is notoriously difficult to get working for beginners, especially if you have a Windows computer or Mac. You need access to a Linux OS, battle through GUI display issues, compilation issues, memory limitations if you use a VM, robot hardware access if you use WSL or Docker, and many more issues that scare beginners away.

Fortunately, the amazing team at RoboStack built [RoboStack](https://github.com/RoboStack/ros-noetic), an open-source ROS package management tool using [conda](https://docs.conda.io/en/latest/) (a package/environment manager), essentially allowing Linux, MacOS, or Windows to be able to run ROS natively!

RoboStack currently supports ros-noetic and ros2-foxy.

By the end of this tutorial, hopefully you will have a working ROS setup!

**Prerequisite: This tutorial assumes the reader has basic knowledge of using the command line on their system.**

## Step 1: Install miniforge (optionally mamba)
### 1.  Click one of the below links to download the installer. 

Latest installers with Python 3.9 in the base environment:

| OS      | Architecture          | Download  |
| --------|-----------------------|-----------|
| Linux   | x86\_64 (amd64)        | [Miniforge3-Linux-x86_64](https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86\_64.sh) |
| Linux   | aarch64 (arm64)       | [Miniforge3-Linux-aarch64](https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh) |
| Linux   | ppc64le (POWER8/9)    | [Miniforge3-Linux-ppc64le](https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-ppc64le.sh) |
| OS X    | x86\_64                | [Miniforge3-MacOSX-x86_64](https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-x86\_64.sh) |
| OS X    | arm64 (Apple Silicon) | [Miniforge3-MacOSX-arm64](https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-arm64.sh) |
| Windows | x86\_64                | [Miniforge3-Windows-x86_64](https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Windows-x86\_64.exe) |

Taken from the miniforge [README](https://github.com/conda-forge/miniforge),

### 2. Run the installer

Open your terminal in MacOS/Linux or Windows and navigate to the directory with the installed file. Then run the following command:

**MacOS/Linux:**
```bash
bash Miniforge3-your-installer.sh 
```

**Windows:** Double click on .exe file

**Note: at the moment on Windows only the Command Prompt terminal is supported, while Powershell is not supported.**

After accepting terms and conditions, **select yes** to the option to run `conda init`, which will activate the miniforge conda base environment for you once the installer exits.

#### Helpful tip: 
To avoid conflicts with pip or other installations, only activate your conda environment only when you need it. To disable the auto activation, run 
```
conda config --set auto_activate_base false
```

#### Another Helpful tip:
To manually activate your miniforge conda base environment, run:
`source /path/to/miniforge3/bin/activate`

To save yourself from typing that every time you open a new shell. Add this alias to your .bashrc or .zshrc:
```
alias forge_init='source /path/to/miniforge3/bin/activate'
```
Then, just type`forge_init` in your terminal to automatically activate your base conda env.

If you are unfamiliar with .bashrc or .zshrc: check [this tutorial](https://medium.com/@tzhenghao/a-guide-to-building-a-great-bashrc-23c52e466b1c) out.

### 3. Install mamba (Optional, but recommended)
If you want to have faster package download speeds, install [mamba](https://github.com/mamba-org/mamba)

**Ensure that your miniforge base environment is activated**, then run:
`conda install mamba -n base -c conda-forge`

You can then install packages like this:
`mamba install package-name`

## Step 2: Setup conda environment with RoboStack
### 1. Create the robostack conda environment
Ensure that your base conda environment is activated (should see `(base)` in your command line tool). Then run:
```
conda create -n robostackenv python=3.8
conda activate robostackenv
```

### 2. Add channels and set channel priority
This adds the conda-forge and robostack channel to your persistent configuration in ~/.condarc.
```
conda config --add channels conda-forge robostack
conda config --set channel_priority strict
```

## Step 3: Install ROS
### 1. Install ROS using conda or mamba
> Do NOT install ROS packages in your `base` environment.

```
conda install ros-noetic-desktop

# or if you have mamba and want to use it
mamba install ros-noetic-desktop
```

### 2. Install additional packages and reload environment
Install some compiler packages if you want to e.g. build packages in a catkin_ws - with conda:
```
conda install compilers cmake pkg-config make ninja catkin_tools

# or with mamba:
mamba install compilers cmake pkg-config make ninja catkin_tools
```

You can install any ROS Noetic packages that are [on this list](https://robostack.github.io/noetic.html) using:

`conda install ros-noetic-name-of-ROS-package-with-dashes`
or with mamba:
`mamba install ros-noetic-name-of-ROS-package-with-dashes`

**On Windows, install Visual Studio command prompt via Conda:**
```
conda install vs2019_win-64
```

### 3. Reload environment
 Reload environment to activate required scripts before running anything,
 On Windows, restart the Anaconda Prompt / Command Prompt.
 
```
conda deactivate
conda activate robostackenv
```

## Final Step: Test if your ROS install works
By now you have ROS installed in your conda environment. You can now create a ROS catkin workspace and add some packages with it, and run it to test to see if things work.

### 1. Create the catkin workspace
In your terminal, run the following commands to create the catkin workspace in your home directory, build, and initialize the workspace:

Creates the workspace filestructure.
```
mkdir ~/catkin_ws && mkdir ~/catkin_ws/src
```

Always build when you add new packages, create a new workspace, update C++ code that needs to be compiled, or add custom message or service files.
```
cd ~/catkin_ws && catkin build
```

**Important:** Run this to activate your workspace on start and after running catkin build to allow ROS to find any newly built packages
```
source ~/catkin_ws/devel/setup.bash # or setup.zsh if you use zsh
```

### 2. Add a ROS package in your workspace and build
If you have a ROS package in mind to add to your workspace, add it to your `src` folder in your catkin workspace using the `git clone` command, then build and source.

If not, add this robot car test ROS package to your `src` folder, build, and source.
```
cd ~/catkin_ws/src
git clone https://github.com/raghavauppuluri13/robot_car_description.git
catkin build && source ~/catkin_ws/devel/setup.bash # or setup.zsh
```

### 3. Run roslaunch
If everything so far suceeeds, roslaunch the launch file in your own ROS package to test to see if things work.

If you added the `robot_car_description` package, run the following command:
```
# in general, roslaunch name_of_package name_of_launch_file.launch
roslaunch robot_car_description display.launch
```

You should get the following window:
![[rvizdemo.png]]

## Conclusion

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
