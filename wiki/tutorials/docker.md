---
title: Tutorial - Docker
date: 2021-09-19
author: James Baxter
---

The following tutorial has been tested for Windows and Linux computers.

This guide will walk you through setting up your computer to run Docker, a
lightweight virtual machine that allows you to run a standardized development
environment on your machine. It is used extensively by the Rocket League
project team, but other options to get access to ROS do exist.

Most of the content of this guide is replicated from [The ARC Tutorials GitHub
Repo](https://github.com/purdue-arc/arc_tutorials/blob/master/docs/00_introduction.md),
but it has been modified sligthly to be kept up to date.

# Development Environment Explanation
The ARC development environment is an Ubuntu 20.04 instance with ROS Noetic
installed. It has a few other tools installed as well. You will access it
primarily through a shell (command line) interface, but you can also use an IDE
like Visual Studio Code to develop as if everything was installed on your
machine directly.

## ROS
ROS stands for Robot Operating System. It is an industry standard tool for
creating autonomous robots. Essentially, it is a framework to create a mesh of
nodes that work together to control your robot. It has an API for both C++
and Python. Through it, you have a common language to define messages that
are sent from one node to another (for example a path planning node can send
waypoint messages to a low level control node). The beauty of it is that all the
nodes can be modular and developed independently if you have a well defined
interface of messages.

Another really excellent reason to use ROS is the 5000+ number of existing
packages that can act as 'plug and play' into your existing network. If you have
a new sensor you're looking to try, someone has probably already written a ROS
driver for it. Tasks such as mapping, perception, and state estimation that many
robots need to do probably have multiple packages that you can experiment with
and pick the best one for your specific application. There are also pre-defined
messages for standard things like sensor data and control commands. For these
reasons it can be really useful for jump-starting a new project.

Through this club, you will be learning much more about ROS. The `snake_tutorial`
package will walk you through creating a few simple nodes that can be chained
together to control a snake to reach a goal. You will also have the opportunity
to expand on the very basic controller given to you in an open ended challenge
to create the highest performing AI for the snake.

A downside of ROS is that it works really well on Ubuntu, but isn't well
supported on other operating systems. This is why we must set up a very specific
development environment for the club.

For more information on ROS, check out their wiki: <http://wiki.ros.org/ROS/Introduction>

# Operating System
See the below instructions depending on what operating system you are running.

## Windows
This guide was mainly written for students using Windows 10. Historically, this
has been the dominant OS for our members, and it was also difficult to get
started on. Read and follow all set up documents in order. You will first 
install and configure WSL2, then set up Docker, then build and test the ARC
development environment.

Versions of Windows other than Windows 10 do not appear to be supported by
Docker, so therefore the ARC development environment is not supported on them.
You will need to upgrade to Windows 10, dual boot, or run a Linux VM. That is
outside the scope of this document.

## Mac
Presently, this guide has not been fully tested on a Mac system. In theory it
should because there is a version of Docker for Mac, but some additional work is
required to get all features to funciton. If you're interested in running this
on Mac, understand that some experimentation bay be required. You can skip
directly over the WSL2 guide (since it is Windows specific) and continue with
Docker setup. You will then build and test the ARC development environment.

## Linux
This guide has been tested to work on Ubuntu 18.04 and 20.04. If you have a
different distribution, it should also work without issue, provided there is a
Docker release for you. If you run Linux as a dual boot with one of the above
operating systems, it may be worth installing the ARC development environment in
both. See the notes on dual booting below. For setting up the environment, you
can skip directly over the WSL2 guide (since it is Windows specific) and
continue with Docker setup. You will then build and test the ARC development
environment.

## Dual Boot Considerations
For the best possible experience, it may be worth looking into setting up a dual
boot. ROS was designed to run on Ubuntu, so running it natively without anything
extra in the way will give you the least hassle when actually trying to run
code. Setting up the dual boot can be quite a hassle for an inexperienced user,
which is why we recomend using WSL2 / Docker. This also saves you from needing
to restart your computer every time you want to switch modes.

Setting up a dual boot is outside the scope of this guide, but you should be
able to find plenty of resources online. You will want to install the version of
Ubuntu that is correct for the version of ROS you are intending to use. This
information can be found the [ROS wiki](http://wiki.ros.org/ROS/Installation)
Presently, the club is using ROS Noetic, which is designed to run on Ubuntu
20.04. You will then need to decide if you are running ROS natively or though
Docker. That is also outside the scope of this document.

You can also install a different version of Ubuntu for your dual boot, then use
Docker in order to run the version you need. This will likely give you better
performance than running Docker on Windows or Mac, and you can follow the Linux
set up steps noted above.

# WSL2
WSL2 stands for Windows Subsystem for Linux verison 2. Essentially, Windows is
running a full Linux system in tandem with Windows. This has much better
performance than running a virtual machine (VM) or using the original WSL.

This document will guide you through setting up WSL 2 on Windows 10.

## Installing WSL 2
Microsoft has published an [online guide](https://docs.microsoft.com/en-us/windows/wsl/install-win10),
for setting up WSL2, which contains the most up-to-date information for installing
WSL2 on Windows. Follow the guide fully, and for step 6, install [Ubuntu 20.04 LTS](https://www.microsoft.com/store/apps/9n6svws3rx71).

Make sure to [set up a non-root user account](https://docs.microsoft.com/en-us/windows/wsl/user-support)!

## Setting up X forwarding
X forwarding is what will let you display GUIs (graphical user interfaces)
on your Windows machine that are being run from your WSL2 instance. You need
to install an X client, set up the appropriate firewall rules, then tell WSL2
to forward its display to your client.

### X Client Set Up
There are several options available here. VcXsrv has been tested to work with
the ARC development environment and is available on [SourceForge](https://sourceforge.net/projects/vcxsrv/).
VcXsrv is the recommended client.

If you install VcXsrv, launch it with the `XLaunch.exe` executable, not `VcXsrv.exe`.
When you launch it for the first time, you will see several configuration pages.
You may leave them all at the default: multiple windows, no clients.

### Windows Firewall Setup
When you first launch your X client, you should see Windows Firwall pop up. Go
ahead with the default settings.

Now that everything is blocked, we need to put in an exception so that our WSL2
instance is able to access the X client.

These instructions are taken from a [Reddit post](https://www.reddit.com/r/Windows10/comments/gd7n5z/how_do_i_convince_windows_10_that_a_vethernet/)
1. Launch Windows Firewall. Simply search "Firewall" in the Start Menu
2. [Go to properties](https://i.imgur.com/oo9L3pH.png)
3. [Go to "Public Profile](https://i.imgur.com/ZbSjO8k.png)
4. [Select "Customize"](https://i.imgur.com/1rOprzr.png)
5. [Uncheck the "(WSL)" option](https://i.imgur.com/9ZaWc3S.png)

Go ahead and save and close everything. You may need to reboot your X client.

### WSL2 Setup
We will need to edit a file called the `bashrc`. This is a script that is run
every time you open a new terminal in order to set up your environment. Modify
it by copy and pasting the below command in _exactly_. Run these commands in
WLS2, not Powershell or Windows Commandline. From now on, all commands to be
listed like this should be put into the WSL2 commandline.
```bash
echo -e '\nexport DISPLAY=$(awk '\''/nameserver / {print $2; exit}'\'' /etc/resolv.conf 2>/dev/null):0\nexport LIBGL_ALWAYS_INDIRECT=1' >> ~/.bashrc
```
Since we just changed our `bashrc`, we need to run the below command for the
changes to take effect in our current terminal:
```bash
source ~/.bashrc
```

### Testing
You will need to download a small program called `xeyes` in order to test the X
forwarding. It can be installed and run with the following commands:
```bash
sudo apt install -y x11-apps
xeyes
```
You'll see a little pair of eyes follow your cursor around. You can X out of it
or hit `CTRL+C` in order to kill the program.

## Installing Microsoft Terminal
For a much nicer terminal experience, you will want to install the [Windows
Terminal](https://aka.ms/terminal). Once it is installed, launch it, then click
the downwards arrow icon on the list of tabs. Select Ubuntu 20.04 from the
dropdown, and you will be greeted with a bash terminal for your WSL2 instance.

If you'd like to make the default behavior to open WSL2 tabs, you can hit the
dropdown and select settings.

# Docker
Docker is a lightweight virtual machine (VM) that will let you run the ARC
development environment with very little overhead. Docker is useful because of
how lightweight it is, and how these VM instances can be created
programatically. You will learn more about how to use Docker when setting up the
ARC development environment.

This document will guide you through setting up Docker on your machine.
See the below section depending on what OS you are using.

## Windows
Follow the [installation guide](https://docs.docker.com/docker-for-windows/install-windows-home/)
to set up Docker to use the WSL2 backend.

The ARC development environment is currently untested on Windows 10 Pro. There 
are alternate [installation instructions](https://docs.docker.com/docker-for-windows/install/)
provided, and it seems you will have the option to use the WSL2 backend. For the
sake of having a uniform configuration for the club, use the WSL2 backend. There
are [additional notes](https://docs.docker.com/docker-for-windows/wsl/) that may
be useful.

## Mac
The ARC development environment is completely untested on Mac, but it should
work. Follow the [installation guide](https://docs.docker.com/docker-for-mac/install/).

## Linux
The ARC development environment has been tested on Ubuntu 18.04 and 20.04. Other supported
distributions should work too. Follow the [installation guide](https://docs.docker.com/engine/install/ubuntu/)
for your distribution. In the case of Ubuntu, installing via the repository is
the preferred method of installation.
