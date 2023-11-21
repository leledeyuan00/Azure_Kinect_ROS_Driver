# This Branch is for How to use Orbbec Camera instead of Microsoft Azure Kinect Camera

Due to the product line stopped by Microsoft, after 2023 we would only use Femto series provided by Orbbec China.

They wrapped their own SDK caused some libraries such as pose tracking package not free.

But at the same time, considering the amount users of Azure Kinect Camera, they also provided k4a wrapper for using original API.

So this document is wrote to how to use Microsoft API instead of Orbbec.

## Docker

They edit k4a library so that we need build it, but it is designed for Ubuntu20 and would confict with original API, so we must use Docker here.

If you already have an Ubuntu 20 docker container and could communicate with your local env out of the Docker with ROS, you could skip this part.

This Docker provide a `packages` folder. Which could transfer files between local env and Docker easily.

1. Create a Docker ws Following [Foxy Ros Repository](https://github.com/leledeyuan00/Foxy_Display_Docker)

2. Install the Nvidia driver inside Docker. This could by using [Cuda Updated](https://developer.nvidia.com/cuda-downloads). Or by yourself, but make sure this is the same version with your local environment.

## Build the Customized API

Inside docker, download the [k4a wrapper API](https://github.com/orbbec/OrbbecSDK-K4A-Wrapper) provided by Orbbec.

As the Linux build part to build this library. With Cmake is fine.

## Install the Body Tracking Package

### Download the libabt and libabt-dev

Due to Microsoft didn't provide the source code of body tracking. We could only install it by deb they provided.

Download the [k4abt](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1/libk4abt1.1_1.1.2_amd64.deb) and [k4abt-dev](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1-dev/libk4abt1.1-dev_1.1.2_amd64.deb) into the `packages` directory or directly download in the Docker by `wget .....`

Now create a dummy package, because the k4abt depends on k4a1.4 but we install it by source, so it cannot find it.

`apt install equivs`

This command-line tool is used to create a dummy deb package to satisfy k4abt ....

### Create libk4at1.4.deb and libk4a1.4-dev.deb

`equivs-control libk4a1.4`

Edit the package name and version as follow

```
### Commented entries have reasonable defaults.
### Uncomment to edit them.
Source: libk4a1.4
Section: misc
Priority: optional
# Homepage: <enter URL here; no default>
Standards-Version: 3.9.2

Package: libk4a1.4
Version: 1.4.1
# Maintainer: Your Name <yourname@example.com>
# Pre-Depends: <comma-separated list of packages>
# Depends: <comma-separated list of packages>
# Recommends: <comma-separated list of packages>
# Suggests: <comma-separated list of packages>
# Provides: <comma-separated list of packages>
# Replaces: <comma-separated list of packages>
# Architecture: all
# Multi-Arch: <one of: foreign|same|allowed>
# Copyright: <copyright file; defaults to GPL2>
# Changelog: <changelog file; defaults to a generic changelog>
# Readme: <README.Debian file; defaults to a generic one>
# Extra-Files: <comma-separated list of additional files for the doc directory>
# Links: <pair of space-separated paths; First is path symlink points at, second is filename of link>
# Files: <pair of space-separated paths; First is file to include, second is destination>
#  <more pairs, if there's more than one file to include. Notice the starting space>
Description: <short description; defaults to some wise words> 
 long description and info
 .
 second paragraph
```

Build the deb

`equivs-build libk4a1.4`

Then there would generate a new deb, let's install it.

`dpkg -i libk4a1.4_1.4.1_all.deb`

Then create libk4a1.4-dev as same process.

`equivs-control libk4a1.4-dev`

Edit..

`equivs-build libk4a1.4-dev`

`dpkg -i libk4a1.4-dev_1.4.1_all.deb`

### Install the libabt now

`dpkg -i libk4abt1.1_1.1.2_amd64.deb`

`dpkg -i libk4abt1.1-dev_1.1.2_amd64.deb`


## Building the ROS2 Interface

Now we could use this repository

Create a workspace and git clone this repository.

`mkdir ~/orbbec_ws/src && cd ~/orbbec_ws/src`

`git clone -b orbbec https://github.com/leledeyuan00/Azure_Kinect_ROS_Driver.git`

`cd ~/orbbec_ws`

`rosdep install --from-paths src --ignore-src -r -y`

`colcon build`

## Running driver launch with body tracking

`. install/setup.bash`

`ros2 launch azure_kinect_ros_driver driver.launch.py`

## Showing the image and body tracking results

Create a new terminal fow rviz2.

