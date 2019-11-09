# Simulation of a Disturbance Observer-Based Integral Backstepping Controller using PX4+SITL+jMAVSim


This repo contains the simulation code for the IFAC 2020 conference paper submission **'Disturbance Observer-Based Integral Backstepping Controller for Multirotor UAVs'**. 

The code simulates a disturbance observer-based integral backstepping control for a multirotor UAV running the [PX4 autopilot firmware](https://px4.io/). **SITL**   (*Software in the loop*) is combined with the [jMAVSim](https://github.com/PX4/jMAVSim) simulator. The reason for using SITL simulation is to test controller performance using actual PX4 firmware. This ensures the controller is generally implementable on-board physical autopilots (e.g. Pixhawk 1) and that simulation results are closer to what are observed in flight testing. 

## Contents

  * [Important Folders and Files](#important-files)
  * [Usage](#usage)
    + [Installation](#installation)
      - [1.Install the Toolchain](#1install-the-toolchain)
      - [2. Download the code](#2-download-the-code)
    + [Run SITL simulation with JMAVSim](#run-sitl-simulation-with-jmavsim)
  * [Citing](#citing)
  * [Acknowledgement](#acknowledgement)

## Important Folders and Files

1. Folder containing the module which implements the backstepping controller: [mc_TASK](./src/modules/mc_TASK)

2. cmake config file for compiling the project: [posix_sitl_custom](./cmake/configs/posix.sitl_custom.cmake)

3. jMAVSim configuration for the controller: [simulator constructor](./Tools/jmavsim/src/me/drton/jmavsim/Simulator.java#L430)

## Usage

### Installation

**Note:** This code is developed on the **[PX4 (v1.5.5, Jan 25, 2017)](https://github.com/PX4/Firmware/releases/tag/v1.5.5)**, which is behind the current version of the firmware. Hence, the PX4 toolchain should be installed manually using the following steps. Note that this installation will only allow SITL simulation and not compilation for an actual autopilot such as Pixhawk 1. 

#### 1.Installing the Toolchain

* Perform a clean install of **Ubuntu 16.04** for your development environment. **Do not use** a more recent version of Ubuntu (e.g., 18.04) as it will not work. **Do not use** a VMWARE Workstation VM or Virtualbox VM to run your Ubuntu as it will not support jMAVSim.

* Ensure the user is added to the dialout group to have access to the usb ports. To add a user to the dialout group you can use the following command and log out then back in for changes to take effect.

```
sudo usermod -a -G dialout $USER
```

* Update package list and install the PX4 build dependencies

```
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update
sudo apt-get install python-argparse git-core wget zip python-empy qtcreator cmake build-essential genromfs -y
sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-8-jdk openjdk-8-jre clang-3.5 lldb-3.5 -y
```

#### 2. Downloading the Code

The PX4 firmware depends on a number of other projects which are included as submodules. To clone the repo using HTTPS and init all the submodules, run:

```git clone --recursive https://github.com/ANCL/UAV_IBS.git```

Equivalently, you can do this in two steps with:

```
git clone https://github.com/ANCL/UAV_IBS.git
cd UAV_IBS
git submodule update --init --recursive
```

### Running the SITL/jMAVSim Simulation

1. The convenience "Make target" will compile the POSIX target build and start the jMAVSim simultaneously:

```make posix_sitl_custom jmavsim```

When the build is successful, JMAVSim and the PXH shel are launched. In the terminal you should see

```
______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting
...
INFO [tone_alarm] home_set
INFO [tone_alarm] neutral
pxh>

```

A separate jMAVSim window should show the quadrotor sitting in the center of the simulated world. You can begin flying once you have a position lock which is when  PXH  shell displays "gps init".

2. Change quad mode. * There are two flying modes in our customized version of PX4: **ANCL1** and **ANCL2**.

* **ANCL1 mode**: This is to take the quad to the hover at center of the world 1 meter high.

* **ANCL2 mode**: This is the mode that the quad uses our Backstepping Controller (e.g.: circle; 8 figure).

For the safety consideration, before you change to the **ANCL2** mode, make sure you first stay on the **ANCL1** mode to make the quad hover at 1 meter:

```
commander mode ancl1
commander arm # arm the quad, make the quad hover at 1 meter
commander mode ancl2 # change the mode to use our controller
```

## Citing



## Acknowledgement

Thanks to the [PX4 team](https://px4.io/) for their open-source autopilot on which this code is based.

If you run into any problems when testing our code, please open the issue directly.
