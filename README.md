# Backstepping Controller


Code for the simulation-based paper **'Disturbance Observer-Based Integral Backstepping Controller for Multirotor UAVs'**.

This code is meant for use **SITL**(*Software in the loop*) approach to test the controller in the simulated world known as [jMAVSim](https://github.com/PX4/jMAVSim). It is easier to turn the parameter and test the controller in the simulated world before taking the Quatrotor to the sky. So we decide to release this code to reproduce the paper's result.

## Contents

  * [Important Files](#important-files)
  * [Usage](#usage)
    + [Installation](#installation)
      - [1.Install the Toolchain](#1install-the-toolchain)
      - [2. Download the code](#2-download-the-code)
    + [Run SITL simulation with JMAVSim](#run-sitl-simulation-with-jmavsim)
  * [Citing](#citing)
  * [Acknowledgement](#acknowledgement)

## Important Files

1. Main module of our backstepping controller: [mc_TASK](./src/modules/mc_TASK)

2. cmake config file which compile the project: [posix_sitl_custom](./cmake/configs/posix.sitl_custom.cmake)

3. jMAVSim configuration for the controller: [simulator constructor](./Tools/jmavsim/src/me/drton/jmavsim/Simulator.java#L430)

## Usage

### Installation

**Note:** This code is developed on the **[PX4(1.5.5)](https://github.com/PX4/Firmware/releases/tag/v1.5.5)**, which is difference from the stock version, so we must install the PX4 toolchain manually using the following steps.

#### 1.Install the Toolchain

* Install the clean **Ubuntu 16** for your development environment. 

(**Do not use** a higher version of Ubuntu (e.g., 18.04) as it will not work. **Do not use a VMWARE** VM or Virtualbox VM to run your Ubuntu as it will not support jMAVSim.)

* Ensure the user is added to the dialout group to have access to the usb ports. To add user to the dialout group you can use the following command and log out then back in for changes to take effect.

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

#### 2. Download the code

Since this project relies on the submodule, so we have to init all the submodules. To clone the repo and init all the submodules, run:

```git clone --recursive git@github.com:Wonderful99668/UAV_IBS.git```

You can also do this separately:

```
git clone git@github.com:Wonderful99668/UAV_IBS.git
cd UAV_IBS
git submodule update --init --recursive
```

### Run SITL simulation with JMAVSim

1. After ensuring you follow the steps above to setup the developing environment, the convenience "Make target" will compile the POSIX host build and start the jMAVSim simultaneously:

```make posix_sitl_custom jmavsim```

When successfully build, JMAVSim should be automatically launched and bring up the PX4 shell:

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

After you see the quad sitting in the center of the simulated world, you will be able to start flying once you have a position lock(shortly after the console displays the message: gps init …)

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

Thanks [PX4 team](https://px4.io/) for their great work on the open-source autopilot.

If you run into any problems when testing our code, please open the issue directly.
