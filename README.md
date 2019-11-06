# Backstepping Controller


Code for the simulation-based paper **'Disturbance Observer-Based Integral Backstepping Controller for Multirotor UAVs'**.

This code is meant for use **SITL**(*Software in the loop*) approach to test the controller in the simulated world known as [jMAVSim](https://github.com/PX4/jMAVSim). It is easier to turn the parameter and test the controller in the simulated world before taking the Quatrotor to the sky. So we decide to release this code to reproduce the paper's result.

## Usage

### Install the Toolchain

**Note:** This code is developed on the **[PX4(1.5.5)](https://github.com/PX4/Firmware/releases/tag/v1.5.5)**, which is difference from the stock version, so we must install the PX4 toolchain manually using the following steps.

#### 1.Preparation

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

#### 2. Installation

Since this project relies on the submodule, so we have to init all the submodules. To clone the repo and init all the submodules, run:

```git clone git@github.com:Wonderful99668/UAV_IBS.git```

You can also do this separately:

```
git clone git@github.com:Wonderful99668/UAV_IBS.git
cd UAV_IBS
git submodule update --init --recursive
```

#### 3. Run SITL simulation with JMAVSim**