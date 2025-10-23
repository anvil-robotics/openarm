# Using Enactic's Stack

Enactic's stack is a C++-based control system for OpenArm robots.

## Setup

### Install Dependencies

Install the following dependencies required to compiles Enactic's stack:

```bash
sudo apt install cmake g++ libeigen3-dev liborocos-kdl-dev liburdfdom-dev liburdfdom-headers-dev libyaml-cpp-dev
```

### 1. Compiling OpenArm CAN Project

Clone the project and navigate into it:

```bash
git clone https://github.com/enactic/openarm_can.git
cd openarm_can
```

Build and install the project to the system:

```bash
cmake -B build -D CMAKE_BUILD_TYPE=Release
cmake --build build
sudo cmake --install build
```

### 2. Compiling KDL Parser Project

Make sure SSH Key is already set up in GitHub (see [setup guide](./setup.md#setup-ssh-key-in-github)), then clone the project and navigate into it:

```bash
git clone git@github.com:anvil-robotics/kdl-parser.git
cd kdl-parser
```

Build and install the project to the system:

```bash
cmake -B build -D CMAKE_BUILD_TYPE=Release
cmake --build build
sudo cmake --install build
```

### 3. Compiling OpenArm Teleop Project

Clone the project and navigate into it:

```bash
git clone https://github.com/enactic/openarm_teleop.git
cd openarm_teleop
```

Build the project (note: this project is **not** installed to the system):

```bash
cmake -B build -D CMAKE_BUILD_TYPE=Release
cmake --build build
```

### 4. Preparing OpenArm URDF

Navigate to the `openarm_teleop` project directory and download the URDF file for the teleoperation demos:

```bash
curl -L -o openarm_bimanual.urdf https://gist.githubusercontent.com/threeal/d499e9d3e0be398bf9d030b7f3df970a/raw/08100a0ca3bcb360bd9d06822981d8a0d5a19d65/openarm_bimanual.urdf
```

Alternatively, you can generate the URDF file by following the [official guide](https://docs.openarm.dev/software/description/).

## Running the Demos

**Prerequisites** - Before running any demo:

1. All robots are **powered on** and connected via USB-CAN devices
2. All robots are placed in their **resting position**
3. USB-CAN devices are properly configured with persistent names (see [setup guide](./setup.md#configure-usb-can-devices))
4. OpenArm Teleop project is **built successfully** and URDF file is **prepared**

### Gravity Compensation Demo

Navigate to the `openarm_teleop` directory. You'll need **two terminals** to run both arms simultaneously.

**Terminal 1** - Left leader arm:
```bash
build/gravity_comp left_arm leader_l openarm_bimanual.urdf
```

**Terminal 2** - Right leader arm:
```bash
build/gravity_comp right_arm leader_r openarm_bimanual.urdf
```

This enables gravity compensation on the leader arms, allowing them to be moved freely by hand and stay in position without falling due to gravity.

For more information: `build/gravity_comp --help`

### Leader–Follower Teleoperation Demo

Navigate to the `openarm_teleop` directory. You'll need **two terminals** to control both arm pairs simultaneously.

**Terminal 1** - Left arm pair:
```bash
build/bilateral_control openarm_bimanual.urdf openarm_bimanual.urdf left_arm leader_l follower_l
```

**Terminal 2** - Right arm pair:
```bash
build/bilateral_control openarm_bimanual.urdf openarm_bimanual.urdf right_arm leader_r follower_r
```

This runs the leader arms with gravity compensation and makes the follower arms mirror the leader arms' movements in real time.

For more information: `build/bilateral_control --help`
