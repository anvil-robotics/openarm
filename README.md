# Anvil's OpenArm Experimental Testing Tools

This repository contains a variety of python scripts and debugging tools for validating Anvil's OpenArm hardware platform. Please note that this code is very experimental.

As this code is still experimental, there will be breaking changes, and is not meant for production use cases.

For commercial use cases and customers, Anvil Robotics is developing a high performance robot control and human demonstration data collection suite (early-access in Dec 2025). Feel free to reach out to `<tbd>@anvil.bot`, if you're interested.

## Prerequisites

- Python 3.11 or higher
- Ubuntu 24.04 LTS
- An Anvil OpenArm Leader or Follower ([See Anvil Store](https://shop.anvil.bot/)), fully assembled and wired ([see Anvil's hardware assembly guide](https://www.notion.so/anvil-robotics/Anvil-s-OpenArm-Getting-Started-2025Q4-HW-Batch-2948f2efe5658066bac0daa9de065f54)).

*(While the code will likely run on other platforms, your mileage may vary)*

## Setup

### Getting the Code
```bash
git clone https://github.com/anvil-robotics/openarm.git
cd openarm
# <or>
git clone git@github.com:anvil-robotics/openarm.git
cd openarm
```

### Setting up the SW Environment
#### Option 1: Setting up a Python Virtual Environment (recommended) ####
Install Python virtual environment with:

```bash
sudo apt install python3-venv
```

Create and activate a new virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Install the project dependencies:

```bash
pip install -e .
```

Each time you open a new terminal, navigate to inside the `openarm` directory and reactivate the virtual environment with the following:

> ```bash
> source .venv/bin/activate
> ```


#### Option 2: Using VSCode+DevContainer ####
If you are already familiar [VSCode and devcontainers](https://code.visualstudio.com/docs/devcontainers/containers), you can simply open the cloned openarm folder and VSCode will detect the [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json) file.


### Configuring USB-CAN Devices (udev rules)
Each OpenArm uses 2x [Canable-style](https://canable.io/) USB-to-CAN devices (one for the left arm and one for the right arm). Each device is pre-flashed with [Candlelight firmware](https://github.com/normaldotcom/candleLight_fw). These devices are automatically detected and enumerated by the Linux host when plugged in. But, by default, the device's name that shows up in Linux depends on the order in which devices are plugged in & detected by the host. So, without any additional setup, it becomes extremely easy to accidentally swap commands between the left and right arms.

#### Option 1: Script-Guided udev setup (recommended) ####
Download and run this setup script to assign persistent names to the USB-CAN devices (based on their unique serial numbers) and bring them online automatically:

```bash
curl -L -o setup_can.sh https://gist.githubusercontent.com/threeal/f9e982150d3a1836c71b274561044549/raw/ba15c387c4c75565370f6739bcaa25ba8bddfa9a/setup_can.sh
# Run the following, if you are connecting a leader+follower pair
sudo bash setup_can.sh leader_l leader_r follower_l follower_r
# Run the following, if you are connecting a single robot
sudo bash setup_can.sh robot_l robot_r
```

The script will guide you through plugging in four (or two) USB-CAN devices: two for the leader arms and two for the follower arms. Each argument corresponds resulting device names, and the order they are should be connected.

#### Option 2: Manual udev setup
For more complex or especially unique setups, you can manually create & install your own udev rules. There are lots of online resources describing udev, and i2rt has an [example](https://github.com/i2rt-robotics/i2rt/blob/main/doc/set_persist_id_socket_can.md) of what this could look like as well.

## Running Anvil's Experimental Tools

### Individually Testing Every Motor (Hello World)

The following instructions validate that every motor is functional and communicating as expected: See [motor-testing.md](motor-testing.md).

### Motor Monitoring

Monitor motor angles in real-time across all detected CAN buses:

```bash
# Monitor all motors (passive mode - motors disabled)
python -m openarm.damiao.monitor

# Specify custom CAN interface
python -m openarm.damiao.monitor --interface leader_l
```

### Direct Motor Control

Low-level control of individual motors for testing and debugging:

```bash
# Enable a motor
python -m openarm.damiao enable --motor-type DM4310 --iface can0 1 1

# Set MIT Mode and Command motor with MIT parameters
python -m openarm.damiao param set --motor-type DM8009 --iface can0 1 17 control_mode 1
python -m openarm.damiao control mit --motor-type DM4310 --iface can0 1 1 50 0.3 0 0 0  # kp kd q dq tau

# Set PosVel control mode and Command motor with PosVel command
python -m openarm.damiao param set --motor-type DM8009 --iface can0 1 17 control_mode 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface can0 1 1 1.57 2.0  # position(rad) velocity(rad/s)

# Get motor status
python -m openarm.damiao refresh --motor-type DM4310 --iface can0 1 1

# Get motor parameters
python -m openarm.damiao param get --motor-type DM4310 --iface can0 1 1 over_voltage

# Disable motor safely
python -m openarm.damiao disable --motor-type DM4310 --iface can0 1 1
```

**Common Arguments:**

- `--motor-type TYPE`: Motor type (`DM4310`, `DM4340`, `DM6006`, `DM8006`, `DM8009`)
- `--iface INTERFACE`: CAN interface (default: `can0`)
- `slave_id`: Motor ID for commands
- `master_id`: Master ID for responses

**Available Commands:**

- `enable/disable`: Motor power control
- `set-zero`: Set current position as zero
- `refresh`: Get current motor state
- `control`: MIT, position/velocity, velocity, or position/force control
- `param get/set`: Read/write motor parameters
- `save`: Save parameters to flash memory

### Register Dump
Reads all configuration registers from all motors on all busses

(coming soon)

## Running Anvil's Experimental Demos

All of these demos are still in the early prototype & experimental stage. Please expect bugs or other unexpected behavior.

### Gravity Compensation Demo

Navigate to the `openarm` directory and run the gravity compensation demo using leader arms:

```bash
python -m openarm.damiao.gravity --port leader_l:left --port leader_r:right
```

This command enables gravity compensation on the leader arms, allowing them to be moved freely by hand and stay in position without falling due to gravity.

For more information: `python -m openarm.damiao.gravity --help`

**Options:**

- `--port INTERFACE:POSITION`: Specify CAN interface and arm position
  - `POSITION` must be `left` or `right`
  - Can specify multiple `--port` arguments

### Leader-Follower Teleoperation Demo (requires 2 robots)

Navigate to the `openarm` directory and run the teleoperation demo using leader and follower arms:

```bash
python -m openarm.damiao.monitor -t --gravity \
  --follow leader_l:left:follower_l:left \
  --follow leader_r:right:follower_r:right
```

This command runs the leader arms with gravity compensation and makes the follower arms mirror the leader arms' movements in real time.

For more information: `python -m openarm.damiao.monitor --help`

**Options:**

- `-t, --teleop`: Enable teleoperation mode
- `--follow MASTER:POS:SLAVE:POS`: Define master-slave mappings
  - `POS` is `left` or `right` (determines arm position)
  - Mirror mode auto-enabled when positions differ (e.g., `left` master → `right` slave)
- `-g, --gravity`: Enable gravity compensation for master arms
- `-v, --velocity SPEED`: Velocity for slave motors (default: 1.0)


## Running Enactic's OpenArm Demos
The Enactic team has released several repositories and demos that work on the open arm. You can follow our guide for building and running the Enactic's software ([Using Enactic's Stack](enactic.md)), or you can refer directly to the Enactic GitHub (https://github.com/enactic/openarm).

*Please note that Enactic is unaffiliated with Anvil Robotics.*

## ☠️ Danger-Zone - Changing the Motor Configurations ☠️

> [!CAUTION]
> The following tools can flash new configuration parameters to motors. Doing any of the following may result in the motors no longer being able to effectively communicate with the linux host.

### Motor Configuration

Configure Damiao motor IDs, set zero positions, and save parameters:

```bash
# Configure motor with specific master and slave IDs
python -m openarm.damiao.configure --channel can0 --set-master 0x11 --set-slave 0x01

# Configure motor as a predefined joint (J1-J8)
python -m openarm.damiao.configure --channel can0 --set-motor J1

# Set both custom IDs and zero position, then save
python -m openarm.damiao.configure --channel can0 --set-master 0x12 --set-slave 0x02 --set-zero --save

# Configure multiple motors (requires --allow-multiple)
python -m openarm.damiao.configure --channel can0 --set-motor J3 --allow-multiple --save

# Override motor type IDs while using predefined configuration
python -m openarm.damiao.configure --channel can0 --set-motor J1 --set-master 0x20
```

**Arguments:**

- `--channel CHANNEL`: CAN channel (required, e.g., can0, can1)
- `--interface INTERFACE`: CAN interface type (default: socketcan)
- `--set-master ID`: Set master ID (hex: 0x11 or decimal: 17)
- `--set-slave ID`: Set slave ID (hex: 0x01 or decimal: 1)
- `--set-motor MOTOR`: Configure as predefined motor (J1-J8)
- `--set-zero`: Set current position as zero
- `--save`: Save parameters to motor flash memory
- `--allow-multiple`: Allow configuring multiple motors at once

**Notes:**

- When using `--set-motor`, both master and slave IDs are set from predefined configuration
- `--set-master` or `--set-slave` override the predefined values from `--set-motor`
- Motors are auto-detected on the specified channel
- The script assumes DM8009 motor type by default

### Zero Position Calibration

Set all motors to zero position automatically across all detected CAN buses:

```bash
# Set zero position for all motors on all buses
python -m openarm.damiao.set_zero

# Specify custom CAN interface (Linux only)
python -m openarm.damiao.set_zero --interface can1
```
