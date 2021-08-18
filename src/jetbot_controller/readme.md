# jetbot_controller

This ROS node exposes the jetbot's mobile base via ROS control. It also contains a calibration routine for open loop control.

## Parameters

  - **`i2c_bus_path`: string** - The i2c bus device to open (default: `/dev/i2c-1`).
  - **`i2c_address`: unsigned byte** - The i2c device address on the bus (default: `0x60`).

## Usage

```
roslaunch jetbot_controller node
```

## Calibration

```
rosrun jetbot_controller calibrate
```