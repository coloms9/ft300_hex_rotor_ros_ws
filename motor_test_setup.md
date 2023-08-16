# Motor Test 7 Setup

This guide will show you how to setup and operate the `px4_motor_test7` code.

## !!! Warning !!!

THIS CODE IS NOT SAFE TO FLY WITH

KILL SWITCH ONLY WORKS IF HELD DOWN WHILE RUNNING THIS CODE

## Required ROS2 Workspaces

The motor test code requires the following ROS2 workspaces to operate:
   * Micro-XRCE-DDS-Agent
   * PX4-Autopilot
   * ws_sensor_combined

Use this guide to install all required software:
https://docs.px4.io/main/en/ros/ros2_comm.html#ros-2-user-guide

## Implementing the code into your ROS2 Package

When you place the motor test code into your ROS2 workspace, you'll need to add lines to 2 files: `package.xml` and `setup.py`. Here I'll specify the lines of code that you need to add.

### package.xml

Add the following lines to your dependencies in `package.xml`:
```
<depend>rclpy</depend>
<depend>time</depend>
<depend>px4_msgs</depend>
```
You probably don't need to add the `rclpy` dependency as it should already be there.

Your package file should look like this:
```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>px4_stuff</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>px4_msgs</depend>
  <depend>time</depend>
  <depend>geometry_msgs</depend>
  <depend>socket</depend>
  <depend>scipy</depend>
  <exec_depend>ros2launch</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

*Note: The dependencies below are not needed to run just this code, but if you want to also collect data, you'll need them for the data acquisition, data filtering, connecting to the ft300 sensor, and ROS launch.

```
<depend>geometry_msgs</depend>
<depend>socket</depend>
<depend>scipy</depend>
<exec_depend>ros2launch</exec_depend>
```

### setup.py

Add the following line to your `setup.py` file:

```
"px4_motor_test7 = YOUR_PACKAGE_NAME.px4_motor_test7:main",
```

When putting this line into the setup file, replace `YOUR_PACKAGE_NAME` with the name of your package.

Your setup file should look like this:

```
from setuptools import setup

package_name = 'YOUR_PACKAGE_NAME'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jj',
    maintainer_email='jj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "px4_motor_test7 = YOUR_PACKAGE_NAME.px4_motor_test7:main",
            "ur5_ft300_sensor_connect = YOUR_PACKAGE_NAME.ur5_ft300_sensor_connect:main",
            "ur5_ft300_data_filter = YOUR_PACKAGE_NAME.ur5_ft300_data_filter:main",
        ],
    },
)

```

*Note: The lines below are not needed to run just this code, but if you want to also collect data, you'll need them for the data acquisition, data filtering, and connecting to the ft300 sensor.

```
"ur5_ft_sensor = ur5_code.ur5_ft_sensor:main",
"ur5_ft300_data_filter = ur5_code.ur5_ft300_data_filter:main",
```

### Build your Workspace

In the terminal, navigate to the root of your workspace and run the following command:
```
colcon build
```
Now the motor test file is setup and ready to run.

## Running the Motor Test

### Setting up the MicroXRCE Agent

Before running motor test code, ensure that the MicroXRCE agent is running. If it isn't, input this cmd line:
```
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```
Make sure that you are running the agent through the `serial` bus and at a baud rate of `921600`.
The only part you might have to change is:
```
/dev/ttyUSB0
```
This tells the agent which USB port on your device the URXCE bridge is connected to. So depending on your setup, you might have to switch to another USB port.

### Required Arguments

In addition to the regular ROS2 arguments, the motor test code **requires** that you pass arguments m and t.

#### Argument m

Argument m stands for Motors_Thrust and is what controls the throttle of the six motors. The argument takes 6 values separated by a comma. These six values represent the percent throttle of the 6 motors, which needs to be from 0 to 1. This argument needs to come after the initial ROS2 arguments. 

An example of this argument is:
```
-m 0.1,0.1,0.1,0.1,0.1,0.1
```
The argument above is telling the motor test to run all 6 motors at 10% throttle.

#### Argument t

Argument t stands for time. It controls how long the motor test will run. The argument takes a single values representing time in seconds.This argument needs to come after the m argument.

An example of this argument is:
```
-t 3
```
The argument above is telling the motor test to run for 3 seconds.

### Command Line Inputs

To run the motor test 7 code, input the following into the terminal:
```
ros2 run YOUR_PACKAGE_NAME px4_motor_test7 -m 0.1,0.1,0.1,0.1,0.1,0.1 -t 3
```
This will run all 6 motors at 10% for 3 seconds.
