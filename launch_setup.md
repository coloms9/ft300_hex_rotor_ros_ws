# Launch Setup

This guide will show you how to setup and operate the `motor_test_and_bag_launch.py` and `ur5_ft300_data_launch.py` files.

## !!! Important !!!

Before going through this guide, make you have completed the `motor_test_setup.md`, `ur5_ft300_sensor_setup.md`, and `qos_setup.md` guides.

If you haven't finished those guides, **none** of the code in this guide will work.

## Creating Launch Folder

Unlike regular ROS 2 files, launch files need to be put into a separate folder location outside of your package, at the root of your workspace. Here I will show you how to create the launch folder.

In the terminal, navigate to the root of your ROS 2 workspace and input the following command:

```
mkdir launch
```

This command will create a folder named `launch` located at the root of your ROS 2 workspace. 

After you created this folder, place both `motor_test_and_bag_launch.py` and `ur5_ft300_data_launch.py` into it.

## Implementing the code into your ROS2 Package

When you place `motor_test_and_bag_launch.py` and `ur5_ft300_data_launch.py` into your ROS2 workspace, you'll need to add lines to `package.xml`. Here I'll specify the lines of code that you need to add.

Add the following lines to your dependencies in `package.xml`:
```
<depend>rclpy</depend>
<exec_depend>ros2launch</exec_depend>
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

*Note: The dependencies below are needed to run the drone and ur5 code.

```
<depend>px4_msgs</depend>
<depend>time</depend>
<depend>geometry_msgs</depend>
<depend>socket</depend>
<depend>scipy</depend>
```

### Build your Workspace

In the terminal, navigate to the root of your workspace and run the following command:
```
colcon build
```
Now the launch files are setup and ready to run, **assuming you have all the other files setup already**.

## UR5 FT300 Data Launch

The `ur5_ft300_data_launch.py` file is used to run both the `ur5_ft300_sensor_connect` and `ur5_ft300_data_filter` files at the same time.

### Modifying Code

In the launch file, you'll have to modify the package name so that it can run the `ur5_ft300_sensor_connect` and `ur5_ft300_data_filter` code.

There are 2 lines that need to change. They are located in the `return` section of the file as shown here:
```
return LaunchDescription([
        
        Node(
            package = 'ur5_code',
            executable = 'ur5_ft_sensor',
        ),

        Node(
            package = 'ur5_code',
            executable='ur5_ft300_data_filter',
        )

    ])
```
Change the package names for **both** of the nodes from `ur5_code` to the name of the package where the `ur5_ft300_sensor_connect.py` and `ur5_ft300_data_filter.py` files are stored.

#### Build your Workspace

For these changes to take effect, you'll need to build the workspace

In the terminal, navigate to the root of your workspace and run the following command:
```
colcon build
```

### How to use Launch File

To run `ur5_ft300_data_launch.py`, you'll first need to use the terminal to navigate to the launch folder at the root of your ROS 2 workspace. 

Once you are in the launch folder, input the following command:
```
ros2 launch ur5_ft300_data_launch.py
```

Both the `ur5_ft300_sensor_connect` and `ur5_ft300_data_filter` code should now be running

## Motor Test and Bag Launch

The `motor_test_and_bag_launch.py` file is used to run the `px4_motor_test7.py` and create a `ROS bag` that records data from the drone and UR5 ft300 sensor.

### !!! Warning !!!

THIS CODE IS NOT SAFE TO FLY WITH

KILL SWITCH ONLY WORKS IF HELD DOWN WHILE RUNNING THIS CODE

### Modifying Code

In the launch file, you'll have to modify the package name so that it can run the `px4_motor_test7.py` code and create the ROS bag.

There are 2 sections that need modification, the `motor test 7 launch` and `ROS bag command`.

#### Motor Test 7 Launch Modifications

Starting on line 29 in `motor_test_and_bag_launch.py` you will see the following:

```
    start_motor_test = ExecuteProcess(
        cmd = [[
            'ros2 ',
            'run ',
            'px4_stuff ',
            'px4_motor_test7 '
            '-m ',
            m,
            ' -t ',
            ti
        ]],
        shell=True
    )
```
Change the name of the package by replacing `px4_stuff` with the name of the package where the `px4_motor_test7.py` file is stored. REMEMBER TO LEAVE A SPACE AFTER THE NAME OF YOUR PACKAGE.

#### ROS Bag Command Modifications

Starting on line 43 in `motor_test_and_bag_launch.py` you will see the following:
```
    start_ros_bag = ExecuteProcess(
        cmd = [[
            'ros2 ',
            'bag ',
            'record ',
            '-o ',
            f,
            ' /ft300_data/raw /ft300_data/exp_filtered /ft300_data/lowpass_filtered '
            'fmu/in/vehicle_command ',
            '--qos-profile-overrides-path /home/jj/qos_settings/px4_reliability_override.yaml',
        ]],
        shell=True
    )
```
Change the path of the qos settings from `/home/jj/qos_settings/px4_reliability_override.yaml` to `/home/YOUR_USERNAME/qos_settings/px4_reliability_override.yaml`, replacing `YOUR_USERNAME` with the username of your computer.

#### Build your Workspace

For these changes to take effect, you'll need to build the workspace

In the terminal, navigate to the root of your workspace and run the following command:
```
colcon build
```

### Required Arguments

In addition to the regular ROS2 launch arguments, the this launch file **requires** that you pass arguments motor_thrust, test_time, and file_location.

#### Argument `motors_thrust`

The argument `motors_thrust` is what controls the throttle of the six motors. The argument takes 6 values separated by a comma. These six values represent the percent throttle of the 6 motors, which needs to be from 0 to 1.

An example of this argument is:
```
motor_thrust:='0.1,0.1,0.1,0.1,0.1,0.1'
```
The argument above is telling the motor test to run all 6 motors at 10% throttle. 

`motor_thrust` default argument is `0,0,0,0,0,0`. You can change this default value in line 16 of `motor_test_and_bag_launch.py`

#### Argument `test_time`

The argument `test_time` controls how long the motor test will run. The argument takes a single values representing time in seconds.

An example of this argument is:
```
test_time:='3'
```
The argument above is telling the motor test to run for 3 seconds.

`test_time` default argument is `1.5`. You can change this default value in line 21 of `motor_test_and_bag_launch.py`

#### Argument `file_location`

The argument `file_location` controls where the data from the ROS bag will be located. The argument takes a file path in the form of a string.

An example of this argument is:
```
file_location:='/home/user/ur5_hex_copter_test1'
```
The argument above is telling the ROS bag to store the data collected into the folder `ur5_hex_copter_test1`.

`file_location` default argument is `~/ur5_hex_copter_data_store/test_default`. You can change this default value in line 26 of `motor_test_and_bag_launch.py`

### How to use Launch File

To run `motor_test_and_bag_launch.py`, you'll first need to use the terminal to navigate to the launch folder at the root of your ROS 2 workspace. Once you are in the launch folder, input the following commands: 
```
ros2 launch ur5_ft300_data_launch.py
```
```
ros2 launch motor_test_and_bag_launch.py
```

*Note: You'll want to launch `ur5_ft300_data_launch.py` first in order for the ROS bag to capture data from the ft300 sensor.

An example of passing arguments with this launch file would be:
```
ros2 launch motor_test_and_bag_launch.py motor_thrust:='0.1,0.1,0.1,0.1,0.1,0.1' test_time:='3' file_location:='/home/user/ur5_hex_copter_test1'
```
The command above will run the all `6 motors at 10%` for `3` seconds while logging UR5 ft300 and drone data and storing it in the folder `ur5_hex_copter_test1`.

*Note: You'll need to kill the launch code with crtl+c for the ROS bag to stop collecting data and publish the data to the specified folder.

### How to Visualize ROS Bag Data

To visualize the wrench data from the ROS bag you'll need plotjuggler. 

In the terminal, input the following:
```
sudo snap install plotjuggler
```
Plotjuggler should now be installed. To run plotjuggler, input the following:
```
ros2 run plotjuggler plotjuggler
```

When you first run plotjuggler, it should look like this:
![Alt text](<Screenshot from 2023-08-15 15-32-56.png>)

To import data from a ROS bag, click the button in the top left corner next to "**Data**". Then navigate to your ROS bag and double click the yaml file. 

Here is an example:
![Alt text](<Screenshot from 2023-08-03 15-00-23.png>)
*Note: This bag is from a test that ran all 6 motors at 100% for a minute


Original plotjuggler installation instructions here: https://index.ros.org/p/plotjuggler/

