# UR5 FT300 Sensor Setup

This guide will show you how to setup and operate the `ur5_ft300_sensor_connect` and `ur5_ft300_data_filter` files.

## Implementing the code into your ROS2 Package

When you place `ur5_ft300_sensor_connect.py` and `ur5_ft300_data_filter.py` into your ROS2 workspace, you'll need to add lines to 2 files: `package.xml` and `setup.py`. Here I'll specify the lines of code that you need to add.

### Package.xml

Add the following lines to your dependencies in `package.xml`:
```
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>socket</depend>
<depend>scipy</depend>
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

*Note: The dependencies below are not needed to run just this code, but if you want to also run the drone and use ROS launch, you'll need them.

```
<depend>px4_msgs</depend>
<depend>time</depend>
<exec_depend>ros2launch</exec_depend>
```

### Setup.py

Add the following lines to your `setup.py` file:

```
"ur5_ft300_sensor_connect = YOUR_PACKAGE_NAME.ur5_ft300_sensor_connect:main" to setup.py",
"ur5_ft300_data_filter = YOUR_PACKAGE_NAME.ur5_ft300_data_filter:main",
```

When putting these lines into the setup file, replace `YOUR_PACKAGE_NAME` with the name of your package.

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

*Note: The line below is not needed to run just this code, but if you want to also run the drone, you'll need them.

```
"px4_motor_test7 = YOUR_PACKAGE_NAME.px4_motor_test7:main",
```

### Build your Workspace

In the terminal, navigate to the root of your workspace and run the following command:
```
colcon build
```

## UR5 FT300 Sensor Connect

The ur5_ft300_sensor_connect code is what connects ROS2 to the ft300 sensor on the UR5. This code creates a node that publishes the raw wrench data from the ft300 to the topic:
> /ft300_data/raw

*Note: Message type is StampedWrench from geometry_msgs

### Modifying The Code

To Connect to the ft300 sensor, you'll need to modify the code.

#### IP Address

In the code file, line 24 is where the IP address of the UR5 is located. By default, the code will have:
```
HOST = '192.168.1.9'
```
This is a place holder IP address and you'll need to find the IP address of your UR5 and input it here.

#### Port Number

In the code file, line 25 is where the port of the UR5 is located. By default, the code will have:
```
PORT = 63351
```
This is a place holder port and you'll need to find the port of your UR5 and input it here.

### Command Line Inputs

To run the ur5_ft300_sensor_connect code, input the following into the terminal:
```
ros2 run YOUR_PACKAGE_NAME ur5_ft300_sensor_connect
```

## UR5 FT300 Data Filter

The ur5_ft300_data_filter code takes the raw wrench data from the `/ft300_data/raw` topic and filters it with both an exponential and lowpass filter. It then publishes the filtered data to 2 topics:

> /ft300_data/exp_filtered
>
> /ft300_data/lowpass_filtered

*Note: Message type is StampedWrench from geometry_msgs

### Command Line Inputs

To run the ur5_ft300_data_filter code, input the following into the terminal:
```
ros2 run YOUR_PACKAGE_NAME ur5_ft300_data_filter
```

### Modifying the Code

To better tailor the data filters to your specific sensor and use case, you might consider changing the parameters for each of the data filters.

#### Exponential Filter

To tune the exponential filter, you will need to change the alpha values. They are represented as 3 letter variables. The first letter, a, in the name stands for alpha. The second letter, f or t, stands for force and torque respectively. The third letter, x,y, or z, stands for x, y, and z axis respectively. 

For example:
```
afx = 0.25
```
This variable shows that it is the alpha value (a) for force (f) in the x axis (x) anf that it is equal to 0.25.

There are 6 of these values located in lines 65 - 70. By default the values are:
```
self.afx = 0.25
self.afy = 0.25
self.afz = 0.2
self.atx = 0.5
self.aty = 0.5
self.atz = 0.6
```
Change these values to tune the exponential filter

*Note: Do not change the alpha values **without** the `self.` in front of them, only change the values above.

### Lowpass Filter

To tune the lowpass filter, you will need to change cutoff frequencies. They are represented as a 4 letter variable. The first letter, f or t, stands for force and torque respectively. The second letter, x, y, or z, stands for x, y, and z axis respectively. The last 2 letters, wc, stand for cutoff frequency. 

For example:
```
fx_wc = 30
```
This variable show that it is the cutoff frequency (wc) for the force (f) in the x axis (x) and that it is equal to 30 Hz.

There are 6 of these values located in lines 105 - 110. By default, the values are:
```
fx_wc = 30
fy_wc = 30
fz_wc = 30
tx_wc = 30
ty_wc = 30
tz_wc = 30
```
Change these values to tune the lowpass filter

*Note: It is not recommended that you change the sampling time (T) or the damping ratios (Variables that end with a dr).
