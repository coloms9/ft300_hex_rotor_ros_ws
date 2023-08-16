# QOS Setup

This guide will show you how to setup the qos settings required for ROS bag to reliably capture px4 vehicle command topic: `fmu/in/vehicle_command`.

Place the `qos_settings` folder in you home directory and... thats it! 

Now the qos settings are setup.

### Important Note

These settings are **not default** and if you want to use them, you'll have to include the following argument:
```
--qos-profile-overrides-path /home/YOUR_USERNAME/qos_settings/px4_reliability_override.yaml
```
Make sure to replace `YOUR_USERNAME` with the username of your computer.