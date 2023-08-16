#!/usr/bin/env python3

# CODE REQUIRES the following ROS2 workspaces to operate:
#   Micro-XRCE-DDS-Agent
#   PX4-Autopilot
#   ws_sensor_combined

# MAKE SURE THAT MicroXRCE Agent IS RUNNING!
#   If it isn't, run "MicroXRCEAgent udp4 -p 8888" in separate terminal
#   For connecting to a real drone, run "sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600"

# Add "<depend>px4_msgs</depend>" to package.xml

# Add "px4_motor_test7 = YOUR_PACKAGE_NAME.px4_motor_test7:main" to setup.py

# !WARNING!

# NOT SAFE TO FLY WITH

# KILL SWITCH ONLY WORKS IF HELD DOWN WHILE RUNNING THIS CODE

import rclpy
import time
import getopt, sys
import numpy as np
from rclpy.node import Node
from rclpy import qos
from px4_msgs.msg import VehicleCommand

class PX4MotorTest7(Node):

    def __init__(self):

        super().__init__("px4_motor_test7")
        self.get_logger().info("PX4 Motor Test 7 Initialized.")

        grab_mt = np.zeros(6) # Creates an array with 6 elements, all equaling 0, and assigns it to grab_mt
        mt = np.zeros(6) # Creates an array with 6 elements, all equaling 0, and assigns it to mt

        # Remove 1st argument from the list of command line arguments
        argumentList = sys.argv[1:]
        
        # Options
        options = 'hm:t:'
        
        # Long options
        long_options = ["Help", "Motors_Thrust=", "Time="]

        t=0 # If you don't give t an assigned value, it may have trouble connecting
        
        try:
            # Parsing argument
            arguments, values = getopt.getopt(argumentList, options, long_options)
            
            # checking each argument
            for currentArgument, currentValue in arguments:
        
                if currentArgument in ("-h", "--Help"): # If the user inputs -h or --Help
                    print ("-h --Help   Displays help\n")
                    print ("\n-m --Motors_thrust   Used to set the thrust of each motor\n")
                    print ("            ex: -m 0.1,0.2,0.3,0.4,0.5,0.6\n")
                    print ("\n-t --Time   Time to run motors for in seconds\n")

                    
                elif currentArgument in ("-m", "--Motors_Thrust"): # If the user inputs -m or --Motors_Thrust
                    
                    grab_mt = sys.argv[2].split(',') # grabs the motor thrust values as an array of 6 strings

                    for k in range(0,6): # This for loop converts the string values from grab_mt to float values as an array in mt
                    
                        mt[k] = float(grab_mt[k])

                    for i in range(0,6): # This for loop checks if the values are from 0.0 to 1.0 and if there are 6 values

                        if mt[i] < 0.0 or mt[i] > 1.0:
                            print("Error, input range for motor thrust is from 0 to 1.")

                        if len(mt) < 6 or len(mt) > 6:

                            print("Error, must input 6 values, from 0 to 1, for motor thrust.")

                        else:

                            continue

                elif currentArgument in ("-t", "--Time"): # If the user inputs -t or --Time
                    
                    test_time = float(sys.argv[4])
                    t = test_time * 10

                
        except getopt.error as err:
            # output error, and return with an error code
            print (str(err))

        # Input delay in seconds
        time.sleep(2) # This delay allows the ROS bag that this code is meant to work with time to start up

        self.get_logger().info("Motor test started.")

        self.px4_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos.qos_profile_sensor_data)
        # If ros bag doesn't capture the vehicle command topic, try adding a custom qos profile
        #   This link might help: https://docs.ros.org/en/foxy/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html
        
        # This while loop is what runs the motors 
        while (t*10) > 0:

            self.timercallback(mt)
            t = t-1
            time.sleep(0.1)


        # Shuts off the motors exactly after t time
        self.publish_vehicle_command(310, 0.0, 1.0, 0.0, 0.0, 1101.0, 0.0, 0.0) # motor 1
        self.publish_vehicle_command(310, 0.0, 1.0, 0.0, 0.0, 1102.0, 0.0, 0.0) # motor 2
        self.publish_vehicle_command(310, 0.0, 1.0, 0.0, 0.0, 1103.0, 0.0, 0.0) # motor 3
        self.publish_vehicle_command(310, 0.0, 1.0, 0.0, 0.0, 1104.0, 0.0, 0.0) # motor 4
        self.publish_vehicle_command(310, 0.0, 1.0, 0.0, 0.0, 1105.0, 0.0, 0.0) # motor 5
        self.publish_vehicle_command(310, 0.0, 1.0, 0.0, 0.0, 1106.0, 0.0, 0.0) # motor 6
        # This isn't needed, however, without the above commands, the motors would spin a bit longer then t time




    def timercallback(self, mt):

        # Format:
        # self.publish_vehicle_command(command, param1, param2, param3, param4, param5, param6, param7)
        # self.publish_vehicle_command(310, 0.25, 1.0, 0.0, 0.0, 1103.0, 0.0, 0.0)
        # 310 is the command value for testing actuators
        # param1 is mapped to the throttle percent
        #   Ex: if you want to run motors at 25% throttle, input 0.25 for param1
        # param5 is mapped to which motor is being tested plus 1100
        #   Ex: if testing motor 3, input 1103 for param5

        self.publish_vehicle_command(310, mt[0], 1.0, 0.0, 0.0, 1101.0, 0.0, 0.0) # motor 1
        self.publish_vehicle_command(310, mt[1], 1.0, 0.0, 0.0, 1102.0, 0.0, 0.0) # motor 2
        self.publish_vehicle_command(310, mt[2], 1.0, 0.0, 0.0, 1103.0, 0.0, 0.0) # motor 3
        self.publish_vehicle_command(310, mt[3], 1.0, 0.0, 0.0, 1104.0, 0.0, 0.0) # motor 4
        self.publish_vehicle_command(310, mt[4], 1.0, 0.0, 0.0, 1105.0, 0.0, 0.0) # motor 5
        self.publish_vehicle_command(310, mt[5], 1.0, 0.0, 0.0, 1106.0, 0.0, 0.0) # motor 6


    def publish_vehicle_command(self, command, param1, param2, param3, param4, param5, param6, param7):

        msg = VehicleCommand()
        msg.param1 = param1 
        msg.param2 = param2 
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command 
        msg.target_system = 1 
        msg.target_component = 1 
        msg.source_system = 1 
        msg.source_component = 1 
        msg.from_external = True 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.px4_command_pub.publish(msg)

        # Use this link to reference the vehicle command topic:
        #   https://docs.px4.io/main/en/msg_docs/VehicleCommand.html
        # In addition to this, you could use the mavlink inspector with QGC Daily Build to log messages and convert them to csv
        #   Use this link for the mavlink message reference: https://mavlink.io/en/messages/common.html#COMMAND_LONG



def main(args=None):
    rclpy.init(args=args)
    node = PX4MotorTest7()
    rclpy.spin(node)
    rclpy.shutdown()
