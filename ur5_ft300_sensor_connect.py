#!/usr/bin/env python3

# If its not already there, add "<depend>geometry_msgs</depend>" and "<depend>socket</depend>" to package.xml

# Add "ur5_ft300_sensor_connect = YOUR_PACKAGE_NAME.ur5_ft300_sensor_connect:main" to setup.py

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import WrenchStamped
import socket

class UR5_FT300_Sensor_Connect(Node):

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self):

        super().__init__("ur5_ft_sensor")
        self.get_logger().info("UR5 ft sensor reader started.")

        self.ft_sensor_pub = self.create_publisher(WrenchStamped, "/ft300_data/raw", 10)

        HOST = '192.168.1.9' # IP address of the ur5
        PORT = 63351 # A port of the ur5
        publish_zeros_on_error = False

        try:

            self.get_logger().info("Connecting to " + HOST)
            self.s.connect((HOST,PORT))
            self.s.settimeout(5.0)
            self.get_logger().info("Connection established")
            self.timer = self.create_timer(1/100, self.publish_ft300_data) # Runs the publish_ft300_data function at 100Hz

        except Exception as e:

            self.get_logger().info(f"No Connection: Error Message:\t {e}")

    def publish_ft300_data(self):

        try:

            data = str(self.s.recv(1024))

            if data:

                data = data.split(')')[-2]
                data = data.replace("(","")
                data = data.replace(" ","")
                data = data.replace("b'","")
                data = data.split(",")
                data = np.array([float(val) for val in data])

                msg = WrenchStamped()
                msg.wrench.force.x = data[0]
                msg.wrench.force.y = data[1]
                msg.wrench.force.z = data[2]
                msg.wrench.torque.x = data[3]
                msg.wrench.torque.y = data[4]
                msg.wrench.torque.z = data[5]

                self.ft_sensor_pub.publish(msg)

            else:

                self.get_logger().info(f'empty: {data}')

        except Exception as e:

            self.get_logger().info(f'n\nread error \n\t| {e} \n {data}\n\n')
    

def main(args=None):

    rclpy.init(args=args)
    node = UR5_FT300_Sensor_Connect()
    rclpy.spin(node)
    rclpy.shutdown()
