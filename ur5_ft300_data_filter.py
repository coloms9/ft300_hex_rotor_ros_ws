#!/usr/bin/env python3

# If its not already there, add "<depend>geometry_msgs</depend>" and "<depend>scipy</depend>" to package.xml

# Add "ur5_ft300_data_logger = YOUR_PACKAGE_NAME.ur5_ft300_data_logger:main" to setup.py

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import numpy as np
from scipy.signal import butter,filtfilt

class UR5FT300DataFilterNode(Node):

    data_raw_fx = np.zeros(3)
    data_lpf_fx = np.zeros(3)
    data_raw_fy = np.zeros(3)
    data_lpf_fy = np.zeros(3)
    data_raw_fz = np.zeros(3)
    data_lpf_fz = np.zeros(3)

    data_raw_tx = np.zeros(3)
    data_lpf_tx = np.zeros(3)
    data_raw_ty = np.zeros(3)
    data_lpf_ty = np.zeros(3)
    data_raw_tz = np.zeros(3)
    data_lpf_tz = np.zeros(3)

    msg = WrenchStamped()
    fx1 = msg.wrench.force.x
    fy1 = msg.wrench.force.y
    fz1 = msg.wrench.force.z
    tx1 = msg.wrench.torque.x
    ty1 = msg.wrench.torque.y
    tz1 = msg.wrench.torque.z

    afx = 1.0
    afy = 1.0
    afz = 1.0
    atx = 1.0
    aty = 1.0
    atz = 1.0

    def __init__(self):
        super().__init__("ur5_ft300_data_filter")
        self.get_logger().info("UR5 FT300 sensor data filter started.")
        self.ur5_ft300_data_exp_logger_ = self.create_subscription(WrenchStamped, "/ft300_data/raw", self.publish_exp_filtered_data, 10)
        self.ur5_ft300_data_exp_filter_pub = self.create_publisher(WrenchStamped, "/ft300_data/exp_filtered", 10)
        self.ur5_ft300_data_lowpass_logger_ = self.create_subscription(WrenchStamped, "/ft300_data/raw", self.publish_lowpass_filtered_data, 10)
        self.ur5_ft300_data_lowpass_filter_pub = self.create_publisher(WrenchStamped, "/ft300_data/lowpass_filtered", 10)


    def publish_exp_filtered_data(self, data: WrenchStamped):
        t = time.time()

        fx2 = data.wrench.force.x
        fy2 = data.wrench.force.y
        fz2 = data.wrench.force.z
        tx2 = data.wrench.torque.x
        ty2 = data.wrench.torque.y
        tz2 = data.wrench.torque.z
        fdata = WrenchStamped()

        self.afx = 0.25
        self.afy = 0.25
        self.afz = 0.2
        self.atx = 0.5
        self.aty = 0.5
        self.atz = 0.6

        fdata.wrench.force.x = self.afx*fx2 + ((1-self.afx)*self.fx1)
        self.fx1 = fdata.wrench.force.x
        fdata.wrench.force.y = self.afy*fy2 + ((1-self.afy)*self.fy1)
        self.fy1 = fdata.wrench.force.y
        fdata.wrench.force.z = self.afz*fz2 + ((1-self.afz)*self.fz1)
        self.fz1 = fdata.wrench.force.z

        fdata.wrench.torque.x = self.atx*tx2 + ((1-self.atx)*self.tx1)
        self.tx1 = fdata.wrench.torque.x
        fdata.wrench.torque.y = self.aty*ty2 + ((1-self.aty)*self.ty1)
        self.ty1 = fdata.wrench.torque.y         
        fdata.wrench.torque.z = self.atz*tz2 + ((1-self.atz)*self.tz1)
        self.tz1 = fdata.wrench.torque.z

        fdata.header.stamp = self.get_clock().now().to_msg()
        
        self.ur5_ft300_data_exp_filter_pub.publish(fdata)


    def publish_lowpass_filtered_data(self, data: WrenchStamped):

        fx = data.wrench.force.x
        fy = data.wrench.force.y
        fz = data.wrench.force.z
        tx = data.wrench.torque.x
        ty = data.wrench.torque.y
        tz = data.wrench.torque.z
        fdata = WrenchStamped()

        T = 1/100 # Sampling time in seconds

        # wc is cutoff frequency
        
        fx_wc = 30
        fy_wc = 30
        fz_wc = 30
        tx_wc = 30
        ty_wc = 30
        tz_wc = 30

        # dr is dampening ratio

        fx_dr = 1
        fy_dr = 1
        fz_dr = 1
        tx_dr = 1
        ty_dr = 1
        tz_dr = 1

        self.data_raw_fx, self.data_lpf_fx = self.lpf_filt(self.data_raw_fx, self.data_lpf_fx, fx_wc, fx_dr, T, fx)
        self.data_raw_fy, self.data_lpf_fy = self.lpf_filt(self.data_raw_fy, self.data_lpf_fy, fy_wc, fy_dr, T, fy)
        self.data_raw_fz, self.data_lpf_fz = self.lpf_filt(self.data_raw_fz, self.data_lpf_fz, fz_wc, fz_dr, T, fz)
        self.data_raw_tx, self.data_lpf_tx = self.lpf_filt(self.data_raw_tx, self.data_lpf_tx, tx_wc, tx_dr, T, tx)
        self.data_raw_ty, self.data_lpf_ty = self.lpf_filt(self.data_raw_ty, self.data_lpf_ty, ty_wc, ty_dr, T, ty)
        self.data_raw_tz, self.data_lpf_tz = self.lpf_filt(self.data_raw_tz, self.data_lpf_tz, tz_wc, tz_dr, T, tz)

        fdata.wrench.force.x = self.data_lpf_fx[0]
        fdata.wrench.force.y = self.data_lpf_fy[0]
        fdata.wrench.force.z = self.data_lpf_fz[0]
        fdata.wrench.torque.x = self.data_lpf_tx[0]
        fdata.wrench.torque.y = self.data_lpf_ty[0]
        fdata.wrench.torque.z = self.data_lpf_tz[0]

        self.ur5_ft300_data_lowpass_filter_pub.publish(fdata)


    def lpf_filt(self, raw, lpf, wc, dr, T, new_sample):

        raw = [new_sample, raw[0], raw[1]]
        lpf[2] = lpf[1]
        lpf[1] = lpf[0]
        den = 1/(T*T*wc*wc+4*T*dr*wc+4)
        lpf[0] = den*(T*T*wc*wc*raw[0] + 2*T*T*wc*wc*raw[1] + T*T*wc*wc*raw[2] 
                                    - (2*T*T*wc*wc-8)*lpf[1] - (T*T*wc*wc-4*dr*T*wc+4)*lpf[2])
        return raw, lpf


def main(args=None):
    rclpy.init(args=args)
    node = UR5FT300DataFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()
