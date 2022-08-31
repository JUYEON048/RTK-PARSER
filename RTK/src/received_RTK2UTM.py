#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial
import pynmea2
from pyproj import Proj, transform
import csv
import time
import rospy
from mi_msgs.msg import *

class RTK_Class(object):
    def __init__(self):
        port = "/dev/ttyUSB1" #IONIQ차량에서 사용할 때는 serial port "/dev/ttyRTK"로 지정
        self.serialPort = serial.Serial(port, baudrate = 38400, timeout = 0.1) #115200

        self.rtk_msg = RTK()  # 사전에 정의된 RTK타입의 message 선언(RTK.msg 경로: ~/mi_msgs/msg/RTK.msg)
        self.RTK_msg_pub = rospy.Publisher('RTK_messages', RTK, queue_size=1)

        self.proj_WGS84 = Proj("+proj=latlong +datum=WGS84 +ellps=WGS84") # WGS84좌표계 pyproj
        self.proj_UTM = Proj("+proj=utm +zone=52 +ellps=WGS84 +datum=WGS84 +units=m +no_defs")

    def parsing(self, event=None):

        RTK_NMEA = self.serialPort.readline() #Serial Data read
        rtk_mnea = RTK_NMEA.decode("utf-8", "ignore") #Transform bit data to string

        
        if 'RMC' in rtk_mnea: #GNGGA DATA 사용
            if len(rtk_mnea) > 120:
                pass
        
        
            else:
                print(rtk_mnea)
                split_com = rtk_mnea.split(',')
                gnprmc = split_com[-14].split("$")[-1]
                data = ",".join(split_com[-13:])
                
                split_rmc =f"${gnprmc},{data}"
                
                
                print(split_rmc)
                print("-"*10)
                if len(split_rmc) < 10:
                    pass   
                
                else:
                    msg = pynmea2.parse(split_rmc)
            
            
                    # nmea to wgs84
                    lat = (float(msg.lat[2:])/60) + float(msg.lat[:2]) 
                    lon = (float(msg.lon[3:])/60) + float(msg.lon[0:3])
            
                    # wgs84 to UTM
                    self.rtk_msg.utm_x, self.rtk_msg.utm_y = transform(self.proj_WGS84, self.proj_UTM, lon, lat) #Transform WGS84 to UTM
            
                    self.rtk_msg.heading = msg.data[7]
                    self.rtk_msg.header.stamp = rospy.Time.now()
                    self.rtk_msg.header.frame_id = "rtk"

                    self.RTK_msg_pub.publish(self.rtk_msg)

        else:
            pass


if __name__=='__main__':
    rospy.init_node('RTK_parser')
    rc = RTK_Class()
    rospy.Timer(rospy.Duration(1.0/20.0), rc.parsing)
    rospy.spin()
    
    




