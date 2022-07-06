#!/usr/bin/env python3
from asyncio import futures
from curses import raw
from distutils.archive_util import make_archive
from inspect import Parameter
from site import execsitecustomize
from cv2 import WARP_INVERSE_MAP, GaussianBlur, cvtColor, waitKey
from numpy import array, interp
import numpy
import rclpy
import numpy as np
import cv2 as cv
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
import math

class ThermalSubscriberNode(Node): 
    def __init__(self):
        super().__init__("thermal_subscriber")
        self.subscriber = self.create_subscription(Image, "thermal_image", self.dataReceivedCallback, qos_profile=10)
        self.laserSubscriber = self.create_subscription(LaserScan, "scan", self.laserReceivedCallback, qos_profile=10)
        self._markerPublisher = self.create_publisher(Marker, "tc_goal_angle", 10)
        self.imagePublisher = self.create_publisher(Image, "tc_image_devel", 10)
        self.posePublisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.twist = Twist()
        self.marker = Marker()
        self.goForward = False
        self.goToHottest = False
        self.velocity = 0
        self.iterator = 0
        self.lastCenterPixel = 0
        self.targetTempMin = 34
        self.targetTempMax = 37
        self.lastAngle = 0
        self.targetAngle = 0.0
        self.buf= []
        self.stitcher = cv.Stitcher_create()
        self.timer = self.create_timer(0.5, self.parseParams)
        self.declare_parameter('target_min_temp', 29)
        self.declare_parameter('target_max_temp', 37)
        self.declare_parameter('go_to_hottest_point', False)

        self.get_logger().info("Node started")

    def laserReceivedCallback(self, msg):
        #primitive lidar collision detection
        closest = float("inf")
        for i in range(0, 180):
            if(msg.ranges[i] != float("inf")):
                if(closest > msg.ranges[i]):
                    closest = msg.ranges[i]

        for i in range(1259, 1439):
            if(msg.ranges[i] != float("inf")):
               if(closest > msg.ranges[i]):
                    closest = msg.ranges[i]

        #self.get_logger().info(str("CLOSEST: {:.4f}".format(closest)))
        if (closest < 0.20 or closest == float("inf")):
            self.velocity = 0.0
        else:
            if(self.goForward == True):
                self.velocity = closest/4.0
                if(self.velocity > 0.3):
                    self.velocity = 0.3
            else:
                self.velocity = 0.0
        self.twist.linear.x = self.velocity
        self.posePublisher.publish(self.twist)

    def parseParams(self):
        #parameter handler
        self.targetTempMin = self.get_parameter('target_min_temp').get_parameter_value().integer_value
        self.targetTempMax = self.get_parameter('target_max_temp').get_parameter_value().integer_value
        self.goToHottest = self.get_parameter('go_to_hottest_point').get_parameter_value().bool_value
        self.get_logger().info("target min: {} \ntarget max: {}   \ngo to hottest: {}  \nvelocity:{}".format(self.targetTempMin, self.targetTempMax, self.goToHottest, self.velocity))

    def publishArrow(self, angle):
        #arrow visible in rviz, pointing at the angle of the hottest object
        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = "basic_shapes"
        self.marker.id = 0
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.scale.x = 1.0
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        
        self.marker.pose.orientation.x = math.sin(3.1415 * (angle-180)/360)
        self.marker.pose.orientation.y = math.cos(3.1415 * (angle-180)/360)
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 0.0
        
        #rotate the robot to face the hottest object
        if(abs(angle) > 15):
            self.twist.angular.z = 0.006 * -angle
            self.goForward = False
        else:
            self.twist.angular.z = 0.0
            self.goForward = True

        self._markerPublisher.publish(self.marker)

    def publishImage(self, image):
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(image)
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        self.imagePublisher.publish(msg)

    def dataReceivedCallback(self, msg): #new thermal image frame has arrived
        #OpenCV Bridge
        bridge = CvBridge()
        #normalizing input image
        raw_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image = np.zeros_like(raw_image)
        cv_image = cv.normalize(raw_image, cv_image, 0, 65535, cv.NORM_MINMAX)
        blur = np.zeros_like(raw_image)
        #blur = cv.medianBlur(cv_image, 3)
        #converting from 16bit mono to 8 bit 
        img8bit = (cv_image/256).astype("uint8")
        #creating color image that can display colored contours and markers
        color_img = cvtColor(img8bit, cv.COLOR_GRAY2RGB)

        #resizing image 3 times to increase clarity
        color_img = cv.resize(color_img, (16*9, 12*3), interpolation=cv.INTER_LINEAR)
        if(self.goToHottest == True):
            #creating a mask image that contains the hotest pixels
            ret, bin_img = cv.threshold(img8bit, 190, 255, cv.THRESH_BINARY)
            bin_img = cv.resize(bin_img, (16*9, 12*3), interpolation=cv.INTER_LINEAR)
            contours, hier = cv.findContours(bin_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            index = 0
            warmest = 0 

            #creating a mask to find group of the hotest pixels
            mask = np.zeros_like(bin_img)
            for i in range(len(contours)):
                cv.drawContours(mask, contours, i, (255,255,255), thickness=cv.FILLED)
                temp = cv.mean(color_img, mask=mask)[0]
                if(temp>warmest and len(contours[i]>3)):
                    warmest = temp
                    index = i

            #bounding rect helps finding center of the contour
            try:
                x,y,w,h = cv.boundingRect(contours[index])
                centerPixel = int(x+(w)/2)
            except:
                centerPixel = 0
        else:
            index = 0
            #ret, bin_img = cv.threshold(raw_image, self.targetTempMin*100, self.targetTempMin*100, cv.THRESH_BINARY)
            bin_img = cv.inRange(raw_image, self.targetTempMin*100, self.targetTempMax*100)
            bin_img = cv.resize(bin_img, (16*9, 12*3), interpolation=cv.INTER_LINEAR)
            bin_img = bin_img.astype("uint8")
            contours, hier = cv.findContours(bin_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            cv.normalize(bin_img, bin_img, 0, 255, cv.NORM_MINMAX)
            largestArea = 0
            for i in range(len(contours)):
                area = cv.contourArea(contours[i])
                if(area>largestArea and area < 185):
                    largestArea = area
                    index = i
            
            if(len(contours)>0):
                x,y,w,h = cv.boundingRect(contours[index])
                centerPixel = int(x+(w)/2)
                self.lastCenterPixel = centerPixel
            else:
                centerPixel = self.lastCenterPixel
        #converting pixel position to angle
        hottestAngle = -165 + centerPixel*6.875 /3

        #average the goal point
        self.buf.append(hottestAngle)
        if(len(self.buf)>=10):
            self.buf.pop(0)
        mean = 0
        for vals in self.buf:
            mean += vals
        self.targetAngle = mean/10
        if(self.iterator<10):
            self.iterator +=1
        if(self.iterator>=10):
            self.iterator = 0

        #calculating markers position based on averaged angle
        redMarker = int((165 + self.targetAngle)*3/6.875)

        self.lastAngle = hottestAngle
        cv.drawMarker(color_img, (centerPixel, 6*3), (255,0,0))
        cv.drawMarker(color_img, (redMarker, 6*3), (0,0,255))
        cv.drawContours(color_img, contours, index, (0,255,0), 1)

        self.publishImage(color_img)
        self.publishArrow(self.targetAngle)
        # hottest = 0.0
        # temps = []
        # for i in range(0,msg.width * msg.height * 2, 2):
        #     temps.append(float(msg.data[i] | msg.data[i+1]<<8)/100)
        # for i in range(0, len(temps)):
        #     if temps[i] > hottest:
        #         hottest = temps[i]
            
        # self.get_logger().info(str(temps) + "\n" + str("Hottest:  {}".format(hottest)))

def main(args=None):
    rclpy.init(args=args)
    node = ThermalSubscriberNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
