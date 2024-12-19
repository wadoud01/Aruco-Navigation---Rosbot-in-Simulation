#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image, CameraInfo
from assignment_1.msg import Aruco_info
import imutils
import numpy as np
from opencv import detection

dict_aruco = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
prm_aruco = aruco.DetectorParameters_create()
cap = cv2.VideoCapture(0)

class camera_cv2_rosbot:
    
    def __init__(self):
        
        self.detect = detection()
        self.camera_center_x = 0
        self.camera_center_y = 0
    
        self.aruco_info_pub = rospy.Publisher('aruco_info', Aruco_info, queue_size=10) 
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_callback)

    def camera_callback(self, camera_msg):
        self.camera_center_x = camera_msg.width / 2
        self.camera_center_y = camera_msg.height / 2

    def image_callback(self, img_msg):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        (corners, ids, rejected) = aruco.detectMarkers(image, dict_aruco, parameters=prm_aruco)
        
        image = imutils.resize(image, width=1000)


        Result_cv2 = self.detect.aruco_detection(image)
        cv2.imshow("Frame", Result_cv2)
        cv2.waitKey(1)

        if ids is not None:
            
            marker_center_x = (corners[0][0][0][0] + corners[0][0][1][0] +
                                corners[0][0][2][0] + corners[0][0][3][0]) / 4
            marker_center_y = (corners[0][0][0][1] + corners[0][0][1][1] +
                                corners[0][0][2][1] + corners[0][0][3][1]) / 4            
                
            camera_center = [self.camera_center_x, self.camera_center_y]
            marker_center = [marker_center_x, marker_center_y]
            
            top_right = [corners[0][0][0][0], corners[0][0][0][1]]
            bottom_right = [corners[0][0][3][0], corners[0][0][3][1]]

            x_cord = top_right[0] - bottom_right[0]
            y_cord = top_right[1] - bottom_right[1]

            size = int(np.sqrt(np.power(x_cord, 2) + np.power(y_cord, 2)))
                        
            info_msg = Aruco_info()
                
            info_msg.id = int(ids[0][0])
            info_msg.camera_center = camera_center
            info_msg.marker_center = marker_center
            info_msg.marker_size = [size]            
            self.aruco_info_pub.publish(info_msg)
                              
        else:
            print("None")

        
    
def main():
    obc = camera_cv2_rosbot()
    rospy.init_node('aruco_detector')

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
            exit()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
