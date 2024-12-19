# ! /usr/bin/env python3

import imutils
import cv2
import math

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

arucoParams = cv2.aruco.DetectorParameters_create()
cap = cv2.VideoCapture()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

class detection():
    
    def aruco_detection(self, image):

        self.image = image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        image = imutils.resize(image, width=1000)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image,
                                                           arucoDict, parameters=arucoParams)
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
            print("[Inference] ArUco marker ID: {}".format(markerID))

        return image
               
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            exit()

    

