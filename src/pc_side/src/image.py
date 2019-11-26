#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

frontalface = cv2.CascadeClassifier('/home/navaneeth/raspi/src/pc_side/src/haarcascade_frontalface_default.xml') 
profileface = cv2.CascadeClassifier('/home/navaneeth/raspi/src/pc_side/src/haarcascade_profileface.xml')

face = [0, 0, 0, 0]  
Cface = [0, 0]  
lastface = 0 

def rotate_bound(image, angle):
    # grab the dimensions of the image and then determine the
    # center
    (h, w) = image.shape[:2]
    (cX, cY) = (w // 2, h // 2)

    # grab the rotation matrix (applying the negative of the
    # angle to rotate clockwise), then grab the sine and cosine
    # (i.e., the rotation components of the matrix)
    M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])

    # compute the new bounding dimensions of the image
    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))

    # adjust the rotation matrix to take into account translation
    M[0, 2] += (nW / 2) - cX
    M[1, 2] += (nH / 2) - cY

    # perform the actual rotation and return the image
    return cv2.warpAffine(image, M, (nW, nH))



class image_converter:

  def __init__(self):
    self.pos_pub = rospy.Publisher("face_pos",Int32,queue_size = 1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/image_decompressed",Image,self.callback)

  def callback(self,data):
    global lastface, Cface, face
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    faceFound = False  

    if not faceFound:
        if lastface == 0 or lastface == 1:
            fface = frontalface.detectMultiScale(cv_image, scaleFactor=1.2, minNeighbors=5);
            if fface != ():  
                lastface = 1  
                for f in fface:  
                    faceFound = True
                    face = f

    if not faceFound: 
        if lastface == 0 or lastface == 2:  
            pfacer = profileface.detectMultiScale(cv_image, scaleFactor=1.2, minNeighbors=5)
            if pfacer != ():  
                lastface = 2
                for f in pfacer:
                    faceFound = True
                    face = f

    if not faceFound: 
        lastface = 0  
        face = [0, 0, 0, 0]  

    x, y, w, h = face
    Cface = [int(w / 2 + x), int(h / 2 + y)]
    a = "{0:0=3d}".format(Cface[0])
    self.pos_pub.publish(int('1'+"{0:0=3d}".format(Cface[0])+"{0:0=3d}".format(Cface[1])))
    

def main(args):
  ic = image_converter()
  rospy.init_node('face_detect')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
