#!/usr/bin/env python

import rospy

from std_msgs.msg import Int32

rospy.init_node('user_input')

pub_camera = rospy.Publisher('camera_status',Int32,latch = True,queue_size = 10)

while not rospy.is_shutdown():
	
	usr_cam = int(input('Change Camera Status:\n1. Camera ON \n2. Camera OFF\n'))
	if usr_cam == 1:
		pub_camera.publish(1)
		print('\nCamera Status is ON\n\n\n\n')
	elif usr_cam  == 2:
		pub_camera.publish(0)
		print('Camera Status is OFF\n\n\n\n')
	else:
		print('\nEnter 1 or 2\n')
		continue
	


