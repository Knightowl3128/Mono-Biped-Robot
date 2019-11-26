#!/usr/bin/env python3
from Robot import *
from numpy import *
import rospy
from pc_side.msg import pwm_data
import socket

HOST = ''                 # Symbolic name meaning all available interfaces
PORT = 50007              # Arbitrary non-privileged port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn, addr = s.accept()
print('Connected by', addr)




body = Robot()

# creating the core and the corresponding links
core_l = Link('Centre', 0, 1, 'Left', None, 1)
l_1 = Link('Yaw', 1, 1, 'Left', 0, 2)
l_2 = Link('Shoulder_1', 2, 1, 'Left', 1, 3)
l_3 = Link('Shoulder_2', 3, 1, 'Left', 2, 4)
l_4 = Link('Elbow', 4, 1, 'Left', 3, None)

core_r = Link('Centre', 0, 1, 'Right', None, 1)
r_1 = Link('Yaw', 1, 1, 'Right', 0, 2)
r_2 = Link('Shoulder_1', 2, 1, 'Right', 1, 3)
r_3 = Link('Shoulder_2', 3, 1, 'Right', 2, 4)
r_4 = Link('Elbow', 4, 1, 'Right', 3, None)


body.add_link(core_l)
body.add_link(l_1)
body.add_link(l_2)
body.add_link(l_3)
body.add_link(l_4)

body.add_link(core_r)
body.add_link(r_1)
body.add_link(r_2)
body.add_link(r_3)
body.add_link(r_4)


#d-h parameters
core_l.dh = array([[0,   0,  64.6, 0]])
l_1.dh = array([[0,   65.98,    0,  -pi/2]])
l_2.dh = array([[0,   53.38, 0,  -pi/2]])
l_3.dh = array([[0,   0,  118.55, pi/2]])
l_4.dh = array([[0,   0,  118.55,  0]])

core_r.dh = array([[0,   0,  -64.6, 0]])
r_1.dh = array([[0,   65.98,    0,  -pi/2]])
r_2.dh = array([[0,   53.38, 0,  -pi/2]])
r_3.dh = array([[0,   0,  118.55, pi/2]])
r_4.dh = array([[0,   0,  118.55,  0]])


body.set_angle([0,-pi/2,pi/2,0,0],'Left')
body.set_angle([0,pi/2,pi/2,0,0],'Right')


final_pos = array([[-117.98, 0, -171.1]])


final_pos = array([[0, -190.225, 0]])



spline_1_l, spline_2_l, spline_3_l, spline_4_l, time_taken = body.moveit( final_pos,'Left',wing=True)
#spline_1_r, spline_2_r, spline_3_r, spline_4_r, time_taken = body.moveit(array([[0, -190.225, 0]]),'Right')
spline_1_r, spline_2_r, spline_3_r, spline_4_r, time_taken = body.moveit(final_pos,'Right',wing = True)



i = 0
path_x = []
path_y = []
path_z = []

def send_pwm(angles):
#first four are right next 4 are left
        angles = rad2deg(angles)
        angles_l = angles[0]
        angles_r = angles[1]
        msg = pwm_data()
        servo_min = 150  # Min pulse length out of 4096
        servo_max = 600  # Max pulse length out of 4096
        unit_pw = (600-150)/180
        
        wing_r_off = -5
        shoulder1_r_off = 10
        shoulder2_r_off = -10
        elbow_r_off = 0
        
        pwm_wing_r = int(150 + (wing_r_off+angles_r[1])*unit_pw)
        pwm_shoulder_1_r = int(150 + (180-(shoulder1_r_off+angles_r[2]))*unit_pw)
        pwm_shoulder_2_r = int(150 + (shoulder2_r_off-angles_r[3])*unit_pw)
        elbow_r = int(150 + (elbow_r_off+angles_r[4]+90)*unit_pw)        
        
        wing_l_off = -5
        shoulder1_l_off = -5
        shoulder2_l_off = 0
        elbow_l_off = 0
        
        pwm_wing_l = int(150 + (wing_l_off+180+angles_l[1])*unit_pw)
        pwm_shoulder_1_l = int(150 + (+180-(shoulder1_l_off+angles_l[2]))*unit_pw)
        pwm_shoulder_2_l = int(150 + ( -(shoulder2_l_off+angles_l[3])*unit_pw))
        elbow_l = int(150 + (elbow_l_off+angles_l[4]+90)*unit_pw)        
        
        msg.data = [pwm_wing_l, pwm_shoulder_1_l, pwm_shoulder_2_l, elbow_l , 
                    pwm_wing_r, pwm_shoulder_1_r, pwm_shoulder_2_r, elbow_r]
#        print(msg.data)
        pub.publish(msg)
   

rospy.init_node('upper_body_iksolver')
pub = rospy.Publisher('PWM_Data',pwm_data,queue_size = 1)
#sub = rospy.Subscriber('hand_pos',pos_data,callback)

while not rospy.is_shutdown():
#	i +=.07
	if i >= time_taken+2:
	        data = conn.recv(1024).decode('utf-16')
	        print(data)
	        final_pos[0,0] = int(data[0:3])
	        final_pos[0,1] = int(data[3:6])
	        final_pos[0,2] = int(data[6:9])
	        spline_1_l, spline_2_l, spline_3_l, spline_4_l, time_taken = body.moveit( final_pos,'Left',wing=True)
	        i = 0
	
	
	
	
	if i >= time_taken:
	        send_pwm([[0, spline_1_l(time_taken), spline_2_l(time_taken), spline_3_l(time_taken), spline_4_l(time_taken)],[0, spline_1_r(time_taken), spline_2_r(time_taken), spline_3_r(time_taken), spline_4_r(time_taken)]])
	        rospy.spin()
	        continue

	send_pwm([[0, spline_1_l(i), spline_2_l(i), spline_3_l(i), spline_4_l(i)],[0, spline_1_r(i), spline_2_r(i), spline_3_r(i), spline_4_r(i)]])

        i +=.08


