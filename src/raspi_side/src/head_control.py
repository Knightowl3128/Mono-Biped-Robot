#!/usr/bin/env python

from multiprocessing import Process, Queue
from std_msgs.msg import Int32
import rospy

Servo0CP = Queue()  # Servo zero current position, sent by subprocess and read by main process
Servo1CP = Queue()  # Servo one current position, sent by subprocess and read by main process
Servo0DP = Queue()  # Servo zero desired position, sent by main and read by subprocess
Servo1DP = Queue()  # Servo one desired position, sent by main and read by subprocess
Servo0S = Queue()  # Servo zero speed, sent by main and read by subprocess
Servo1S = Queue()  # Servo one speed, sent by main and read by subprocess


def P0():  # Process 0 controlles servo0
    speed = .1  # Here we set some defaults:
    _Servo0CP = 99  # by making the current position and desired position unequal,-
    _Servo0DP = 100  # we can be sure we know where the servo really is. (or will be soon)
    while True:
        time.sleep(speed)
        if Servo0CP.empty():  # Constantly update Servo0CP in case the main process needs-
            Servo0CP.put(_Servo0CP)  # to read it
        if not Servo0DP.empty():  # Constantly read read Servo0DP in case the main process-
            _Servo0DP = Servo0DP.get()  # has updated it
        if not Servo0S.empty():  # Constantly read read Servo0S in case the main process-
            _Servo0S = Servo0S.get()  # has updated it, the higher the speed value, the shorter-
            speed = .1 / _Servo0S  # the wait between loops will be, so the servo moves faster
        if _Servo0CP < _Servo0DP:  # if Servo0CP less than Servo0DP
            _Servo0CP += 1  # incriment Servo0CP up by one
            Servo0CP.put(_Servo0CP)  # move the servo that little bit
            ServoBlaster.write('0=' + str(_Servo0CP) + '\n')  #
            ServoBlaster.flush()  #
     # Upper limit


def P1():  # Process 1 controlles servo 1 using same logic as above
    speed = .1
    _Servo1CP = 99
    _Servo1DP = 100
    while True:
        time.sleep(speed)
        if Servo1CP.empty():
            Servo1CP.put(_Servo1CP)
        if not Servo1DP.empty():
            _Servo1DP = Servo1DP.get()
        if not Servo1S.empty():
            _Servo1S = Servo1S.get()
            speed = .1 / _Servo1S
        if _Servo1CP < _Servo1DP:
            _Servo1CP += 1
            Servo1CP.put(_Servo1CP)
            ServoBlaster.write('1=' + str(_Servo1CP) + '\n')
            ServoBlaster.flush()
            if not Servo1CP.empty():
                trash = Servo1CP.get()
        if _Servo1CP > _Servo1DP:
            _Servo1CP -= 1
            Servo1CP.put(_Servo1CP)
            ServoBlaster.write('1=' + str(_Servo1CP) + '\n')
            ServoBlaster.flush()
            if not Servo1CP.empty():
                trash = Servo1CP.get()
        if _Servo1CP == _Servo1DP:
            _Servo1S = 1

# ====================================================================================================

def CamRight(distance, speed):  # To move right, we are provided a distance to move and a speed to move.
    global _Servo0CP  # We Global it so  everyone is on the same page about where the servo is...
    if not Servo0CP.empty():  # Read it's current position given by the subprocess(if it's avalible)-
        _Servo0CP = Servo0CP.get()  # and set the main process global variable.
    _Servo0DP = _Servo0CP + distance  # The desired position is the current position + the distance to move.
    if _Servo0DP > _Servo0UL:  # But if you are told to move further than the servo is built go...
        _Servo0DP = _Servo0UL  # Only move AS far as the servo is built to go.
    Servo0DP.put(_Servo0DP)  # Send the new desired position to the subprocess
    Servo0S.put(speed)  # Send the new speed to the subprocess
    return;


def CamLeft(distance, speed):  # Same logic as above
    global _Servo0CP
    if not Servo0CP.empty():
        _Servo0CP = Servo0CP.get()
    _Servo0DP = _Servo0CP - distance
    if _Servo0DP < _Servo0LL:
        _Servo0DP = _Servo0LL
    Servo0DP.put(_Servo0DP)
    Servo0S.put(speed)
    return;


def CamDown(distance, speed):  # Same logic as above
    global _Servo1CP
    if not Servo1CP.empty():
        _Servo1CP = Servo1CP.get()
    _Servo1DP = _Servo1CP + distance
    if _Servo1DP > _Servo1UL:
        _Servo1DP = _Servo1UL
    Servo1DP.put(_Servo1DP)
    Servo1S.put(speed)
    return;


def CamUp(distance, speed):  # Same logic as above
    global _Servo1CP
    if not Servo1CP.empty():
        _Servo1CP = Servo1CP.get()
    _Servo1DP = _Servo1CP - distance
    if _Servo1DP < _Servo1LL:
        _Servo1DP = _Servo1LL
    Servo1DP.put(_Servo1DP)
    Servo1S.put(speed)
    return;


#

def move(msg):
	Cface = [0,0]
	pos_string = str(msg.data)
	Cface[0] = int(pos_string[1:4])
        Cface[1] = int(pos_string[4:7])
	print(Cface)

'''
	if Cface[0] != 0:  # if the Center of the face is not zero (meaning no face was found)

        if Cface[0] > 180:  # The camera is moved diffrent distances and speeds depending on how far away-
            CamLeft(5, 1)  # from the center of that axis it detects a face
        if Cface[0] > 190:  #
            CamLeft(7, 2)  #
        if Cface[0] > 200:  #
            CamLeft(9, 3)  #

        if Cface[0] < 140:  # and diffrent dirrections depending on what side of center if finds a face.
            CamRight(5, 1)
        if Cface[0] < 130:
            CamRight(7, 2)
        if Cface[0] < 120:
            CamRight(9, 3)

        if Cface[1] > 140:  # and moves diffrent servos depending on what axis we are talking about.
            CamDown(5, 1)
        if Cface[1] > 150:
            CamDown(7, 2)
        if Cface[1] > 160:
            CamDown(9, 3)

        if Cface[1] < 100:
            CamUp(5, 1)
        if Cface[1] < 90:
            CamUp(7, 2)
        if Cface[1] < 80:
            CamUp(9, 3)
	'''
rospy.init_node('head_control')	
print(1)
pos_sub = rospy.Subscriber("face_pos",Int32,move)
rospy.spin()
