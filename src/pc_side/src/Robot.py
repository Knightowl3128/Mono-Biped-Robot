from numpy import *
from Link import *
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy.interpolate import Akima1DInterpolator
from mpl_toolkits.mplot3d import Axes3D
import time

class Robot(Link):

    def __init__(self):

        super(Robot, self).__init__(name=None, id=None, mass=None, direction=None, mother=None, child = None)
        self.links_l = []
        self.links_r = []
        # self.CoM = 0
        # self.ZMP

    def add_link(self, link):

        if link.direction == 'Left':
            self.links_l.append(link)
        if link.direction == 'Right':
            self.links_r.append(link)

    def set_angle(self,angles,direction):
        if direction == 'Left':
            for i, j in enumerate(self.links_l):
                j.dh[0, 0] = angles[i]
                j.q = angles[i]
        if direction == 'Right':
            for i, j in enumerate(self.links_r):
                j.dh[0, 0] = angles[i]
                j.q = angles[i]
        self.get_all_pos()

    def joint_transform_matrix(self, dh):
        theta = dh[0, 0]
        d = dh[0, 1]
        a = dh[0, 2]
        alpha = dh[0, 3]

        trans = mat([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                 [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
                 [0, sin(alpha), cos(alpha), d],
                 [0, 0, 0, 1]])

        return trans

    def get_all_pos(self):
        listnow = self.links_l
        for i in range(2):
            for index,val in enumerate(listnow):
                if index == 0:
                    val.start = array([[0,0,0,1]]).T
                    rot = array([[1, 0, 0],
                                 [0, 1, 0],
                                 [0, 0, 1],
                                 [0, 0, 0]])
                    a = concatenate([rot, val.start], axis=1)
                    val.start = array([[0,0,0]])
                    m = self.joint_transform_matrix(val.dh)
                    val.trans_mat = a * m
                    val.end = (val.trans_mat[0:3, 3]).T

                else:
                    val.start = listnow[index-1].end
                    # a = concatenate([rot, val.start], axis=1)
                    m = self.joint_transform_matrix(val.dh)
                    a = listnow[index-1].trans_mat
                    val.trans_mat = a*m
                    val.end = val.trans_mat[0:3, 3].T

            listnow = self.links_r

    def inverse_kinematics(self,pos,direction):

        pos = array(pos)
        l1 = 118.55 # upper arm length
        l2 = 118.55 # lower arm length

        if direction == 'Left':
            listnow = self.links_l
        elif direction == 'Right':
            listnow = self.links_r
        # here origin is the start of the arm
        origin = listnow[3].start
        origin = array(origin)
        l = linalg.norm(pos-origin,axis=1) # distance between tip and origin

        # x1 = pos[0,0] - origin[0,0]
        # y1 = pos[0,1] - origin[0,1]
        # z1 = pos[0,2] - origin[0,2]
        #
        # l_xz = sqrt(x1**2 + z1**2) # length in x-z plane
        # l_yz = sqrt(y1**2 + z1**2)
        # l_xy = sqrt(y1**2 + x1**2)

        off = pos - origin
        x1 = off[:,0]
        y1 = off[:,1]
        z1 = off[:,2]
        l_yz = sqrt(y1**2 + z1**2) #length in yz

        u = (l**2-l2**2+l1**2)/(2*l1)
        theta2 = arcsin(x1/u)

        h = l1*sin(theta2)
        w = sqrt(l2**2 - (h-x1)**2)

        b = l1*cos(theta2)
        cos_theta1 = (b**2 + l_yz**2 - w**2)/(2*b*l_yz)

        theta1 = pi + arctan2(z1,y1) + arccos(cos_theta1)
        cos_theta_3 = (l1 ** 2 + l2 ** 2 - l ** 2) / (2 * l1 * l2)

        theta3 = arccos(cos_theta_3) - pi
        if direction == 'Left':
            theta2 = -theta2
            final = concatenate(([theta1],[theta2],[theta3]),axis=0).T
            return final

        elif direction == 'Right':
            theta1 = -arccos(cos_theta1) - arctan2(z1,y1)
            theta3 = -theta3
            final = concatenate(([theta1], [theta2], [theta3]), axis=0).T
            return final

    def moveit(self,pos,direction,wing = True):
        # does not support yaw
        # we assume 1 while loop takes 1 millisecond
        wing_ang = deg2rad(-40)
        if direction == "Left":
            wing_ang = -wing_ang
        servo_speed = 230/(pi/3) # ms for 1 rad
        if direction == 'Left':
            listnow = self.links_l
        else:
            listnow = self.links_r
        offset = (listnow[1].start).T
        initial_angles = []
        for i in listnow:
            initial_angles.append(i.q)
        rot = array([[cos(wing_ang), -sin(wing_ang), 0],
                    [sin(wing_ang), cos(wing_ang), 0],
                     [0,       0,     1]])
        if wing:
            new_pos = pos.T - offset
            rotated = rot*new_pos
            rotated += offset
            final_angles = self.inverse_kinematics(rotated.T,direction)
        else:
            final_angles = self.inverse_kinematics(pos, direction)
        print(rad2deg(final_angles))
        initial_angles = array([initial_angles[2:5]])

        angular_dist = final_angles - initial_angles
        angular_dist = append(angular_dist,[wing_ang])

        time_taken = amax(absolute(angular_dist))*servo_speed
        t = arange(0,time_taken,0.01)
        if direction == 'Left':
            if wing:
                spline_1 = CubicSpline([0, time_taken], [-pi/2, -pi/2-wing_ang], bc_type=(((1, 0)), (1, 0)))
            else:
                spline_1 = CubicSpline([0, time_taken], [-pi / 2, -pi / 2], bc_type=(((1, 0)), (1, 0)))
            spline_2 = CubicSpline([0, time_taken], [initial_angles[0,0], final_angles[0,0]], bc_type=(((1,0)),(1,0)))
            spline_3 = CubicSpline([0, time_taken], [initial_angles[0, 1], final_angles[0, 1]], bc_type=(((1, 0)), (1, 0)))
            spline_4 = CubicSpline([0, time_taken], [initial_angles[0, 2], final_angles[0, 2]], bc_type=(((1, 0)), (1, 0)))
        elif direction == 'Right':
            if wing:
                spline_1 = CubicSpline([0, time_taken], [pi / 2, pi / 2 - wing_ang], bc_type=(((1, 0)), (1, 0)))
            else:
                spline_1 = CubicSpline([0, time_taken], [pi / 2, pi / 2], bc_type=(((1, 0)), (1, 0)))
            spline_2 = CubicSpline([0, time_taken], [initial_angles[0, 0], final_angles[0, 0]],
                                   bc_type=(((1, 0)), (1, 0)))
            spline_3 = CubicSpline([0, time_taken], [initial_angles[0, 1], final_angles[0, 1]],
                                   bc_type=(((1, 0)), (1, 0)))
            spline_4 = CubicSpline([0, time_taken], [initial_angles[0, 2], final_angles[0, 2]],
                                   bc_type=(((1, 0)), (1, 0)))



        return spline_1, spline_2, spline_3, spline_4, time_taken

    def draw_all(self,ax):
        listnow = self.links_l
        for i in range(2):
            for index,val in enumerate(listnow):
                if index == 0:

                    ax.scatter(val.start[0,0], val.start[0,1], val.start[0,2], c='red', marker='o',s = 50)
                    ax.scatter(val.end[0,0], val.end[0,1], val.end[0,2], c='black', marker='o')

                    ax.plot([val.start[0, 0], val.end[0, 0]], [val.start[0, 1], val.end[0, 1]],
                            [val.start[0, 2], val.end[0, 2]],
                            'black', lw=1)
                else:
                    ax.scatter(val.end[0,0], val.end[0,1], val.end[0,2], c='black', marker='o')

                    ax.plot([val.start[0, 0], val.end[0, 0]], [val.start[0, 1], val.end[0, 1]],
                            [val.start[0, 2], val.end[0, 2]],
                            'black', lw=1)
                    pass

            listnow = self.links_r



