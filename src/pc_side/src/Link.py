import numpy as np


class Link:

	def __init__(self, name, id, mass, direction, mother, child):
		self.id = id
		self.mother = mother  # Mother ID
		self.direction = direction
		self.name = name  # Name of this link
		self.mass = mass  # Mass of this link
		self.trans_mat = None
		self.child = child  # Child ID of this link
		self.dh = np.zeros((1, 4))  # DH parameter of this link
		self.q = self.dh[0, 0]  # Angle of link joint
		self.dq = 0  # Joint velocity  [rad/s]
		self.ddq = 0  # Joint acceleration [rad/s^2]
		self.c = np.zeros((3, 1))  # Position of the center of gravity [m]
		self.I = np.zeros((3, 3))  # Inertia tensor of the center of gravity around [kg.m^2]
		self.p = np.zeros((3, 1))  # Position of this link
		self.R = np.zeros((3, 1))  # Orientation of this link
		self.rot = None  # this argument using to convert self.R to axis angle
		self.a = None  # Axis angle of link
		self.servo_angleoffset = 0
		self.pin_num = 0

		"""Data for 3D simulation"""
		self.start = None
		self.end = None



