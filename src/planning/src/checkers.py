#!/usr/bin/env python
import rospy
import tf
import sys
from ik import get_artag_location, move_checkers_piece

class CheckersGame(object):
	"""docstring for CheckersGame"""
	def __init__(self, upper_left_marker, lower_right_marker):
		super(CheckersGame, self).__init__()
		self.upper_left = get_artag_location(upper_left_marker)
		self.lower_right = get_artag_location(lower_right_marker)

	def location(self, i, j):
		left, upper, z_1 = self.upper_left
		right, lower, z_2 = self.lower_right
		return i * (right - left)/3, j * (upper - lower)/3, (z1 + z2) /2
		