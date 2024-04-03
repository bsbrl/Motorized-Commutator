import numpy as np
from math import cos, sin, radians, exp, copysign, acos, degrees, sqrt
from cmath import exp
import matplotlib.pyplot as plt
import time
from matplotlib.patches import FancyArrowPatch


def vector(magnitude, ang):
	# The angle should be supplied in DEGREES
	# The result is in the RECTANGULAR form
	return magnitude * exp(complex(0, radians(ang)))


def exponential(ang):
	# The angle should be in DEGREES
	return exp(complex(0, radians(ang)))


def mag(x, y):
	# Calculated the MAGNITUDE of a vector
	return np.sqrt((x ** 2) + (y ** 2))


def angle(x, y):
	# Calculates the ANGLE of a vector
	# with respect to the positive x-axis
	
	if y < 0 and x < 0:
		return (-1 * np.deg2rad(180)) + np.arctan(y / x)
	
	if y < 0 and x > 0:
		return 1 * np.arctan(y / x)
	
	if y > 0 and x < 0:
		return np.deg2rad(180) + np.arctan(y / x)
	
	return np.arctan(y / x)


def vectorAngle(v1, v2):
	# Calculate the angle between two vectors
	# Using the dot product rule
	
	unit_v1 = v1 / np.linalg.norm(v1)
	unit_v2 = v2 / np.linalg.norm(v2)
	dot_product = np.dot(unit_v1, unit_v2)
	
	# Use cross product to get the direction of rotation
	cross_product = np.cross(v1, v2)
	multiplier = copysign(1, cross_product)
	
	# if abs(dot_product) > 0.99:
	# 	return multiplier
	
	return multiplier * np.arccos(dot_product)


def vectorAngle_v2(v1, v2):
	dot_product = v1[0] * v2[0] + v1[1] * v2[1]
	magnitude_v1 = sqrt(v1[0] ** 2 + v1[1] ** 2)
	magnitude_v2 = sqrt(v2[0] ** 2 + v2[1] ** 2)
	
	cos_angle = dot_product / (magnitude_v1 * magnitude_v2)
	# debug_print(True, "Cos angle: " + str(cos_angle))
	
	if cos_angle > 0.9999999:
		cos_angle = 1
	
	angle_radians = acos(cos_angle)
	# angle_degrees = degrees(angle_radians)
	
	# Use cross product to get the direction of rotation
	cross_product = np.cross(v1, v2)
	multiplier = copysign(1, cross_product)
	
	return angle_radians * multiplier


def animate_donut(fig_axis, angle, rotations, palette, font, direction=True):
	# Convert rotations to percentage for plotting
	percent = abs(rotations * (100 / 360))
	percent = round(percent, 2)
	
	# Calculate donut plot direction
	if rotations < 0:
		direction = False
	
	# Clear plot axis before plotting
	plt.cla()
	
	# Draw donut plot
	fig_axis.pie([percent, (100 - percent)], wedgeprops={'width': 0.3}, startangle=90, colors=palette,
				 counterclock=direction)
	
	# Format plot
	plt.title('Mouse Rotation\nBetween Frames', fontsize=12, loc='center', color='#587686', **font)
	plt.text(0, 0, (str(round(rotations, 2)) + "\u00B0"), ha='center', va='center', fontsize=42, color='#587686',
			 **font)
	plt.text(0, 1.05, " 0\u00B0\n|", ha='center', fontsize=12, color='#587686', **font)
	
	baseline_text = "Angular Change: " + str(round(np.rad2deg(angle), 2)) + "\u00B0"
	plt.text(0, -1.2, baseline_text, ha='center', va='bottom', fontsize=12, color='#587686', **font)
	
	# Display plot
	# plt.tight_layout()
	plt.pause(0.5)


# plt.show()


def arrow_plot(fig_axis, angle):
	z = np.e ** (2 * np.pi * 1j * angle / 360)  # Imaginary numbers in python is not i but j
	# print("Angle is: ", angle, "Z is: ", z)
	
	# plt.figure(figsize=(6, 6))  # Size of Graph
	
	# Clear plot axis before plotting
	#  plt.cla()
	
	# Try fancy arrow patch
	arrow = FancyArrowPatch(posA=(0, 0), posB=(np.imag(z), np.real(z)),
							arrowstyle='-|>', mutation_scale=20,
							shrinkA=0, shrinkB=0, color='y', linewidth=5)
	plt.gca().add_patch(arrow)
	
	# x = np.linspace(0, 2 * np.pi, 100)  # code for circle (also best loop ever)
	# plt.plot(np.cos(x), np.sin(x), 'b', linewidth=5)  # code for circle
	
	plt.grid(True, alpha=.5)
	plt.style.use('dark_background')  # background
	
	plt.pause(0.1)


# plt.show()


def donut_arrow(fig_axis, angle, rotations, palette, font, direction=True):
	# Calculate donut plot direction
	if rotations < 0:
		direction = False
	
	# Clear plot axis before plotting
	plt.cla()
	
	# Draw donut plot
	fig_axis.pie([0, 100], wedgeprops={'width': 0.95}, startangle=90, colors=palette)
	
	# Format plot
	plt.title('Mouse Rotation\nBetween Frames', fontsize=12, loc='center', color='#587686', **font)
	# plt.text(0, 0, (str(round(rotations, 2)) + "\u00B0"), ha='center', va='center', fontsize=42, color='#587686',
	# 		 **font)
	plt.text(0, 1.05, " 0\u00B0\n|", ha='center', fontsize=12, color='#587686', **font)
	
	baseline_text = "Angular Change: " + str(round(np.rad2deg(angle), 2)) + "\u00B0"
	plt.text(0, -1.2, baseline_text, ha='center', va='bottom', fontsize=12, color='#587686', **font)
	
	# Try fancy arrow patch
	z = np.e ** (2 * np.pi * 1j * rotations / 360)  # Imaginary numbers in python is not i but j
	arrow = FancyArrowPatch(posA=(0, 0), posB=(np.imag(z), np.real(z)),
							arrowstyle='-|>', mutation_scale=20,
							shrinkA=0, shrinkB=0, color='#6F58FF', linewidth=5)
	plt.gca().add_patch(arrow)
	
	# Display plot
	# plt.tight_layout()
	plt.pause(0.05)


def rotations_over_time(fig_axis, rotations, time):
	fig_axis.plot(time, rotations)


def write_read(x, mcu):
	"""Function for writing data to serial
	And get a response of the data received."""
	
	# Convert to Arduino filter mode
	# x = "<" + x + ">"
	
	print("\nValue being sent to MCU: ", x)
	mcu.write(bytes(x, 'utf-8'))
	time.sleep(0.01)			# Changed from 0.05 | 3 Oct 2023
	# data = mcu.readline()
	# return data
	return 0


def read_only(arduino):
	"""Function to read serial data from Arduino."""
	
	data = arduino.readline()
	return data


def write_only(x, arduino):
	# Function for writing data to Arduino serial
	
	arduino.write(bytes(x, 'utf-8'))
	# time.sleep(1)
	return True


def debug_print(condition, message):
	# This function prints out debug messages depending on the condition
	if condition:
		print(message)


def check_datatype(meso, tailbase):
	# This function checks the pose values for nans
	
    # Split supplied positions into individual values
	chk_mt_0 = (str(meso[0].dtype) == 'float32')
	chk_mt_1 = (str(meso[1].dtype) == 'float32')
	chk_nk_0 = (str(tailbase[0].dtype) == 'float32')
	chk_nk_1 = (str(tailbase[1].dtype) == 'float32')
	
	# print("\nInside debug: ", str(tailbase[1].dtype))
	#
	# print("Data Types: ")
	# print(str(meso[0].dtype))
	# print(str(meso[1].dtype))
	# print(str(tailbase[0].dtype))
	# print(str(tailbase[1].dtype))
	# print("")
	
	return chk_nk_0 and chk_nk_1 and chk_mt_0 and chk_mt_1


def radius_filter(point1, point2, radius):
	"""Checks if a 2D point is within a radius of another 2D point.
	
	Args:
	point1: The first 2D point.
	point2: The second 2D point.
	radius: The radius.
	
	Returns:
	True if the first point is within the radius of the second point, False otherwise.
	"""
	
	# Calculate the distance between the two points.
	distance = sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
	# debug_print(True, "Distance within last point with filter radius: " + str(distance) + " pixels.")
	
	# If the distance is less than or equal to the radius, then the first point is within the radius of the second point.
	if distance <= radius:
		return True
	else:
		return False
