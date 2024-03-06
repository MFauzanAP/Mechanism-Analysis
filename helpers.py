import math

# Calculate the end point of a link from the given starting position, length and angle
def calculate_end_point(start, length, angle):
	return (start[0] + length*math.cos(math.radians(angle)), start[1] - length*math.sin(math.radians(angle)))

# Calculate magnitude of a vector
def magnitude(vector):
	return math.sqrt(vector[0]**2 + vector[1]**2)

# Calculate the angle of a vector
def angle(vector):
	return math.degrees(math.atan2(vector[1], vector[0]))

# Rotates a vector by the given angle (deg) clockwise
def rotate(vector, angle):
	x = vector[0] * math.cos(math.radians(-angle)) - vector[1] * math.sin(math.radians(-angle))
	y = vector[0] * math.sin(math.radians(-angle)) + vector[1] * math.cos(math.radians(-angle))
	return (x, y)