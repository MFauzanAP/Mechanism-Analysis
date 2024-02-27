import math
from numpy import linspace

# Calculate theta 2 and 3 from the given input angle
def calculate_position(a, b, c, d, position, theta1):

	# Calculate k-constants
	K1 = d / a
	K2 = d / c
	K3 = (a**2 - b**2 + c**2 + d**2) / (2*a*c)
	K4 = d/b
	K5 = (c**2 - d**2 - a**2 - b**2) / (2*a*b)

	# Calculate alphabet constants
	A = math.cos(theta1) - K1 - K2*math.cos(theta1) + K3
	B = -2*math.sin(theta1)
	C = K1 - (K2 + 1)*math.cos(theta1) + K3
	D = math.cos(theta1) - K1 + K4*math.cos(theta1) + K5
	E = -2*math.sin(theta1)
	F = K1 + (K4 - 1)*math.cos(theta1) + K5

	# Decide if we should add or subtract based on the position
	position_modifier = 1 if position == "open" else -1

	# Calculate theta 2 and 3
	theta2 = 2*math.atan((-B + position_modifier*math.sqrt(B**2 - 4*A*C)) / (2*A))
	theta3 = 2*math.atan((-E + position_modifier*math.sqrt(E**2 - 4*D*F)) / (2*D))

	# Return the calculated angles
	return (theta2, theta3)

# Calculate and aggregate position, velocity, and acceleration analysis results for the given input angle
def calculate_results(a, b, c, d, position, theta1):

	# Convert to radians
	theta1 = math.radians(theta1)

	# Solve for position
	(theta2, theta3) = calculate_position(a, b, c, d, position, theta1)

	# Return the calculated angles
	return (theta1, theta2, theta3)

# Loops through all possible angles, calculates the results, and aggregates them into arrays
def analyze_mechanism(a, b, c, d, resolution=360, position="OPEN", convert=True):

	# Output arrays
	theta1_array = []
	theta2_array = []
	theta3_array = []

	# Loop through all possible angles
	for theta1 in linspace(1, 360, num=resolution):

		# Calculate the results
		(theta1, theta2, theta3) = calculate_results(a, b, c, d, position, theta1)

		# Convert units if specified
		if convert:
			theta1 = math.degrees(theta1)
			theta2 = math.degrees(theta2)
			theta3 = math.degrees(theta3)

		# Append to the output arrays
		theta1_array.append(theta1)
		theta2_array.append(theta2)
		theta3_array.append(theta3)

	# Return the results
	return (theta1_array, theta2_array, theta3_array)