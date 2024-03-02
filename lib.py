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
	theta2 = 2*math.atan((-E + position_modifier*math.sqrt(E**2 - 4*D*F)) / (2*D))
	theta3 = 2*math.atan((-B + position_modifier*math.sqrt(B**2 - 4*A*C)) / (2*A))

	# Return the calculated angles
	return (theta2, theta3)

# Calculate omega 2 and 3, and linear velocity A, B, and BA
def calculate_velocity(a, b, c, theta1, omega1, theta2, theta3):

	# Calculate omega 2 and 3
	omega2 = omega1 * (a / b) * (math.sin(theta3 - theta1) / math.sin(theta2 - theta3))
	omega3 = omega1 * (a / c) * (math.sin(theta1 - theta2) / math.sin(theta3 - theta2))

	# Calculate linear velocity A, B, and BA
	velocityA = (a * omega1 * -math.sin(theta1), a * omega1 * math.cos(theta1))
	velocityBA = (b * omega2 * -math.sin(theta2), b * omega2 * math.cos(theta2))
	velocityB = (c * omega3 * -math.sin(theta3), c * omega3 * math.cos(theta3))

	# Return the calculated velocities
	return (omega2, omega3, velocityA, velocityB, velocityBA)

# Calculate alpha 2 and 3, and linear acceleration A, B, and BA
def calculate_acceleration(a, b, c, theta1, omega1, alpha1, theta2, theta3, omega2, omega3):

	# Calculate alphabetic constants
	A = c * math.sin(theta3)
	B = b * math.sin(theta2)
	C = a*(alpha1*math.sin(theta1) + omega1**2*math.cos(theta1)) + b*omega2**2*math.cos(theta2) - c*omega3**2*math.cos(theta3)
	D = c * math.cos(theta3)
	E = b * math.cos(theta2)
	F = a*(alpha1*math.cos(theta1) - omega1**2*math.sin(theta1)) - b*omega2**2*math.sin(theta2) + c*omega3**2*math.sin(theta3)

	# Calculate alpha 2 and 3
	alpha2 = (C*D - A*F) / (A*E - B*D)
	alpha3 = (C*E - B*F) / (A*E - B*D)

	# Calculate linear acceleration A, B, and BA
	accelerationA = (-a*alpha1*math.sin(theta1) - a*omega1**2*math.cos(theta1), a*alpha1*math.cos(theta1) - a*omega1**2*math.sin(theta1))
	accelerationBA = (-b*alpha2 *math.sin(theta2) - b*omega2**2*math.cos(theta2), b*alpha2*math.cos(theta2) - b*omega2**2*math.sin(theta2))
	accelerationB = (-c*alpha3*math.sin(theta3) - c*omega3**2*math.cos(theta3), c*alpha3*math.cos(theta3) - c*omega3**2*math.sin(theta3))

	# Return the calculated accelerations
	return (alpha2, alpha3, accelerationA, accelerationB, accelerationBA)

# Calculate and aggregate position, velocity, and acceleration analysis results for the given input angle
def calculate_results(a, b, c, d, position, theta1, omega1, alpha1):

	# Convert to radians
	theta1 = math.radians(theta1)

	# Solve for position
	(theta2, theta3) = calculate_position(
		a=a,
		b=b,
		c=c,
		d=d,
		position=position,
		theta1=theta1,
	)

	# Solve for velocity
	(omega2, omega3, velocityA, velocityB, velocityBA) = calculate_velocity(
		a=a,
		b=b,
		c=c,
		theta1=theta1,
		omega1=omega1,
		theta2=theta2,
		theta3=theta3,
	)

	# Solve for acceleration
	(alpha2, alpha3, accelerationA, accelerationB, accelerationBA) = calculate_acceleration(
		a=a,
		b=b,
		c=c,
		theta1=theta1,
		omega1=omega1,
		alpha1=alpha1,
		theta2=theta2,
		theta3=theta3,
		omega2=omega2,
		omega3=omega3,
	)

	# Return the calculated angles
	return (
		theta1,
		theta2,
		theta3,
		omega2,
		omega3,
		alpha2,
		alpha3,
		velocityA,
		velocityB,
		velocityBA,
		accelerationA,
		accelerationB,
		accelerationBA,
	)

# Loops through all possible angles, calculates the results, and aggregates them into arrays
def analyze_mechanism(a, b, c, d, omega1, alpha1, resolution=360, position="OPEN", convert=True):

	# Output arrays
	theta1_array = []
	theta2_array = []
	theta3_array = []
	omega2_array = []
	omega3_array = []
	alpha2_array = []
	alpha3_array = []
	velocityA_array = []
	velocityB_array = []
	velocityBA_array = []
	accelerationA_array = []
	accelerationB_array = []
	accelerationBA_array = []

	# Loop through all possible angles
	for theta1 in linspace(1, 360, num=resolution):

		# Calculate the results
		(
			theta1,
			theta2,
			theta3,
			omega2,
			omega3,
			alpha2,
			alpha3,
			velocityA,
			velocityB,
			velocityBA,
			accelerationA,
			accelerationB,
			accelerationBA,
		) = calculate_results(a, b, c, d, position, theta1, omega1, alpha1)

		# Convert units if specified
		if convert:
			theta1 = math.degrees(theta1)
			theta2 = math.degrees(theta2)
			theta3 = math.degrees(theta3)
			omega2 = math.degrees(omega2)
			omega3 = math.degrees(omega3)
			alpha2 = math.degrees(alpha2)
			alpha3 = math.degrees(alpha3)

		# Append to the output arrays
		theta1_array.append(theta1)
		theta2_array.append(theta2)
		theta3_array.append(theta3)
		omega2_array.append(omega2)
		omega3_array.append(omega3)
		alpha2_array.append(alpha2)
		alpha3_array.append(alpha3)
		velocityA_array.append(velocityA)
		velocityB_array.append(velocityB)
		velocityBA_array.append(velocityBA)
		accelerationA_array.append(accelerationA)
		accelerationB_array.append(accelerationB)
		accelerationBA_array.append(accelerationBA)

	# Return the results
	return (
		theta1_array,
		theta2_array,
		theta3_array,
		omega2_array,
		omega3_array,
		alpha2_array,
		alpha3_array,
		velocityA_array,
		velocityB_array,
		velocityBA_array,
		accelerationA_array,
		accelerationB_array,
		accelerationBA_array,
	)