import math
from numpy import linspace
from helpers import calculate_end_point

# Calculate theta 2 and 3 from the given input angle
def calculate_position(offset, link_scale, a, b, c, d, knife_offset, position, theta1, theta4):

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

	# Calculate positions of each joint
	d1 = offset
	d2 = calculate_end_point(d1, d * link_scale, math.degrees(theta4))
	a1 = offset
	a2 = calculate_end_point(a1, a * link_scale, math.degrees(theta1))
	b1 = a2
	b2 = calculate_end_point(b1, b * link_scale, math.degrees(theta2))
	c1 = d2
	c2 = b2
	kl1 = c2
	kl2 = calculate_end_point(kl1, knife_offset * link_scale, math.degrees(theta3))

	# Return the calculated angles
	return (
		theta2,
		theta3,
		a1,
		a2,
		b1,
		b2,
		c1,
		c2,
		d1,
		d2,
		kl1,
		kl2,
	)

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
def calculate_results(offset, link_scale, a, b, c, d, knife_offset, position, theta1, theta4, omega1, alpha1):

	# Convert to radians
	theta1 = math.radians(theta1)

	# Solve for position
	(
		theta2,
		theta3,
		a1,
		a2,
		b1,
		b2,
		c1,
		c2,
		d1,
		d2,
		kl1,
		kl2,
	) = calculate_position(
		offset=offset,
		link_scale=link_scale,
		a=a,
		b=b,
		c=c,
		d=d,
		knife_offset=knife_offset,
		position=position,
		theta1=theta1,
		theta4=theta4,
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
		a1,
		a2,
		b1,
		b2,
		c1,
		c2,
		d1,
		d2,
		kl1,
		kl2,
		velocityA,
		velocityB,
		velocityBA,
		accelerationA,
		accelerationB,
		accelerationBA,
	)

# Loops through all possible angles, calculates the results, and aggregates them into arrays
def analyze_mechanism(a, b, c, d, knife_offset, theta4, omega1, alpha1, offset=0, link_scale=500, resolution=360, position="OPEN", convert=True):

	# Output arrays
	output = []
	NUM_RESULTS = 23
	NUM_SCALAR_RESULTS = 6

	# For each data returned from the analysis
	for i in range(NUM_RESULTS):

		# Append an empty array
		output.append([])

		# Loop through all possible angles
		for theta1 in linspace(1, 360, num=resolution):

			# Calculate the results
			results = calculate_results(
				offset=offset,
				link_scale=link_scale,
				a=a,
				b=b,
				c=c,
				d=d,
				knife_offset=knife_offset,
				position=position,
				theta1=theta1,
				theta4=theta4,
				omega1=omega1,
				alpha1=alpha1,
			)

			# Convert units if specified
			if convert and i < NUM_SCALAR_RESULTS: output[i].append(math.degrees(results[i]))

			# Append to the output arrays
			else: output[i].append(results[i])

	# Return the results
	return output