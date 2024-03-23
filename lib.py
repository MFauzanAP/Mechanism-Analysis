import math
from numpy import linspace, array, matrix, transpose
from numpy.linalg import inv
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
def calculate_acceleration(a, b, c, knife_offset, theta1, omega1, alpha1, theta2, theta3, omega2, omega3):

	# Calculate alphabetic constants
	A = c * math.sin(theta3)
	B = b * math.sin(theta2)
	C = a*alpha1*math.sin(theta1) + a*(omega1**2)*math.cos(theta1) + b*(omega2**2)*math.cos(theta2) - c*(omega3**2)*math.cos(theta3)
	D = c * math.cos(theta3)
	E = b * math.cos(theta2)
	F = a*alpha1*math.cos(theta1) - a*(omega1**2)*math.sin(theta1) - b*(omega2**2)*math.sin(theta2) + c*(omega3**2)*math.sin(theta3)

	# Calculate alpha 2 and 3
	alpha2 = (C*D - A*F) / (A*E - B*D)
	alpha3 = (C*E - B*F) / (A*E - B*D)

	# Calculate linear acceleration A, B, and BA
	accelerationA = (-a*alpha1*math.sin(theta1) - a*(omega1**2)*math.cos(theta1), a*alpha1*math.cos(theta1) - a*(omega1**2)*math.sin(theta1))
	accelerationBA = (-b*alpha2 *math.sin(theta2) - b*(omega2**2)*math.cos(theta2), b*alpha2*math.cos(theta2) - b*(omega2**2)*math.sin(theta2))
	accelerationB = (-c*alpha3*math.sin(theta3) - c*(omega3**2)*math.cos(theta3), c*alpha3*math.cos(theta3) - c*(omega3**2)*math.sin(theta3))

	# Calculate acceleration at center of gravity
	cgAccelerationA = (-0.5*a*alpha1*math.sin(theta1) - 0.5*a*(omega1**2)*math.cos(theta1), 0.5*a*alpha1*math.cos(theta1) - 0.5*a*(omega1**2)*math.sin(theta1))
	cgAccelerationBA = (-0.5*b*alpha2 *math.sin(theta2) - 0.5*b*(omega2**2)*math.cos(theta2), 0.5*b*alpha2*math.cos(theta2) - 0.5*b*(omega2**2)*math.sin(theta2))
	cgAccelerationB = (-0.5*c*alpha3*math.sin(theta3) - 0.5*c*(omega3**2)*math.cos(theta3), 0.5*c*alpha3*math.cos(theta3) - 0.5*c*(omega3**2)*math.sin(theta3))

	# Return the calculated accelerations
	return (alpha2, alpha3, accelerationA, accelerationB, accelerationBA, cgAccelerationA, cgAccelerationB, cgAccelerationBA)

# Calculate transmission angle and mechanical advantage
def calculate_performance(a, b, c, d, theta1, omega1, omega3):
	transmission_angle = math.acos((a**2 + d**2 - b**2 - c**2 - 2*a*d*math.cos(theta1)) / (-2*b*c))
	velocity_ratio = omega3 / omega1
	mechanical_advantage = (omega1 * a) / (omega3 * c)
	return (transmission_angle, velocity_ratio, mechanical_advantage)

# Calculate link geometric properties
def calculate_geometric_properties(link_diameter, cross_shape, density, a, b, c, theta1):
	link_radius = link_diameter / 2

	# Calculate the mass of each link
	cross_area = 0
	if cross_shape == "square": cross_area = link_diameter**2
	elif cross_shape == "circle": cross_area = math.pi * link_radius**2
	elif cross_shape == "hollow circle (50%)": cross_area = math.pi * link_radius**2 * 0.5
	m1 = density * cross_area * a
	m2 = density * cross_area * b
	m3 = density * cross_area * c

	# Calculate the moment of inertia of each link
	i1 = math.pi * link_radius ** 4
	i2 = math.pi * link_radius ** 4
	i3 = math.pi * link_radius ** 4

	# Return the calculated properties
	return (m1, m2, m3, cross_area, i1, i2, i3)

# Calculate the forces at each joint and at the knife edge, as well as the driving torque required
def calculate_dynamic(
	m1,
	m2,
	m3,
	i1,
	i2,
	i3,
	force_start,
	force_end,
	force_mag,
	a,
	b,
	c,
	knife_offset,
	theta1,
	theta2,
	theta3,
	alpha1,
	alpha2,
	alpha3,
	cgAccelerationA,
	cgAccelerationB,
	cgAccelerationBA,
):

	# Construct coefficient matrix
	r41y = -(a/2)*math.sin(theta1)
	r41x = -(a/2)*math.cos(theta1)
	r21y = -r41y
	r21x = -r41x
	r12y = -(b/2)*math.sin(theta2)
	r12x = -(b/2)*math.cos(theta2)
	r32y = -r12y
	r32x = -r12x
	r23y = ((c+knife_offset)/2)*(1-knife_offset)*math.sin(theta3)
	r23x = ((c+knife_offset)/2)*(1-knife_offset)*math.cos(theta3)
	r43y = -((c+knife_offset)/2)*math.sin(theta3)
	r43x = -((c+knife_offset)/2)*math.cos(theta3)
	coefficient_matrix = matrix(array([
		[1,0,1, 0,0,0, 0,0,0],
		[0,1,0, 1,0,0, 0,0,0],
		[-r41y,r41x,-r21y, r21x,0,0, 0,0,1],

		[0,0,-1, 0,1,0, 0,0,0],
		[0,0,0, -1,0,1, 0,0,0],
		[0,0,r12y, -r12x,-r32y,r32x, 0,0,0],

		[0,0,0, 0,-1,0, 1,0,0],
		[0,0,0, 0,0,-1, 0,1,0],
		[0,0,0, 0,r23y,-r23x, -r43y,r43x,0],
	]))

	# Construct known-values matrix
	force_mid = (force_start + force_end) / 2 if force_start < force_end else (force_start + force_end + 360) / 2
	force_amp = (force_end - force_start) / 2 if force_start < force_end else (force_end - force_start + 360) / 2
	f_cut = 0
	if force_start < force_end:
		if math.degrees(theta1) > force_start and math.degrees(theta1) < force_end:
			f_cut = (abs(math.degrees(theta1) - force_mid) - force_amp) * force_mag / force_amp
	else:
		if math.degrees(theta1) > force_start or math.degrees(theta1) < force_end:
			clamped_theta = math.degrees(theta1) if math.degrees(theta1) > force_start else math.degrees(theta1) + 360
			f_cut = (abs(clamped_theta - force_mid) - force_amp) * force_mag / force_amp
	known_matrix = transpose(matrix(array([
		-m1*cgAccelerationA[0],
		-m1*cgAccelerationA[1] + m1*9.81,
		-i1*alpha1,
		-m2*cgAccelerationBA[0],
		-m2*cgAccelerationBA[1] + m2*9.81,
		-i2*alpha2,
		-m3*cgAccelerationB[0] - f_cut,
		-m3*cgAccelerationB[1] + m3*9.81,
		-i3*alpha3 + f_cut*math.sin(theta3)*((c+knife_offset)/2),
	])))

	# Calculate the forces and torques
	forces_and_torques = inv(coefficient_matrix) * known_matrix
	
	# Return results
	return (
		f_cut,
		forces_and_torques[0,0],
		forces_and_torques[1,0],
		forces_and_torques[2,0],
		forces_and_torques[3,0],
		-1*forces_and_torques[2,0],
		-1*forces_and_torques[3,0],
		forces_and_torques[4,0],
		forces_and_torques[5,0],
		-1*forces_and_torques[4,0],
		-1*forces_and_torques[5,0],
		forces_and_torques[6,0],
		forces_and_torques[7,0],
		forces_and_torques[8,0],
	)

# Calculate mechanism strength properties
def calculate_strength(
	link_diameter,
	cross_shape,
	s_ut,
):
	s_e_prime = s_ut / 2
	if s_ut > 1400: s_e_prime = 700

	# Calculate marin factors
	# TODO make these changeable
	ka = 3.04 * (s_ut**-0.217)
	d_e = 0.37 * link_diameter
	kb = 1.51 * (d_e**-0.157) if d_e > 0.051 else 1.24 * (d_e**-0.107)
	kc = 1.0
	kd = 1.0
	ke = 0.62

	# Calculate actual fatigue strength
	s_e = s_e_prime * ka * kb * kc * kd * ke

	# Find stress concentration factor
	kt = 1.85					# assuming the hole is atleast a third of the diameter of the link
	roota = 1.24 - (2.25e-3 * s_ut) + (1.6e-6 * s_ut**2) - (4.11e-10 * s_ut**3)
	kf = 1 + ((kt - 1) / (1 + math.sqrt((roota**2) / (link_diameter / 2))))
	print(ka, kb, kf)

	return (s_e, kf)

# Calculate stresses for each link
def calculate_stresses(
	a,
	b,
	c,
	knife_offset,
	theta1,
	theta2,
	theta3,
	alpha1,
	alpha2,
	alpha3,
	cgAccelerationA,
	cgAccelerationB,
	cgAccelerationBA,
	f_cut,
	f41x,
	f41y,
	f21x,
	f21y,
	f12x,
	f12y,
	f32x,
	f32y,
	f23x,
	f23y,
	f43x,
	f43y,
	torque,
	link_diameter,
	m1,
	m2,
	m3,
	i1,
	i2,
	i3,
	s_ut,
	s_e,
	kf,
):
	r41y = -(a/2)*math.sin(theta1)
	r41x = -(a/2)*math.cos(theta1)
	r21y = -r41y
	r21x = -r41x
	r12y = -(b/2)*math.sin(theta2)
	r12x = -(b/2)*math.cos(theta2)
	r32y = -r12y
	r32x = -r12x
	r23y = ((c+knife_offset)/2)*(1-knife_offset)*math.sin(theta3)
	r23x = ((c+knife_offset)/2)*(1-knife_offset)*math.cos(theta3)
	r43y = -((c+knife_offset)/2)*math.sin(theta3)
	r43x = -((c+knife_offset)/2)*math.cos(theta3)

	# Calculate maximum bending moment for each link (N/m)
	m_max_1 = (r41x * m1 * 9.81) + (2 * r21x * f21y) - (2 * r21y * f21x)
	m_max_2 = (i2 * alpha2) + (r12x * m2 * 9.81) - (r12y * m2 * cgAccelerationBA[0]) - (r12x * m2 * cgAccelerationBA[1]) - (2 * r32x * f32y) - (2 * r32y * f32x)
	m_max_3 = (i3 * alpha3) + (r43x * m3 * 9.81) - (r43y * m3 * cgAccelerationB[0]) - (r43x * m3 * cgAccelerationB[1]) - (c * math.cos(theta3) * f23y) + (c * math.sin(theta3) * f23x) - (2 * r23y * f_cut)

	# Calculate bending stress for each link (MPa)
	sigma_max_1 = (m_max_1 * (link_diameter / 2) / i1) / 1e6
	sigma_max_2 = (m_max_2 * (link_diameter / 2) / i2) / 1e6
	sigma_max_3 = (m_max_3 * (link_diameter / 2) / i3) / 1e6

	# Assuming stress is fluctuating from 0 to max
	sigma_1 = sigma_max_1 * 1.85
	sigma_2 = sigma_max_2 * 1.85
	sigma_3 = sigma_max_3 * 1.85

	return (sigma_1, sigma_2, sigma_3)

# Calculate yield and fatigue factors of safety
def calculate_factors_of_safety(
	sigma_1,
	sigma_2,
	sigma_3,
	s_ut,
	s_y,
	s_e,
	kf,
):
	# Get the minimum and maximum stresses for each link
	min_1 = min(sigma_1) * kf[0]
	min_2 = min(sigma_2) * kf[0]
	min_3 = min(sigma_3) * kf[0]
	max_1 = max(sigma_1) * kf[0]
	max_2 = max(sigma_2) * kf[0]
	max_3 = max(sigma_3) * kf[0]

	# Calculate the yield factor of safety
	n_yield_1 = s_y / max(abs(min_1), abs(max_1))
	n_yield_2 = s_y / max(abs(min_2), abs(max_2))
	n_yield_3 = s_y / max(abs(min_3), abs(max_3))

	# Calculate sigma a and m
	sigma_a_1 = abs((max_1 - min_1) / 2)
	sigma_a_2 = abs((max_2 - min_2) / 2)
	sigma_a_3 = abs((max_3 - min_3) / 2)
	sigma_m_1 = abs((max_1 + min_1) / 2)
	sigma_m_2 = abs((max_2 + min_2) / 2)
	sigma_m_3 = abs((max_3 + min_3) / 2)

	# Calculate fatigue factor of safety
	n_fatigue_1 = pow((sigma_a_1 / s_e[0]) + (sigma_m_1 / s_ut), -1)
	n_fatigue_2 = pow((sigma_a_2 / s_e[0]) + (sigma_m_2 / s_ut), -1)
	n_fatigue_3 = pow((sigma_a_3 / s_e[0]) + (sigma_m_3 / s_ut), -1)

	return (n_yield_1, n_yield_2, n_yield_3, n_fatigue_1, n_fatigue_2, n_fatigue_3)

# Calculate and aggregate position, velocity, and acceleration analysis results for the given input angle
def calculate_results(
	offset,
	link_scale,
	link_diameter,
	cross_shape,
	density,
	s_ut,
	force_start,
	force_end,
	force_mag,
	a,
	b,
	c,
	d,
	knife_offset,
	position,
	theta1,
	theta4,
	omega1,
	alpha1,
):

	# Convert to appropriate units
	theta1 = math.radians(theta1)
	theta4 = math.radians(theta4)
	omega1 = omega1 * 0.10472
	alpha1 = math.radians(alpha1)

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
	(
		alpha2,
		alpha3,
		accelerationA,
		accelerationB,
		accelerationBA,
		cgAccelerationA,
		cgAccelerationB,
		cgAccelerationBA,
	) = calculate_acceleration(
		a=a,
		b=b,
		c=c,
		knife_offset=knife_offset,
		theta1=theta1,
		omega1=omega1,
		alpha1=alpha1,
		theta2=theta2,
		theta3=theta3,
		omega2=omega2,
		omega3=omega3,
	)

	# Solve for performance
	(transmission_angle, velocity_ratio, mechanical_advantage) = calculate_performance(
		a=a,
		b=b,
		c=c,
		d=d,
		theta1=theta1,
		omega1=omega1,
		omega3=omega3,
	)

	# Solve for link geometric properties
	(m1, m2, m3, cross_area, i1, i2, i3) = calculate_geometric_properties(
		theta1=theta1,
		link_diameter=link_diameter,
		cross_shape=cross_shape,
		density=density,
		a=a,
		b=b,
		c=c,
	)

	# Solve for dynamic forces and torque
	(
		f_cut,
		f41x,
		f41y,
		f21x,
		f21y,
		f12x,
		f12y,
		f32x,
		f32y,
		f23x,
		f23y,
		f43x,
		f43y,
		torque,
	) = calculate_dynamic(
		m1=m1,
		m2=m2,
		m3=m3,
		i1=i1,
		i2=i2,
		i3=i3,
		force_start=force_start,
		force_end=force_end,
		force_mag=force_mag,
		a=a,
		b=b,
		c=c,
		knife_offset=knife_offset,
		theta1=theta1,
		theta2=theta2,
		theta3=theta3,
		alpha1=alpha1,
		alpha2=alpha2,
		alpha3=alpha3,
		cgAccelerationA=cgAccelerationA,
		cgAccelerationB=cgAccelerationB,
		cgAccelerationBA=cgAccelerationBA,
	)

	# Solve for strength properties
	(s_e, kf) = calculate_strength(
		link_diameter=link_diameter,
		cross_shape=cross_shape,
		s_ut=s_ut,
	)

	# Solve for factor of safety
	(sigma_1, sigma_2, sigma_3) = calculate_stresses(
		a=a,
		b=b,
		c=c,
		knife_offset=knife_offset,
		theta1=theta1,
		theta2=theta2,
		theta3=theta3,
		alpha1=alpha1,
		alpha2=alpha2,
		alpha3=alpha3,
		cgAccelerationA=cgAccelerationA,
		cgAccelerationB=cgAccelerationB,
		cgAccelerationBA=cgAccelerationBA,
		f_cut=f_cut,
		f41x=f41x,
		f41y=f41y,
		f21x=f21x,
		f21y=f21y,
		f12x=f12x,
		f12y=f12y,
		f32x=f32x,
		f32y=f32y,
		f23x=f23x,
		f23y=f23y,
		f43x=f43x,
		f43y=f43y,
		torque=torque,
		link_diameter=link_diameter,
		m1=m1,
		m2=m2,
		m3=m3,
		i1=i1,
		i2=i2,
		i3=i3,
		s_ut=s_ut,
		s_e=s_e,
		kf=kf,
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
		transmission_angle,
		velocity_ratio,
		mechanical_advantage,
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
		cgAccelerationA,
		cgAccelerationB,
		cgAccelerationBA,
		f41x,
		f41y,
		f21x,
		f21y,
		f12x,
		f12y,
		f32x,
		f32y,
		f23x,
		f23y,
		f43x,
		f43y,
		torque,
		sigma_1,
		sigma_2,
		sigma_3,
		s_e,
		kf,
	)

# Loops through all possible angles, calculates the results, and aggregates them into arrays
def analyze_mechanism(
	a,
	b,
	c,
	d,
	knife_offset,
	theta4,
	omega1,
	alpha1,
	offset=0,
	link_scale=500,
	link_diameter=0.01,
	cross_shape="circle",
	density=2700,
	s_ut=310,
	s_y=275,
	force_start=0,
	force_end=20,
	force_mag=100,
	resolution=360,
	position="OPEN",
	convert=True,
):

	# Output arrays
	output = []
	NUM_RESULTS = 47
	NUM_SCALAR_RESULTS = 8

	# Prepare output array
	for i in range(NUM_RESULTS): output.append([])

	# Loop through all possible angles
	for theta1 in linspace(1, 360, num=resolution):

		# Calculate the results
		results = calculate_results(
			offset=offset,
			link_scale=link_scale,
			link_diameter=link_diameter,
			cross_shape=cross_shape,
			density=density,
			s_ut=s_ut,
			force_start=force_start,
			force_end=force_end,
			force_mag=force_mag,
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

		# For each data returned from the analysis
		for i in range(NUM_RESULTS):

			# Convert units if specified
			if convert and i < NUM_SCALAR_RESULTS: output[i].append(math.degrees(results[i]))

			# Append to the output arrays
			else: output[i].append(results[i])

	# Calculate the factors of safety
	fos = calculate_factors_of_safety(
		sigma_1=output[42],
		sigma_2=output[43],
		sigma_3=output[44],
		s_ut=s_ut,
		s_y=s_y,
		s_e=output[45],
		kf=output[46],
	)
	for n in fos: output.append(n)

	# Return the results
	return output