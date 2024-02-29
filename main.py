from lib import analyze_mechanism
from gui import run_app

# Constants
RESOLUTION = 360		# How many angles should be calculated?
POSITION = "crossed"	# "open" or "crossed" positions for the linkage

# Link Lengths
a = 1
b = 4.5
c = 2.5
d = 4.5

# Ground Angle
theta4 = 0

# Input Speed (RPM)
omega1 = 10

# Motor Acceleration (RPM/s)
alpha1 = 0

# Analyze the mechanism
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
) = analyze_mechanism(
	a=a,
	b=b,
	c=c,
	d=d,
	omega1=omega1,
	alpha1=alpha1,
	resolution=RESOLUTION,
	position=POSITION,
)

# Run the app
run_app(
	a=a, 
	b=b,
	c=c,
	d=d,
	theta1=theta1,
	theta2=theta2,
	theta3=theta3,
	theta4=theta4,
	omega1=omega1,
	omega2=omega2,
	omega3=omega3,
	alpha1=alpha1,
	alpha2=alpha2,
	alpha3=alpha3,
	velocityA=velocityA,
	velocityB=velocityB,
	velocityBA=velocityBA,
	accelerationA=accelerationA,
	accelerationB=accelerationB,
	accelerationBA=accelerationBA,
)

# Plot the results
# plt.plot(theta1, theta2, 'ro')
# plt.plot(theta1, theta3, 'bo')
# plt.show()