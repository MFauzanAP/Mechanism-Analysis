from lib import analyze_mechanism
from gui import run_app

# Constants
RESOLUTION = 360		# How many angles should be calculated?
POSITION = "crossed"	# "open" or "crossed" positions for the linkage

# Link Lengths
a = 2
b = 6
c = 5
d = 5

# Ground Angle
theta4 = 0

# Input Speed (RPM)
omega1 = 10

# Analyze the mechanism
(
	theta1,
	theta2,
	theta3,
	omega2,
	omega3,
	velocityA,
	velocityB,
	velocityBA,
) = analyze_mechanism(a, b, c, d, omega1, RESOLUTION, POSITION)

# Run the app
run_app(
	a, 
	b,
	c,
	d,
	theta1,
	theta2,
	theta3,
	theta4,
	omega1,
	omega2,
	omega3,
	velocityA,
	velocityB,
	velocityBA,
)

# Plot the results
# plt.plot(theta1, theta2, 'ro')
# plt.plot(theta1, theta3, 'bo')
# plt.show()