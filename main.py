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

# Input Speed (RPM)
omega1 = 10

# Analyze the mechanism
(theta1, theta2, theta3) = analyze_mechanism(a, b, c, d, RESOLUTION, POSITION)

# Run the app
run_app(a, b, c, d, theta1, theta2, theta3)

# Plot the results
# plt.plot(theta1, theta2, 'ro')
# plt.plot(theta1, theta3, 'bo')
# plt.show()