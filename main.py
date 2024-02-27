import math
from lib import analyze_mechanism
from dearpygui import dearpygui as dpg

# Constants
WINDOW_WIDTH = 1600		# Width of the window
WINDOW_HEIGHT = 900		# Height of the window
TAB_HEIGHT = 380		# Height of the settings and plot tabs at the bottom of the window
SCALE = 50				# Scale of the drawing in pixels per unit
RESOLUTION = 360		# How many angles should be calculated?
POSITION = "crossed"	# "open" or "crossed" positions for the linkage

# Colors
RED = (255, 0, 0, 255)
BLUE = (0, 0, 255, 255)
GREEN = (0, 255, 0, 255)
YELLOW = (255, 255, 0, 255)
GREY = (150, 150, 150, 255)

# Thicknesses
LINK_THICKNESS = 4
CIRCLE_THICKNESS = 8

# Link Lengths
a = 2
b = 6
c = 5
d = 5

# Setup PyGUI
dpg.create_context()
dpg.create_viewport(title="Mechanism Analysis and Simulation", width=WINDOW_WIDTH, height=WINDOW_HEIGHT)

# Analyze the mechanism
(theta1, theta2, theta3) = analyze_mechanism(a, b, c, d, RESOLUTION, POSITION)

# Create mechanism drawing window
with dpg.window(
	label="Mechanism Drawing",
	width=WINDOW_WIDTH,
	height=WINDOW_HEIGHT - TAB_HEIGHT,
	no_resize=True,
	no_close=True,
	no_move=True,
	no_collapse=True,
) as mechanism_drawing:
	with dpg.drawlist(width=WINDOW_WIDTH, height=WINDOW_HEIGHT - TAB_HEIGHT) as drawlist:

		# Calculate offset
		offset = (WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2)
		index = 89

		# Draw link A
		with dpg.draw_layer(tag="link_a"):
			a1 = offset
			a2 = (offset[0] + a*math.cos(math.radians(theta1[index]))*SCALE, offset[1] - a*math.sin(math.radians(theta1[index]))*SCALE)
			dpg.draw_line(a1, a2, color=RED, thickness=LINK_THICKNESS)
			dpg.draw_circle(a1, 3, color=RED, thickness=CIRCLE_THICKNESS)
			dpg.draw_circle(a2, 3, color=RED, thickness=CIRCLE_THICKNESS)

		# Draw link B
		with dpg.draw_layer(tag="link_b"):
			b1 = a2
			b2 = (b1[0] + b*math.cos(math.radians(theta3[index]))*SCALE, b1[1] - b*math.sin(math.radians(theta3[index]))*SCALE)
			dpg.draw_line(b1, b2, color=BLUE, thickness=LINK_THICKNESS)
			dpg.draw_circle(b1, 3, color=BLUE, thickness=CIRCLE_THICKNESS)
			dpg.draw_circle(b2, 3, color=BLUE, thickness=CIRCLE_THICKNESS)

		# Draw link C
		with dpg.draw_layer(tag="link_c"):
			c1 = b2
			c2 = (c1[0] - c*math.cos(math.radians(theta2[index]))*SCALE, c1[1] + c*math.sin(math.radians(theta2[index]))*SCALE)
			dpg.draw_line(c1, c2, color=YELLOW, thickness=LINK_THICKNESS)
			dpg.draw_circle(c1, 3, color=YELLOW, thickness=CIRCLE_THICKNESS)
			dpg.draw_circle(c2, 3, color=YELLOW, thickness=CIRCLE_THICKNESS)

		# Draw link D
		with dpg.draw_layer(tag="link_d"):
			d1 = c2
			d2 = a1
			dpg.draw_line(d1, d2, color=GREY, thickness=LINK_THICKNESS)
			dpg.draw_circle(d1, 3, color=GREY, thickness=CIRCLE_THICKNESS)
			dpg.draw_circle(d2, 3, color=GREY, thickness=CIRCLE_THICKNESS)

# Create settings window
with dpg.window(
	label="Mechanism Settings",
	width=300,
	height=TAB_HEIGHT,
	pos=(0, WINDOW_HEIGHT - TAB_HEIGHT),
	no_resize=True,
	no_close=True,
	no_move=True,
	no_collapse=True,
) as mechanism_settings:
	dpg.add_text("Link Lengths")
	dpg.add_slider_float(label="a", default_value=a, max_value=10)
	dpg.add_slider_float(label="b", default_value=b, max_value=10)
	dpg.add_slider_float(label="c", default_value=c, max_value=10)
	dpg.add_slider_float(label="d", default_value=d, max_value=10)
	dpg.add_text("Analysis Settings")
	dpg.add_slider_int(label="Resolution", default_value=RESOLUTION, max_value=720)
	dpg.add_radio_button(["Open", "Crossed"], label="Position", default_value=0)
	dpg.add_button(label="Analyze")

# Create position plot window
with dpg.window(
	label="Position Analysis",
	height=TAB_HEIGHT,
	pos=(300, WINDOW_HEIGHT - TAB_HEIGHT),
	no_resize=True,
	no_close=True,
	no_move=True,
	no_collapse=True,
) as position_analysis:
	with dpg.plot():
		dpg.add_plot_legend()
		dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle")
		dpg.set_axis_limits(dpg.last_item(), 0, 360)
		dpg.add_plot_axis(dpg.mvYAxis, label="Output Angle", tag="y_axis")
		dpg.add_line_series(theta1, theta2, label="Theta 2", parent="y_axis")
		dpg.add_line_series(theta1, theta3, label="Theta 3", parent="y_axis")

# Run PyGUI
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()

# Plot the results
# plt.plot(theta1, theta2, 'ro')
# plt.plot(theta1, theta3, 'bo')
# plt.show()