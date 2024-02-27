from lib import analyze_mechanism
from dearpygui import dearpygui as dpg

# Constants
WINDOW_WIDTH = 1600	# Width of the window
WINDOW_HEIGHT = 900	# Height of the window
RESOLUTION = 360	# How many angles should be calculated?
POSITION = "crossed"	# "open" or "crossed" positions for the linkage

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

with dpg.window(label="Mechanism Settings", width=300, no_resize=True, no_close=True) as mechanism_settings:
	dpg.add_text("Link Lengths")
	dpg.add_slider_float(label="a", default_value=a, max_value=10)
	dpg.add_slider_float(label="b", default_value=b, max_value=10)
	dpg.add_slider_float(label="c", default_value=c, max_value=10)
	dpg.add_slider_float(label="d", default_value=d, max_value=10)
	dpg.add_text("Analysis Settings")
	dpg.add_slider_int(label="Resolution", default_value=RESOLUTION, max_value=720)
	dpg.add_radio_button(["Open", "Crossed"], label="Position", default_value=0)
	dpg.add_button(label="Analyze")

with dpg.window(label="Position Analysis", no_resize=True, no_close=True) as position_analysis:
	with dpg.plot():
		dpg.add_plot_legend()
		dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle")
		dpg.set_axis_limits(dpg.last_item(), 0, 360)
		dpg.add_plot_axis(dpg.mvYAxis, label="Output Angle", tag="y_axis")
		dpg.add_line_series(theta1, theta2, label="Theta 2", parent="y_axis")
		dpg.add_line_series(theta1, theta3, label="Theta 3", parent="y_axis")

# Run PyGUI
dpg.set_item_pos(mechanism_settings, (0, 0))
dpg.set_item_pos(position_analysis, (300, 0))
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()

# Plot the results
# plt.plot(theta1, theta2, 'ro')
# plt.plot(theta1, theta3, 'bo')
# plt.show()