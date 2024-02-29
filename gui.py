from dearpygui import dearpygui as dpg
from helpers import angle, calculate_end_point, magnitude

# Constants
WINDOW_WIDTH = 1600		# Width of the window
WINDOW_HEIGHT = 900		# Height of the window
TAB_WIDTH = 420			# Width of the plot tabs at the bottom of the window
TAB_HEIGHT = 400		# Height of the settings and plot tabs at the bottom of the window
RESOLUTION = 360		# How many angles should be calculated?
POSITION = "crossed"	# "open" or "crossed" positions for the linkage

# Colors
RED = (255, 0, 0, 255)
RED_50 = (138, 14, 14, 255)
BLUE = (0, 0, 255, 255)
BLUE_50 = (58, 137, 222, 255)
BROWN = (139, 69, 19, 255)
GREEN = (0, 255, 0, 255)
YELLOW = (255, 255, 0, 255)
YELLOW_50 = (138, 108, 14, 255)
GREY = (150, 150, 150, 255)

# Sizes
LINK_SCALE = 50
LINK_THICKNESS = 3
VELOCITY_SCALE = 5
VELOCITY_THICKNESS = 6
CIRCLE_THICKNESS = 8

# Create or update mechanism drawing window
def update_mechanism_drawing(
	a, 
	b,
	c,
	d,
	knife_offset,
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
):
	with dpg.window(
		tag="mechanism_drawing",
		label="Mechanism Drawing",
		width=WINDOW_WIDTH,
		height=WINDOW_HEIGHT - TAB_HEIGHT,
		pos=(TAB_WIDTH, 0),
		no_resize=True,
		no_close=True,
		no_move=True,
		no_collapse=True,
	) as mechanism_drawing:
		with dpg.drawlist(width=WINDOW_WIDTH, height=WINDOW_HEIGHT - TAB_HEIGHT) as drawlist:

			# Calculate offset
			offset = (WINDOW_WIDTH * 0.3, (WINDOW_HEIGHT - TAB_HEIGHT) * 0.75)
			index = 59

			# Draw ground link (D)
			with dpg.draw_layer(tag="link_d"):
				d1 = offset
				d2 = calculate_end_point(d1, d*LINK_SCALE, theta4)
				dpg.draw_line(d1, d2, color=GREY, thickness=LINK_THICKNESS)
				dpg.draw_circle(d1, 3, color=GREY, thickness=CIRCLE_THICKNESS)
				dpg.draw_circle(d2, 3, color=GREY, thickness=CIRCLE_THICKNESS)

			# Draw input link (A)
			with dpg.draw_layer(tag="link_a"):
				a1 = offset
				a2 = calculate_end_point(a1, a*LINK_SCALE, theta1[index])
				dpg.draw_line(a1, a2, color=RED, thickness=LINK_THICKNESS)
				dpg.draw_circle(a1, 3, color=RED, thickness=CIRCLE_THICKNESS)
				dpg.draw_circle(a2, 3, color=RED, thickness=CIRCLE_THICKNESS)

			# Draw link B
			with dpg.draw_layer(tag="link_b"):
				b1 = a2
				b2 = calculate_end_point(b1, b*LINK_SCALE, theta2[index])
				dpg.draw_line(b1, b2, color=BLUE, thickness=LINK_THICKNESS)
				dpg.draw_circle(b1, 3, color=BLUE, thickness=CIRCLE_THICKNESS)
				dpg.draw_circle(b2, 3, color=BLUE, thickness=CIRCLE_THICKNESS)

			# Draw link C
			with dpg.draw_layer(tag="link_c"):
				c1 = d2
				c2 = b2
				dpg.draw_line(c1, c2, color=YELLOW, thickness=LINK_THICKNESS)
				dpg.draw_circle(c1, 3, color=YELLOW, thickness=CIRCLE_THICKNESS)
				dpg.draw_circle(c2, 3, color=YELLOW, thickness=CIRCLE_THICKNESS)

			# Draw the knife
			with dpg.draw_layer(tag="knife"):
				kl1 = c2
				kl2 = calculate_end_point(kl1, knife_offset*LINK_SCALE, theta3[index])
				dpg.draw_line(kl1, kl2, color=YELLOW, thickness=LINK_THICKNESS)

				k1 = kl2
				k2 = calculate_end_point(k1, 2*LINK_SCALE, theta3[index])
				k3 = calculate_end_point(k1, 0.5*LINK_SCALE, theta3[index] - 90)
				dpg.draw_triangle(k1, k2, k3, color=BROWN, fill=BROWN)

			# Draw the velocity arrows
			with dpg.draw_layer(tag="velocity_arrows"):
				vA = velocityA[index]
				vB = velocityB[index]
				vBA = velocityBA[index]

				vA1 = a2
				vA2 = calculate_end_point(vA1, magnitude(vA)*VELOCITY_SCALE, angle(vA))
				vB1 = b2
				vB2 = calculate_end_point(vB1, magnitude(vB)*VELOCITY_SCALE, angle(vB))
				vBA1 = c2
				vBA2 = calculate_end_point(vBA1, magnitude(vBA)*VELOCITY_SCALE, angle(vBA))

				dpg.draw_arrow(vA2, vA1, color=RED_50, thickness=VELOCITY_THICKNESS)
				dpg.draw_arrow(vB2, vB1, color=BLUE_50, thickness=VELOCITY_THICKNESS)
				dpg.draw_arrow(vBA2, vBA1, color=YELLOW_50, thickness=VELOCITY_THICKNESS)

# Create arcs showing movement of crank and rocker links
# def create_paths(a, c, theta1, theta3):

# Create settings window
def create_settings(a, b, c, d, theta4, omega1):
	with dpg.window(
		label="Mechanism Settings",
		width=TAB_WIDTH,
		height=WINDOW_HEIGHT - TAB_HEIGHT,
		pos=(0, 0),
		no_resize=True,
		no_close=True,
		no_move=True,
		no_collapse=True,
	) as mechanism_settings:
		dpg.add_text("Link Lengths")
		dpg.add_slider_float(label="Input Link A (m)", default_value=a, max_value=10)
		dpg.add_slider_float(label="B (m)", default_value=b, max_value=10)
		dpg.add_slider_float(label="C (m)", default_value=c, max_value=10)
		dpg.add_slider_float(label="Ground Link D (m)", default_value=d, max_value=10)
		dpg.add_text("Ground Angle")
		dpg.add_slider_float(label="degree", default_value=theta4, min_value=0, max_value=360)
		dpg.add_text("Input Speed")
		dpg.add_slider_float(label="rpm", default_value=omega1, min_value=5, max_value=10)
		dpg.add_text("Analysis Settings")
		dpg.add_slider_int(label="Resolution", default_value=RESOLUTION, max_value=500)
		dpg.add_radio_button(["Open", "Crossed"], label="Position", default_value=0)
		dpg.add_button(label="Analyze")

# Create position plot window
def create_position_plot(theta1, theta2, theta3):
	with dpg.window(
		label="Position Analysis",
		width=TAB_WIDTH,
		height=TAB_HEIGHT,
		pos=(0, WINDOW_HEIGHT - TAB_HEIGHT),
		no_resize=True,
		no_close=True,
		no_move=True,
		no_collapse=True,
	) as position_analysis:
		with dpg.plot():
			dpg.add_plot_legend()
			dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
			dpg.set_axis_limits(dpg.last_item(), 0, 360)
			dpg.add_plot_axis(dpg.mvYAxis, label="Output Angle (degree)", tag="position_y_axis")
			dpg.add_line_series(theta1, theta2, label="Theta 2", parent="position_y_axis")
			dpg.add_line_series(theta1, theta3, label="Theta 3", parent="position_y_axis")

# Create velocity plot window
def create_velocity_plot(theta1, velocityA, velocityB, velocityBA):
	with dpg.window(
		label="Velocity Analysis",
		width=TAB_WIDTH,
		height=TAB_HEIGHT,
		pos=(TAB_WIDTH, WINDOW_HEIGHT - TAB_HEIGHT),
		no_resize=True,
		no_close=True,
		no_move=True,
		no_collapse=True,
	) as velocity_analysis:
		vA = [ magnitude(v) for v in velocityA ]
		vB = [ magnitude(v) for v in velocityB ]
		vBA = [ magnitude(v) for v in velocityBA ]

		with dpg.plot():
			dpg.add_plot_legend()
			dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
			dpg.set_axis_limits(dpg.last_item(), 0, 360)
			dpg.add_plot_axis(dpg.mvYAxis, label="Output Velocity (m/s)", tag="velocity_y_axis")
			dpg.add_line_series(theta1, vA, label="Velocity A", parent="velocity_y_axis")
			dpg.add_line_series(theta1, vB, label="Velocity B", parent="velocity_y_axis")
			dpg.add_line_series(theta1, vBA, label="Velocity BA", parent="velocity_y_axis")

# Create acceleration plot window
def create_acceleration_plot(theta1, accelerationA, accelerationB, accelerationBA):
	with dpg.window(
		label="Acceleration Analysis",
		width=TAB_WIDTH,
		height=TAB_HEIGHT,
		pos=(2*TAB_WIDTH, WINDOW_HEIGHT - TAB_HEIGHT),
		no_resize=True,
		no_close=True,
		no_move=True,
		no_collapse=True,
	) as acceleration_analysis:
		aA = [ magnitude(a) for a in accelerationA ]
		aB = [ magnitude(a) for a in accelerationB ]
		aBA = [ magnitude(a) for a in accelerationBA ]

		with dpg.plot():
			dpg.add_plot_legend()
			dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
			dpg.set_axis_limits(dpg.last_item(), 0, 360)
			dpg.add_plot_axis(dpg.mvYAxis, label="Output Acceleration (m/s^2)", tag="acceleration_y_axis")
			dpg.add_line_series(theta1, aA, label="Acceleration A", parent="acceleration_y_axis")
			dpg.add_line_series(theta1, aB, label="Acceleration B", parent="acceleration_y_axis")
			dpg.add_line_series(theta1, aBA, label="Acceleration BA", parent="acceleration_y_axis")

# Setup PyGUI
def setup_app():
	dpg.create_context()
	dpg.create_viewport(title="Mechanism Analysis and Simulation", width=WINDOW_WIDTH, height=WINDOW_HEIGHT)
	dpg.setup_dearpygui()

# Run PyGUI
def run_app(
	a,
	b,
	c,
	d,
	knife_offset,
	theta1,
	theta2,
	theta3,
	theta4,
	omega1,
	omega2,
	omega3,
	alpha1,
	alpha2,
	alpha3,
	velocityA,
	velocityB,
	velocityBA,
	accelerationA,
	accelerationB,
	accelerationBA,
):
	setup_app()

	create_settings(a, b, c, d, theta4, 10)
	create_position_plot(theta1, theta2, theta3)
	create_velocity_plot(theta1, velocityA, velocityB, velocityBA)
	create_acceleration_plot(theta1, accelerationA, accelerationB, accelerationBA)
	update_mechanism_drawing(
		a=a, 
		b=b,
		c=c,
		d=d,
		knife_offset=knife_offset,
		theta1=theta1,
		theta2=theta2,
		theta3=theta3,
		theta4=theta4,
		omega1=omega1,
		omega2=omega2,
		omega3=omega3,
		velocityA=velocityA,
		velocityB=velocityB,
		velocityBA=velocityBA,
	)

	dpg.show_viewport()
	dpg.start_dearpygui()
	dpg.destroy_context()
