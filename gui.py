from lib import analyze_mechanism
from dearpygui import dearpygui as dpg
from helpers import angle, calculate_end_point, magnitude

class App:

	# Constants
	WINDOW_WIDTH = 1600		# Width of the window
	WINDOW_HEIGHT = 900		# Height of the window
	TAB_WIDTH = 420			# Width of the plot tabs at the bottom of the window
	TAB_HEIGHT = 400		# Height of the settings and plot tabs at the bottom of the window

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

	# Windows
	mechanism_settings = None
	mechanism_drawing = None
	position_analysis = None
	velocity_analysis = None
	acceleration_analysis = None

	# Link Lengths
	a = 1
	b = 4.5
	c = 2.5
	d = 4.5

	# Link Angles
	theta1 = 0
	theta2 = 0
	theta3 = 0
	theta4 = 0		# Ground Angle

	# Link Angular Velocities
	omega1 = 10		# Input Speed (RPM)
	omega2 = 0
	omega3 = 0
	omega4 = 0

	# Link Linear Velocities
	velocityA = (0, 0)
	velocityB = (0, 0)
	velocityBA = (0, 0)

	# Link Angular Accelerations
	alpha1 = 0		# Motor Acceleration (RPM/s)
	alpha2 = 0
	alpha3 = 0
	alpha4 = 0

	# Link Linear Accelerations
	accelerationA = (0, 0)
	accelerationB = (0, 0)
	accelerationBA = (0, 0)

	# Knife Offset
	knife_offset = 1

	# Analysis Settings
	resolution = 360
	position = "crossed"

	# Constructor for the app, sets up the PyGUI context and viewport
	def __init__(self):
		# Analyze the initial mechanism
		self.analyze_mechanism()

		# Setup the app
		dpg.create_context()
		dpg.create_viewport(title="Mechanism Analysis and Simulation", width=App.WINDOW_WIDTH, height=App.WINDOW_HEIGHT)
		dpg.setup_dearpygui()
		
		# Create the windows
		self.create_settings()
		self.create_position_plot()
		self.create_velocity_plot()
		self.create_acceleration_plot()
		self.update_mechanism_drawing()

	# Uses the updated mechanism properties to update the computed values
	def analyze_mechanism(self):
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
			a=self.a,
			b=self.b,
			c=self.c,
			d=self.d,
			omega1=self.omega1,
			alpha1=self.alpha1,
			resolution=self.resolution,
			position=self.position,
		)

		# Update mechanism properties
		self.theta1 = theta1
		self.theta2 = theta2
		self.theta3 = theta3
		self.omega2 = omega2
		self.omega3 = omega3
		self.alpha2 = alpha2
		self.alpha3 = alpha3
		self.velocityA = velocityA
		self.velocityB = velocityB
		self.velocityBA = velocityBA
		self.accelerationA = accelerationA
		self.accelerationB = accelerationB
		self.accelerationBA = accelerationBA

	# Runs the app and starts the render loop
	def run(self):
		dpg.show_viewport()
		dpg.start_dearpygui()
		dpg.destroy_context()

	# Create or update mechanism drawing window
	def update_mechanism_drawing(self):
		with dpg.window(
			tag="mechanism_drawing",
			label="Mechanism Drawing",
			width=App.WINDOW_WIDTH,
			height=App.WINDOW_HEIGHT - App.TAB_HEIGHT,
			pos=(App.TAB_WIDTH, 0),
			no_resize=True,
			no_close=True,
			no_move=True,
			no_collapse=True,
		) as mechanism_drawing:
			with dpg.drawlist(width=App.WINDOW_WIDTH, height=App.WINDOW_HEIGHT - App.TAB_HEIGHT) as drawlist:

				# Calculate offset
				offset = (App.WINDOW_WIDTH * 0.3, (App.WINDOW_HEIGHT - App.TAB_HEIGHT) * 0.75)
				index = 59

				# Draw ground link (D)
				with dpg.draw_layer(tag="link_d"):
					d1 = offset
					d2 = calculate_end_point(d1, self.d*App.LINK_SCALE, self.theta4)
					dpg.draw_line(d1, d2, color=App.GREY, thickness=App.LINK_THICKNESS)
					dpg.draw_circle(d1, 3, color=App.GREY, thickness=App.CIRCLE_THICKNESS)
					dpg.draw_circle(d2, 3, color=App.GREY, thickness=App.CIRCLE_THICKNESS)

				# Draw input link (A)
				with dpg.draw_layer(tag="link_a"):
					a1 = offset
					a2 = calculate_end_point(a1, self.a*App.LINK_SCALE, self.theta1[index])
					dpg.draw_line(a1, a2, color=App.RED, thickness=App.LINK_THICKNESS)
					dpg.draw_circle(a1, 3, color=App.RED, thickness=App.CIRCLE_THICKNESS)
					dpg.draw_circle(a2, 3, color=App.RED, thickness=App.CIRCLE_THICKNESS)

				# Draw link B
				with dpg.draw_layer(tag="link_b"):
					b1 = a2
					b2 = calculate_end_point(b1, self.b*App.LINK_SCALE, self.theta2[index])
					dpg.draw_line(b1, b2, color=App.BLUE, thickness=App.LINK_THICKNESS)
					dpg.draw_circle(b1, 3, color=App.BLUE, thickness=App.CIRCLE_THICKNESS)
					dpg.draw_circle(b2, 3, color=App.BLUE, thickness=App.CIRCLE_THICKNESS)

				# Draw link C
				with dpg.draw_layer(tag="link_c"):
					c1 = d2
					c2 = b2
					dpg.draw_line(c1, c2, color=App.YELLOW, thickness=App.LINK_THICKNESS)
					dpg.draw_circle(c1, 3, color=App.YELLOW, thickness=App.CIRCLE_THICKNESS)
					dpg.draw_circle(c2, 3, color=App.YELLOW, thickness=App.CIRCLE_THICKNESS)

				# Draw the knife
				with dpg.draw_layer(tag="knife"):
					kl1 = c2
					kl2 = calculate_end_point(kl1, self.knife_offset*App.LINK_SCALE, self.theta3[index])
					dpg.draw_line(kl1, kl2, color=App.YELLOW, thickness=App.LINK_THICKNESS)

					k1 = kl2
					k2 = calculate_end_point(k1, 2*App.LINK_SCALE, self.theta3[index])
					k3 = calculate_end_point(k1, 0.5*App.LINK_SCALE, self.theta3[index] - 90)
					dpg.draw_triangle(k1, k2, k3, color=App.BROWN, fill=App.BROWN)

				# Draw the velocity arrows
				with dpg.draw_layer(tag="velocity_arrows"):
					vA = self.velocityA[index]
					vB = self.velocityB[index]
					vBA = self.velocityBA[index]

					vA1 = a2
					vA2 = calculate_end_point(vA1, magnitude(vA)*App.VELOCITY_SCALE, angle(vA))
					vB1 = b2
					vB2 = calculate_end_point(vB1, magnitude(vB)*App.VELOCITY_SCALE, angle(vB))
					vBA1 = c2
					vBA2 = calculate_end_point(vBA1, magnitude(vBA)*App.VELOCITY_SCALE, angle(vBA))

					dpg.draw_arrow(vA2, vA1, color=App.RED_50, thickness=App.VELOCITY_THICKNESS)
					dpg.draw_arrow(vB2, vB1, color=App.BLUE_50, thickness=App.VELOCITY_THICKNESS)
					dpg.draw_arrow(vBA2, vBA1, color=App.YELLOW_50, thickness=App.VELOCITY_THICKNESS)

		# Save the mechanism drawing window
		self.mechanism_drawing = mechanism_drawing

	# Create arcs showing movement of crank and rocker links
	# def create_paths(a, c, theta1, theta3):

	# Create settings window
	def create_settings(self):
		with dpg.window(
			label="Mechanism Settings",
			width=App.TAB_WIDTH,
			height=App.WINDOW_HEIGHT - App.TAB_HEIGHT,
			pos=(0, 0),
			no_resize=True,
			no_close=True,
			no_move=True,
			no_collapse=True,
		) as mechanism_settings:
			dpg.add_text("Link Lengths")
			dpg.add_slider_float(label="Input Link A (m)", default_value=self.a, max_value=10)
			dpg.add_slider_float(label="B (m)", default_value=self.b, max_value=10)
			dpg.add_slider_float(label="C (m)", default_value=self.c, max_value=10)
			dpg.add_slider_float(label="Ground Link D (m)", default_value=self.d, max_value=10)
			dpg.add_text("Ground Angle")
			dpg.add_slider_float(label="degree", default_value=self.theta4, min_value=0, max_value=360)
			dpg.add_text("Input Speed")
			dpg.add_slider_float(label="rpm", default_value=self.omega1, min_value=5, max_value=10)
			dpg.add_text("Analysis Settings")
			dpg.add_slider_int(label="Resolution", default_value=App.resolution, max_value=500)
			dpg.add_radio_button(["Open", "Crossed"], label="Position", default_value=0)
			dpg.add_button(label="Analyze")

		# Save the mechanism settings window
		self.mechanism_settings = mechanism_settings

	# Create position plot window
	def create_position_plot(self):
		with dpg.window(
			label="Position Analysis",
			width=App.TAB_WIDTH,
			height=App.TAB_HEIGHT,
			pos=(0, App.WINDOW_HEIGHT - App.TAB_HEIGHT),
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
				dpg.add_line_series(self.theta1, self.theta2, label="Theta 2", parent="position_y_axis")
				dpg.add_line_series(self.theta1, self.theta3, label="Theta 3", parent="position_y_axis")

		# Save the position plot window
		self.position_analysis = position_analysis

	# Create velocity plot window
	def create_velocity_plot(self):
		with dpg.window(
			label="Velocity Analysis",
			width=App.TAB_WIDTH,
			height=App.TAB_HEIGHT,
			pos=(App.TAB_WIDTH, App.WINDOW_HEIGHT - App.TAB_HEIGHT),
			no_resize=True,
			no_close=True,
			no_move=True,
			no_collapse=True,
		) as velocity_analysis:
			vA = [ magnitude(v) for v in self.velocityA ]
			vB = [ magnitude(v) for v in self.velocityB ]
			vBA = [ magnitude(v) for v in self.velocityBA ]

			with dpg.plot():
				dpg.add_plot_legend()
				dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
				dpg.set_axis_limits(dpg.last_item(), 0, 360)
				dpg.add_plot_axis(dpg.mvYAxis, label="Output Velocity (m/s)", tag="velocity_y_axis")
				dpg.add_line_series(self.theta1, vA, label="Velocity A", parent="velocity_y_axis")
				dpg.add_line_series(self.theta1, vB, label="Velocity B", parent="velocity_y_axis")
				dpg.add_line_series(self.theta1, vBA, label="Velocity BA", parent="velocity_y_axis")

		# Save the velocity plot window
		self.velocity_analysis = velocity_analysis

	# Create acceleration plot window
	def create_acceleration_plot(self):
		with dpg.window(
			label="Acceleration Analysis",
			width=App.TAB_WIDTH,
			height=App.TAB_HEIGHT,
			pos=(2*App.TAB_WIDTH, App.WINDOW_HEIGHT - App.TAB_HEIGHT),
			no_resize=True,
			no_close=True,
			no_move=True,
			no_collapse=True,
		) as acceleration_analysis:
			aA = [ magnitude(a) for a in self.accelerationA ]
			aB = [ magnitude(a) for a in self.accelerationB ]
			aBA = [ magnitude(a) for a in self.accelerationBA ]

			with dpg.plot():
				dpg.add_plot_legend()
				dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
				dpg.set_axis_limits(dpg.last_item(), 0, 360)
				dpg.add_plot_axis(dpg.mvYAxis, label="Output Acceleration (m/s^2)", tag="acceleration_y_axis")
				dpg.add_line_series(self.theta1, aA, label="Acceleration A", parent="acceleration_y_axis")
				dpg.add_line_series(self.theta1, aB, label="Acceleration B", parent="acceleration_y_axis")
				dpg.add_line_series(self.theta1, aBA, label="Acceleration BA", parent="acceleration_y_axis")

		# Save the acceleration plot window
		self.acceleration_analysis = acceleration_analysis
