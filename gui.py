import math
from lib import analyze_mechanism
from dearpygui import dearpygui as dpg
from helpers import angle, calculate_end_point, magnitude

class App:

	# Constants
	WINDOW_WIDTH = 1400		# Width of the window
	WINDOW_HEIGHT = 800		# Height of the window
	TAB_WIDTH = 400			# Width of the plot tabs at the bottom of the window
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
	DARK_GREY = (100, 100, 100, 255)

	# Sizes
	LINK_SCALE = 500
	LINK_THICKNESS = 3
	VELOCITY_SCALE = 80
	VELOCITY_THICKNESS = 6
	CIRCLE_THICKNESS = 8
	DIMENSION_OFFSET = 0.1
	DIMENSION_THICKNESS = 4

	# Link Lengths (m)
	a = 0.1
	b = 0.45
	c = 0.25
	d = 0.45
	knife_offset = 0.1

	# Kinematic Input
	theta4 = 0				# Ground Angle (deg)
	omega1 = 10				# Input Speed (rpm)
	alpha1 = 0				# Motor Acceleration (deg/s^2)

	# Analysis Settings
	current_angle = 180		# Current angle of the input link
	resolution = 360
	position = "crossed"
	running = True

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
		) = analyze_mechanism(
			a=self.a,
			b=self.b,
			c=self.c,
			d=self.d,
			knife_offset=self.knife_offset,
			theta4=self.theta4,
			omega1=self.omega1 * 6,
			alpha1=self.alpha1,
			offset=self._offset,
			link_scale=App.LINK_SCALE,
			resolution=self.resolution,
			position=self.position,
		)

		# Calculate maximum linear velocity
		max_velocity = max([magnitude(v) for v in velocityA] + [magnitude(v) for v in velocityB] + [magnitude(v) for v in velocityBA])

		# Calculate new scale
		velocity_scale = App.VELOCITY_SCALE / max_velocity

		# Update scales
		self._velocity_scale = velocity_scale

		# Update mechanism properties
		self._theta1 = theta1
		self._theta2 = theta2
		self._theta3 = theta3
		self._omega2 = omega2
		self._omega3 = omega3
		self._alpha2 = alpha2
		self._alpha3 = alpha3
		self._a1 = a1
		self._a2 = a2
		self._b1 = b1
		self._b2 = b2
		self._c1 = c1
		self._c2 = c2
		self._d1 = d1
		self._d2 = d2
		self._kl1 = kl1
		self._kl2 = kl2
		self._velocityA = velocityA
		self._velocityB = velocityB
		self._velocityBA = velocityBA
		self._accelerationA = accelerationA
		self._accelerationB = accelerationB
		self._accelerationBA = accelerationBA

	# Runs the app and starts the render loop
	def run(self):
		dpg.show_viewport()
		dpg.set_viewport_vsync(True)
		self._angle_index = self.current_angle % (self.resolution - 1)
		self.create_mechanism_drawing()
		while dpg.is_dearpygui_running():
			if self.running:
				new_angle = self.current_angle + (self.omega1 * 6 * dpg.get_delta_time()) % 360
				new_angle_index = math.floor(new_angle / (360 / self.resolution)) % (self.resolution - 1)
				self.current_angle = new_angle
				self._angle_index = new_angle_index
				self.update_mechanism()
			dpg.render_dearpygui_frame()
		dpg.destroy_context()

	# Toggles the running state of the app
	def toggle_running(self):
		self.running = not self.running
		dpg.configure_item("toggle_running", label="Pause" if self.running else "Resume")

	# Update the mechanism to the current angle
	def update_mechanism(self):
		# Get the current link positions
		a1 = self._a1[self._angle_index]
		a2 = self._a2[self._angle_index]
		b1 = self._b1[self._angle_index]
		b2 = self._b2[self._angle_index]
		c1 = self._c1[self._angle_index]
		c2 = self._c2[self._angle_index]
		d1 = self._d1[self._angle_index]
		d2 = self._d2[self._angle_index]
		kl1 = self._kl1[self._angle_index]
		kl2 = self._kl2[self._angle_index]

		# Update ground link (D)
		dpg.configure_item("link_d", p1=d1, p2=d2)
		dpg.configure_item("joint_d1", center=d1)
		dpg.configure_item("joint_d2", center=d2)

		# Update input link (A)
		dpg.configure_item("link_a", p1=a1, p2=a2)
		dpg.configure_item("joint_a1", center=a1)
		dpg.configure_item("joint_a2", center=a2)

		# Update link B
		dpg.configure_item("link_b", p1=b1, p2=b2)
		dpg.configure_item("joint_b1", center=b1)
		dpg.configure_item("joint_b2", center=b2)

		# Update link C
		dpg.configure_item("link_c", p1=c1, p2=c2)
		dpg.configure_item("joint_c1", center=c1)
		dpg.configure_item("joint_c2", center=c2)

		# Update the knife
		dpg.configure_item("knife_link", p1=kl1, p2=kl2)

		k1 = kl2
		k2 = calculate_end_point(k1, 0.2*App.LINK_SCALE, self._theta3[self._angle_index])
		k3 = calculate_end_point(k1, 0.05*App.LINK_SCALE, self._theta3[self._angle_index] - 90)
		dpg.configure_item("knife", p1=k1, p2=k2, p3=k3)

		# Update the velocity arrows
		vA = self._velocityA[self._angle_index]
		vB = self._velocityB[self._angle_index]
		vBA = self._velocityBA[self._angle_index]

		vA1 = a2
		vA2 = calculate_end_point(vA1, magnitude(vA)*self._velocity_scale, angle(vA))
		vB1 = b2
		vB2 = calculate_end_point(vB1, magnitude(vB)*self._velocity_scale, angle(vB))
		vBA1 = c2
		vBA2 = calculate_end_point(vBA1, magnitude(vBA)*self._velocity_scale, angle(vBA))

		dpg.configure_item("vA_arrow", p1=vA2, p2=vA1)
		dpg.configure_item("vB_arrow", p1=vB2, p2=vB1)
		dpg.configure_item("vBA_arrow", p1=vBA2, p2=vBA1)

		# Update the bounding box
		x_coords = [a1[0], a2[0], b1[0], b2[0], c1[0], c2[0], d1[0], d2[0], kl1[0], kl2[0]]
		min_x = min(x_coords)
		max_x = max(x_coords)

		y_coords = [a1[1], a2[1], b1[1], b2[1], c1[1], c2[1], d1[1], d2[1], kl1[1], kl2[1]]
		min_y = min(y_coords)
		max_y = max(y_coords)

		dx1 = (min_x, max_y + App.DIMENSION_OFFSET*App.LINK_SCALE)
		dx2 = (max_x, max_y + App.DIMENSION_OFFSET*App.LINK_SCALE)
		dpg.configure_item("dim_x", p1=dx1, p2=dx2)

		dy1 = (max_x + App.DIMENSION_OFFSET*App.LINK_SCALE, min_y)
		dy2 = (max_x + App.DIMENSION_OFFSET*App.LINK_SCALE, max_y)
		dpg.configure_item("dim_y", p1=dy1, p2=dy2)

		dx_pos = (min_x + ((dx2[0] - dx1[0]) / 2), max_y + App.DIMENSION_OFFSET*App.LINK_SCALE)
		dx_text = f"{round((dx2[0] - dx1[0]) / App.LINK_SCALE, 2)} m"
		dpg.configure_item("dim_x_text", pos=dx_pos, text=dx_text)

		dy_pos = (max_x + App.DIMENSION_OFFSET*App.LINK_SCALE, min_y + ((dy2[1] - dy1[1]) / 2))
		dy_text = f"{round((dy2[1] - dy1[1]) / App.LINK_SCALE, 2)} m"
		dpg.configure_item("dim_y_text", pos=dy_pos, text=dy_text)

	# Create mechanism drawing window
	def create_mechanism_drawing(self):
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
			with dpg.drawlist(width=App.WINDOW_WIDTH, height=App.WINDOW_HEIGHT - App.TAB_HEIGHT) as mechanism_canvas:

				# Get the current link positions
				a1 = self._a1[self._angle_index]
				a2 = self._a2[self._angle_index]
				b1 = self._b1[self._angle_index]
				b2 = self._b2[self._angle_index]
				c1 = self._c1[self._angle_index]
				c2 = self._c2[self._angle_index]
				d1 = self._d1[self._angle_index]
				d2 = self._d2[self._angle_index]
				kl1 = self._kl1[self._angle_index]
				kl2 = self._kl2[self._angle_index]

				# Draw ground link (D)
				with dpg.draw_layer(tag="d"):
					dpg.draw_line(d1, d2, tag="link_d", color=App.GREY, thickness=App.LINK_THICKNESS)
					dpg.draw_circle(d1, 3, tag="joint_d1", color=App.GREY, thickness=App.CIRCLE_THICKNESS)
					dpg.draw_circle(d2, 3, tag="joint_d2", color=App.GREY, thickness=App.CIRCLE_THICKNESS)

				# Draw input link (A)
				with dpg.draw_layer(tag="a"):
					dpg.draw_line(a1, a2, tag="link_a", color=App.RED, thickness=App.LINK_THICKNESS)
					dpg.draw_circle(a1, 3, tag="joint_a1", color=App.RED, thickness=App.CIRCLE_THICKNESS)
					dpg.draw_circle(a2, 3, tag="joint_a2", color=App.RED, thickness=App.CIRCLE_THICKNESS)

				# Draw link B
				with dpg.draw_layer(tag="b"):
					dpg.draw_line(b1, b2, tag="link_b", color=App.BLUE, thickness=App.LINK_THICKNESS)
					dpg.draw_circle(b1, 3, tag="joint_b1", color=App.BLUE, thickness=App.CIRCLE_THICKNESS)
					dpg.draw_circle(b2, 3, tag="joint_b2", color=App.BLUE, thickness=App.CIRCLE_THICKNESS)

				# Draw link C
				with dpg.draw_layer(tag="c"):
					dpg.draw_line(c1, c2, tag="link_c", color=App.YELLOW, thickness=App.LINK_THICKNESS)
					dpg.draw_circle(c1, 3, tag="joint_c1", color=App.YELLOW, thickness=App.CIRCLE_THICKNESS)
					dpg.draw_circle(c2, 3, tag="joint_c2", color=App.YELLOW, thickness=App.CIRCLE_THICKNESS)

				# Draw the knife
				with dpg.draw_layer(tag="k"):
					dpg.draw_line(kl1, kl2, tag="knife_link", color=App.YELLOW, thickness=App.LINK_THICKNESS)

					k1 = kl1
					k2 = calculate_end_point(k1, 0.2*App.LINK_SCALE, self._theta3[self._angle_index])
					k3 = calculate_end_point(k1, 0.05*App.LINK_SCALE, self._theta3[self._angle_index] - 90)
					dpg.draw_triangle(k1, k2, k3, tag="knife", color=App.BROWN, fill=App.BROWN)

				# Draw the velocity arrows
				with dpg.draw_layer(tag="velocity_arrows"):
					vA = self._velocityA[self._angle_index]
					vB = self._velocityB[self._angle_index]
					vBA = self._velocityBA[self._angle_index]

					vA1 = a2
					vA2 = calculate_end_point(vA1, magnitude(vA)*self._velocity_scale, angle(vA))
					vBA1 = b2
					vBA2 = calculate_end_point(vBA1, magnitude(vBA)*self._velocity_scale, angle(vBA))
					vB1 = c2
					vB2 = calculate_end_point(vB1, magnitude(vB)*self._velocity_scale, angle(vB))

					dpg.draw_arrow(vA2, vA1, tag="vA_arrow", color=App.RED_50, thickness=App.VELOCITY_THICKNESS)
					dpg.draw_arrow(vBA2, vBA1, tag="vBA_arrow", color=App.BLUE_50, thickness=App.VELOCITY_THICKNESS)
					dpg.draw_arrow(vB2, vB1, tag="vB_arrow", color=App.YELLOW_50, thickness=App.VELOCITY_THICKNESS)

				# Draw the bounding box
				with dpg.draw_layer(tag="bounding_box"):
					x_coords = [a1[0], a2[0], b1[0], b2[0], c1[0], c2[0], d1[0], d2[0], kl1[0], kl2[0]]
					min_x = min(x_coords)
					max_x = max(x_coords)

					y_coords = [a1[1], a2[1], b1[1], b2[1], c1[1], c2[1], d1[1], d2[1], kl1[1], kl2[1]]
					min_y = min(y_coords)
					max_y = max(y_coords)

					dx1 = (min_x, max_y + App.DIMENSION_OFFSET*App.LINK_SCALE)
					dx2 = (max_x, max_y + App.DIMENSION_OFFSET*App.LINK_SCALE)
					dpg.draw_line(
						dx1, dx2,
						tag="dim_x",
						color=App.DARK_GREY,
						thickness=App.DIMENSION_THICKNESS
					)

					dy1 = (max_x + App.DIMENSION_OFFSET*App.LINK_SCALE, min_y)
					dy2 = (max_x + App.DIMENSION_OFFSET*App.LINK_SCALE, max_y)
					dpg.draw_line(
						dy1, dy2,
						tag="dim_y",
						color=App.DARK_GREY,
						thickness=App.DIMENSION_THICKNESS
					)

					dx_pos = (min_x + ((dx2[0] - dx1[0]) / 2), max_y + App.DIMENSION_OFFSET*App.LINK_SCALE)
					dx_text = f"{round((dx2[0] - dx1[0]) / App.LINK_SCALE, 2)} m"
					dpg.draw_text(
						dx_pos, dx_text,
						tag="dim_x_text",
						size=14,
					)
					
					dy_pos = (max_x + App.DIMENSION_OFFSET*App.LINK_SCALE, min_y + ((dy2[1] - dy1[1]) / 2))
					dy_text = f"{round((dy2[1] - dy1[1]) / App.LINK_SCALE, 2)} m"
					dpg.draw_text(
						dy_pos, dy_text,
						tag="dim_y_text",
						size=14,
					)

		# Save the mechanism drawing window
		self._mechanism_drawing = mechanism_drawing
		self._mechanism_canvas = mechanism_canvas

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
			dpg.add_text("Link Lengths (m)")
			dpg.add_slider_float(label="Input Link A", default_value=self.a, max_value=1)
			dpg.add_slider_float(label="B", default_value=self.b, max_value=1)
			dpg.add_slider_float(label="C", default_value=self.c, max_value=1)
			dpg.add_slider_float(label="Ground Link D", default_value=self.d, max_value=1)
			dpg.add_text("Kinematic Input")
			dpg.add_slider_float(label="Ground Angle (deg)", default_value=self.theta4, min_value=0, max_value=360)
			dpg.add_slider_float(label="Input Speed (rpm)", default_value=self.omega1, min_value=5, max_value=30)
			dpg.add_slider_float(label="Motor Acc. (deg/s^2)", default_value=self.alpha1, min_value=0, max_value=5)
			dpg.add_text("Analysis Settings")
			dpg.add_slider_float(label="Input Angle", default_value=self.current_angle, min_value=0, max_value=360)
			dpg.add_slider_int(label="Resolution", default_value=App.resolution, max_value=500)
			dpg.add_radio_button(["Open", "Crossed"], label="Position", default_value=0)
			dpg.add_button(label="Update and Analyze")
			dpg.add_button(label="Pause" if self.running else "Resume", tag="toggle_running", callback=self.toggle_running)

		# Save the mechanism settings window
		self._mechanism_settings = mechanism_settings

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
				dpg.add_line_series(self._theta1, self._theta2, label="Theta 2", parent="position_y_axis")
				dpg.add_line_series(self._theta1, self._theta3, label="Theta 3", parent="position_y_axis")

		# Save the position plot window
		self._position_analysis = position_analysis

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
			vA = [ magnitude(v) for v in self._velocityA ]
			vB = [ magnitude(v) for v in self._velocityB ]
			vBA = [ magnitude(v) for v in self._velocityBA ]

			with dpg.plot():
				dpg.add_plot_legend()
				dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
				dpg.set_axis_limits(dpg.last_item(), 0, 360)
				dpg.add_plot_axis(dpg.mvYAxis, label="Output Velocity (m/s)", tag="velocity_y_axis")
				dpg.add_line_series(self._theta1, vA, label="Velocity A", parent="velocity_y_axis")
				dpg.add_line_series(self._theta1, vB, label="Velocity B", parent="velocity_y_axis")
				dpg.add_line_series(self._theta1, vBA, label="Velocity BA", parent="velocity_y_axis")

		# Save the velocity plot window
		self._velocity_analysis = velocity_analysis

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
			aA = [ magnitude(a) for a in self._accelerationA ]
			aB = [ magnitude(a) for a in self._accelerationB ]
			aBA = [ magnitude(a) for a in self._accelerationBA ]

			with dpg.plot():
				dpg.add_plot_legend()
				dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
				dpg.set_axis_limits(dpg.last_item(), 0, 360)
				dpg.add_plot_axis(dpg.mvYAxis, label="Output Acceleration (m/s^2)", tag="acceleration_y_axis")
				dpg.add_line_series(self._theta1, aA, label="Acceleration A", parent="acceleration_y_axis")
				dpg.add_line_series(self._theta1, aB, label="Acceleration B", parent="acceleration_y_axis")
				dpg.add_line_series(self._theta1, aBA, label="Acceleration BA", parent="acceleration_y_axis")

		# Save the acceleration plot window
		self._acceleration_analysis = acceleration_analysis

	# Windows
	_mechanism_settings = None
	_mechanism_drawing = None
	_mechanism_canvas = None
	_position_analysis = None
	_velocity_analysis = None
	_acceleration_analysis = None

	# Analysis Settings
	_velocity_scale = VELOCITY_SCALE
	_offset = (WINDOW_WIDTH * 0.3, (WINDOW_HEIGHT - TAB_HEIGHT) * 0.75)
	_angle_index = 0

	# Analysis Data
	_a1 = []
	_a2 = []
	_b1 = []
	_b2 = []
	_c1 = []
	_c2 = []
	_d1 = []
	_d2 = []
	_kl1 = []
	_kl2 = []
	_theta1 = []
	_theta2 = []
	_theta3 = []
	_omega2 = []
	_omega3 = []
	_omega4 = []
	_velocityA = (0, 0)
	_velocityB = (0, 0)
	_velocityBA = (0, 0)
	_alpha2 = []
	_alpha3 = []
	_alpha4 = []
	_accelerationA = (0, 0)
	_accelerationB = (0, 0)
	_accelerationBA = (0, 0)

App().run()
