import math
from lib import analyze_mechanism
from dearpygui import dearpygui as dpg
from helpers import angle, calculate_end_point, magnitude, rotate

class App:

	# Constants
	WINDOW_WIDTH = 1400		# Width of the window
	WINDOW_HEIGHT = 700		# Height of the window
	TAB_WIDTH = 450			# Width of the plot tabs at the bottom of the window
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
	DARK_GREY = (50, 50, 50, 255)

	# Sizes
	LINK_SCALE = 500
	LINK_THICKNESS = 3
	VELOCITY_SCALE = 80
	VELOCITY_THICKNESS = 6
	CIRCLE_THICKNESS = 8
	DIMENSION_THICKNESS = 2

	# Link Lengths (m)
	a = 0.1524
	b = 0.4064
	c = 0.3048
	d = 0.4572
	knife_offset = 0.1

	# Kinematic Input
	theta4 = 0				# Ground Angle (deg)
	omega1 = 120			# Input Speed (rpm)
	alpha1 = 0				# Motor Acceleration (deg/s^2)

	# Analysis Settings
	current_angle = 30		# Current angle of the input link
	resolution = 360
	position = "crossed"
	running = True

	# Constructor for the app, sets up the PyGUI context and viewport
	def __init__(self):
		# Analyze the initial mechanism
		self.analyze_mechanism()

		# Setup the app
		dpg.create_context()
		dpg.create_viewport(title="Mechanism Analysis and Simulation", width=App.WINDOW_WIDTH, height=App.WINDOW_HEIGHT, )
		dpg.setup_dearpygui()
		
		# Create the windows
		self.create_settings()
		self.create_mechanism_drawing()
		self.create_position_plot()
		self.create_velocity_plot()
		self.create_acceleration_plot()
		self.create_performance_plot()

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
		) = analyze_mechanism(
			a=self.a,
			b=self.b,
			c=self.c,
			d=self.d,
			knife_offset=self.knife_offset,
			theta4=self.theta4,
			omega1=self.omega1,
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
		self._theta1 = theta1						# deg
		self._theta2 = theta2						# deg
		self._theta3 = theta3						# deg
		self._omega2 = [ o / 6 for o in omega2 ]	# rpm
		self._omega3 = [ o / 6 for o in omega3 ]	# rpm
		self._alpha2 = alpha2						# deg/s^2
		self._alpha3 = alpha3						# deg/s^2
		self._a1 = a1								# m
		self._a2 = a2								# m
		self._b1 = b1								# m
		self._b2 = b2								# m
		self._c1 = c1								# m
		self._c2 = c2								# m
		self._d1 = d1								# m
		self._d2 = d2								# m
		self._kl1 = kl1								# m
		self._kl2 = kl2								# m
		self._velocityA = velocityA					# m/s
		self._velocityB = velocityB					# m/s
		self._velocityBA = velocityBA				# m/s
		self._accelerationA = accelerationA			# m/s^2
		self._accelerationB = accelerationB			# m/s^2
		self._accelerationBA = accelerationBA		# m/s^2
		self._cgAccelerationA = cgAccelerationA		# m/s^2
		self._cgAccelerationB = cgAccelerationB		# m/s^2
		self._cgAccelerationBA = cgAccelerationBA	# m/s^2
		self._transmission_angle = transmission_angle
		self._velocity_ratio = velocity_ratio
		self._mechanical_advantage = mechanical_advantage

	# Runs the app and starts the render loop
	def run(self):
		dpg.show_viewport()
		dpg.set_viewport_vsync(True)
		self._angle_index = self.current_angle % (self.resolution - 1)
		self.update_mechanism()
		self.update_plots()
		while dpg.is_dearpygui_running():
			if self.running:
				new_angle = (self.current_angle + (self.omega1 * 6 * dpg.get_delta_time())) % 360
				new_angle_index = math.floor(new_angle / (360 / self.resolution)) % (self.resolution - 1)
				self.current_angle = new_angle
				self._angle_index = new_angle_index
				self.update_mechanism()
				self.update_plots()
			dpg.render_dearpygui_frame()
		dpg.destroy_context()

	# Handler for the update and analyze button
	def handle_update_and_analyze(self):
		self.a = dpg.get_value("a_input")
		self.b = dpg.get_value("b_input")
		self.c = dpg.get_value("c_input")
		self.d = dpg.get_value("d_input")
		self.theta4 = dpg.get_value("theta4_input")
		self.omega1 = dpg.get_value("omega1_input")
		self.alpha1 = dpg.get_value("alpha1_input")
		self.current_angle = dpg.get_value("theta1_input")
		self.resolution = dpg.get_value("resolution_input")
		self.position = str.lower(dpg.get_value("pos_input"))
		self.analyze_mechanism()
		self.update_mechanism()
		self.update_plots()

	# Toggles the running state of the app
	def handle_toggle_running(self):
		self.running = not self.running
		dpg.configure_item("toggle_running", label="Pause" if self.running else "Resume")

	# Handler for the drag position indicator
	def handle_drag_pos_indicator(self, tag):
		new_angle = dpg.get_value(tag)
		new_angle_index = math.floor(new_angle / (360 / self.resolution)) % (self.resolution - 1)
		self.current_angle = new_angle
		self._angle_index = new_angle_index
		self.running = False
		self.update_mechanism()
		self.update_plots()

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

		dx1 = (min_x, self._all_max_y)
		dx2 = (max_x, self._all_max_y)
		dpg.configure_item("dim_x", p1=dx1, p2=dx2)

		dy1 = (self._all_max_x, min_y)
		dy2 = (self._all_max_x, max_y)
		dpg.configure_item("dim_y", p1=dy1, p2=dy2)

		dx_pos = (min_x + ((dx2[0] - dx1[0]) / 2), self._all_max_y)
		dx_text = f"{round((dx2[0] - dx1[0]) / App.LINK_SCALE, 2)} m"
		dpg.configure_item("dim_x_text", pos=dx_pos, text=dx_text)

		dy_pos = (self._all_max_x, min_y + ((dy2[1] - dy1[1]) / 2))
		dy_text = f"{round((dy2[1] - dy1[1]) / App.LINK_SCALE, 2)} m"
		dpg.configure_item("dim_y_text", pos=dy_pos, text=dy_text)

	# Update the graphical plots based on the current angle
	def update_plots(self):
		# Update position values
		t1 = round(self._theta1[self._angle_index], 2)
		t2 = round(self._theta2[self._angle_index], 2)
		t3 = round(self._theta3[self._angle_index], 2)
		dpg.configure_item("theta1_value", default_value=f"Theta 1 (Input Angle): {t1} deg")
		dpg.configure_item("theta2_value", default_value=f"Theta 2: {t2} deg")
		dpg.configure_item("theta3_value", default_value=f"Theta 3: {t3} deg")

		# Update velocity values
		omega2 = round(self._omega2[self._angle_index], 2)
		omega3 = round(self._omega3[self._angle_index], 2)
		dpg.configure_item("omega2_value", default_value=f"Omega 2: {omega2} rpm")
		dpg.configure_item("omega3_value", default_value=f"Omega 3: {omega3} rpm")

		vBA = round(magnitude(self._velocityBA[self._angle_index]), 2)
		vB = round(magnitude(self._velocityB[self._angle_index]), 2)
		dpg.configure_item("mag_vBA_value", default_value=f"Magnitudal Velocity 2: {vBA} m/s")
		dpg.configure_item("mag_vB_value", default_value=f"Magnitudal Velocity 3: {vB} m/s")

		tang_vBA = round(rotate(self._velocityBA[self._angle_index], self._theta2[self._angle_index])[1], 2)
		tang_vB = round(rotate(self._velocityB[self._angle_index], self._theta3[self._angle_index])[1], 2)
		dpg.configure_item("tang_vBA_value", default_value=f"Tangential Velocity 2: {tang_vBA} m/s")
		dpg.configure_item("tang_vB_value", default_value=f"Tangential Velocity 3: {tang_vB} m/s")

		# Update acceleration values
		alpha2 = round(self._alpha2[self._angle_index], 2)
		alpha3 = round(self._alpha3[self._angle_index], 2)
		dpg.configure_item("alpha2_value", default_value=f"Alpha 2: {alpha2} deg/s^2")
		dpg.configure_item("alpha3_value", default_value=f"Alpha 3: {alpha3} deg/s^2")

		aBA = round(magnitude(self._accelerationBA[self._angle_index]), 2)
		aB = round(magnitude(self._accelerationB[self._angle_index]), 2)
		dpg.configure_item("mag_aBA_value", default_value=f"Magnitudal Acceleration 2: {aBA} m/s^2")
		dpg.configure_item("mag_aB_value", default_value=f"Magnitudal Acceleration 3: {aB} m/s^2")

		tang_aBA = round(rotate(self._accelerationBA[self._angle_index], self._theta2[self._angle_index])[1], 2)
		radial_aBA = round(rotate(self._accelerationBA[self._angle_index], self._theta2[self._angle_index])[0], 2)
		tang_aB = round(rotate(self._accelerationB[self._angle_index], self._theta3[self._angle_index])[1], 2)
		radial_aB = round(rotate(self._accelerationB[self._angle_index], self._theta3[self._angle_index])[0], 2)
		dpg.configure_item("tang_aBA_value", default_value=f"Tangential Acceleration 2: {tang_aBA} m/s^2")
		dpg.configure_item("radial_aBA_value", default_value=f"Radial Acceleration 2: {radial_aBA} m/s^2")
		dpg.configure_item("tang_aB_value", default_value=f"Tangential Acceleration 3: {tang_aB} m/s^2")
		dpg.configure_item("radial_aB_value", default_value=f"Radial Acceleration 3: {radial_aB} m/s^2")

		cg_aBA = round(magnitude(self._cgAccelerationBA[self._angle_index]), 2)
		cg_aB = round(magnitude(self._cgAccelerationB[self._angle_index]), 2)
		dpg.configure_item("mag_cg_aBA_value", default_value=f"CG Magnitudal Acceleration 2: {cg_aBA} m/s^2")
		dpg.configure_item("mag_cg_aB_value", default_value=f"CG Magnitudal Acceleration 3: {cg_aB} m/s^2")

		cg_aBA_x = round(self._cgAccelerationBA[self._angle_index][0], 2)
		cg_aBA_y = round(self._cgAccelerationBA[self._angle_index][1], 2)
		cg_aB_x = round(self._cgAccelerationB[self._angle_index][0], 2)
		cg_aB_y = round(self._cgAccelerationB[self._angle_index][1], 2)
		dpg.configure_item("cg_aBA_x_value", default_value=f"CG Acceleration 2 in X: {cg_aBA_x} m/s^2")
		dpg.configure_item("cg_aBA_y_value", default_value=f"CG Acceleration 2 in Y: {cg_aBA_y} m/s^2")
		dpg.configure_item("cg_aB_x_value", default_value=f"CG Acceleration 3 in X: {cg_aB_x} m/s^2")
		dpg.configure_item("cg_aB_y_value", default_value=f"CG Acceleration 3 in Y: {cg_aB_y} m/s^2")

		# Update performance values
		transmission_angle = round(self._transmission_angle[self._angle_index], 2)
		velocity_ratio = round(self._velocity_ratio[self._angle_index], 2)
		mechanical_advantage = round(self._mechanical_advantage[self._angle_index], 2)
		dpg.configure_item("transmission_angle_value", default_value=f"Transmission Angle: {transmission_angle} deg")
		dpg.configure_item("velocity_ratio_value", default_value=f"Velocity Ratio: {velocity_ratio}")
		dpg.configure_item("mechanical_advantage_value", default_value=f"Mechanical Advantage: {mechanical_advantage}")

		# Update the current angle indicators
		dpg.configure_item("pos_indicator", default_value=self.current_angle)
		dpg.configure_item("angular_velocity_indicator", default_value=self.current_angle)
		dpg.configure_item("mag_velocity_indicator", default_value=self.current_angle)
		dpg.configure_item("comp_velocity_indicator", default_value=self.current_angle)
		dpg.configure_item("angular_acceleration_indicator", default_value=self.current_angle)
		dpg.configure_item("mag_acceleration_indicator", default_value=self.current_angle)
		dpg.configure_item("comp_acceleration_indicator", default_value=self.current_angle)
		dpg.configure_item("mag_cg_acceleration_indicator", default_value=self.current_angle)
		dpg.configure_item("comp_cg_acceleration_indicator", default_value=self.current_angle)
		dpg.configure_item("transmission_indicator", default_value=self.current_angle)
		dpg.configure_item("velocity_ratio_indicator", default_value=self.current_angle)
		dpg.configure_item("mechanical_advantage_indicator", default_value=self.current_angle)

	# Create mechanism drawing window
	def create_mechanism_drawing(self):
		with dpg.window(
			tag="mechanism_drawing",
			label="Mechanism Drawing",
			width=App.WINDOW_WIDTH,
			height=App.WINDOW_HEIGHT,
			pos=(App.TAB_WIDTH, 0),
			no_resize=True,
			no_close=True,
			no_move=True,
			no_collapse=True,
		) as mechanism_drawing:
			with dpg.drawlist(width=App.WINDOW_WIDTH, height=App.WINDOW_HEIGHT) as mechanism_canvas:

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
					all_x_coords = [pos[0] for joints in [self._a1, self._a2, self._b1, self._b2, self._c1, self._c2, self._d1, self._d2, self._kl1, self._kl2] for pos in joints] # Flatten the list
					self._all_min_x = min(all_x_coords)
					self._all_max_x = max(all_x_coords)

					all_y_coords = [pos[1] for joints in [self._a1, self._a2, self._b1, self._b2, self._c1, self._c2, self._d1, self._d2, self._kl1, self._kl2] for pos in joints] # Flatten the list
					self._all_min_y = min(all_y_coords)
					self._all_max_y = max(all_y_coords)

					x_coords = [a1[0], a2[0], b1[0], b2[0], c1[0], c2[0], d1[0], d2[0], kl1[0], kl2[0]]
					min_x = min(x_coords)
					max_x = max(x_coords)

					y_coords = [a1[1], a2[1], b1[1], b2[1], c1[1], c2[1], d1[1], d2[1], kl1[1], kl2[1]]
					min_y = min(y_coords)
					max_y = max(y_coords)

					dpg.draw_rectangle((self._all_min_x, self._all_min_y), (self._all_max_x, self._all_max_y), color=App.DARK_GREY, thickness=App.DIMENSION_THICKNESS)

					dx1 = (min_x, self._all_max_y)
					dx2 = (max_x, self._all_max_y)
					dpg.draw_line(dx1, dx2, tag="dim_x", color=App.DARK_GREY, thickness=App.DIMENSION_THICKNESS * 4)

					dy1 = (self._all_max_x, min_y)
					dy2 = (self._all_max_x, max_y)
					dpg.draw_line(dy1, dy2, tag="dim_y", color=App.DARK_GREY, thickness=App.DIMENSION_THICKNESS * 4)

					dx_pos = (min_x + ((dx2[0] - dx1[0]) / 2), self._all_max_y)
					dx_text = f"{round((dx2[0] - dx1[0]) / App.LINK_SCALE, 2)} m"
					dpg.draw_text(dx_pos, dx_text, tag="dim_x_text", size=14)
					
					dy_pos = (self._all_max_x, min_y + ((dy2[1] - dy1[1]) / 2))
					dy_text = f"{round((dy2[1] - dy1[1]) / App.LINK_SCALE, 2)} m"
					dpg.draw_text(dy_pos, dy_text, tag="dim_y_text", size=14)

		# Save the mechanism drawing window
		self._mechanism_drawing = mechanism_drawing
		self._mechanism_canvas = mechanism_canvas

	# Create arcs showing movement of crank and rocker links
	# def create_paths(a, c, theta1, theta3):

	# Create settings window
	def create_settings(self):
		with dpg.window(
			label="Inspector",
			tag="inspector",
			width=App.TAB_WIDTH,
			height=App.WINDOW_HEIGHT,
			pos=(0, 0),
			no_resize=True,
			no_close=True,
			no_move=True,
			no_collapse=True,
		):
			dpg.add_text("Link Lengths (m)")
			dpg.add_slider_float(label="Input Link A", tag="a_input", default_value=self.a, max_value=1)
			dpg.add_slider_float(label="B", tag="b_input", default_value=self.b, max_value=1)
			dpg.add_slider_float(label="C", tag="c_input", default_value=self.c, max_value=1)
			dpg.add_slider_float(label="Ground Link D", tag="d_input", default_value=self.d, max_value=1)
			dpg.add_text("Kinematic Input")
			dpg.add_slider_float(label="Ground Angle (deg)", tag="theta4_input", default_value=self.theta4, min_value=0, max_value=360)
			dpg.add_slider_float(label="Input Speed (rpm)", tag="omega1_input", default_value=self.omega1, min_value=-15, max_value=15)
			dpg.add_slider_float(label="Motor Acc. (deg/s^2)", tag="alpha1_input", default_value=self.alpha1, min_value=0, max_value=5)
			dpg.add_text("Analysis Settings")
			dpg.add_slider_float(label="Input Angle", tag="theta1_input", default_value=self.current_angle, min_value=0, max_value=360)
			dpg.add_slider_int(label="Resolution", tag="resolution_input", default_value=App.resolution, max_value=500)
			dpg.add_radio_button(["Open", "Crossed"], tag="pos_input", label="Position", default_value=str.capitalize(self.position))
			dpg.add_button(label="Update and Analyze", callback=self.handle_update_and_analyze)
			dpg.add_button(label="Pause" if self.running else "Resume", tag="toggle_running", callback=self.handle_toggle_running)

	# Create position plot window
	def create_position_plot(self):
		dpg.add_collapsing_header(label="Position Analysis", tag="position_analysis", parent="inspector")
		with dpg.group(parent="position_analysis"):
			t1 = round(self._theta1[self._angle_index], 2)
			t2 = round(self._theta2[self._angle_index], 2)
			t3 = round(self._theta3[self._angle_index], 2)
			t4 = round(self.theta4, 2)

			dpg.add_text(f"Theta 1 (Input Angle): {t1} deg", tag="theta1_value")
			dpg.add_text(f"Theta 2: {t2} deg", tag="theta2_value")
			dpg.add_text(f"Theta 3: {t3} deg", tag="theta3_value")
			dpg.add_text(f"Theta 4 (Ground Angle): {t4} deg", tag="theta4_value")

			dpg.add_text(f"Range of Theta 2: {round(min(self._theta2), 2)} to {round(max(self._theta2), 2)} deg")
			dpg.add_text(f"Range of Theta 3: {round(min(self._theta3), 2)} to {round(max(self._theta3), 2)} deg")

			dpg.add_text(f"Bounding Box: {round((self._all_max_x - self._all_min_x) / App.LINK_SCALE, 2)} m x {round((self._all_max_y - self._all_min_y) / App.LINK_SCALE, 2)} m")
			with dpg.plot():
				dpg.add_plot_legend()
				dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
				dpg.set_axis_limits(dpg.last_item(), 0, 360)
				dpg.add_plot_axis(dpg.mvYAxis, label="Output Angle (degree)", tag="position_y_axis")
				dpg.add_line_series(self._theta1, self._theta2, label="Theta 2", parent="position_y_axis")
				dpg.add_line_series(self._theta1, self._theta3, label="Theta 3", parent="position_y_axis")
				dpg.add_drag_line(label="Current Angle", tag="pos_indicator", default_value=self.current_angle, callback=self.handle_drag_pos_indicator)

	# Create velocity plot window
	def create_velocity_plot(self):
		dpg.add_collapsing_header(label="Velocity Analysis", tag="velocity_analysis", parent="inspector")
		with dpg.group(parent="velocity_analysis"):
			omega1 = round(self.omega1, 2)
			omega2 = round(self._omega2[self._angle_index], 2)
			omega3 = round(self._omega3[self._angle_index], 2)

			dpg.add_text(f"Omega 1 (Input Speed): {omega1} rpm", tag="omega1_value")
			dpg.add_text(f"Omega 2: {omega2} rpm", tag="omega2_value")
			dpg.add_text(f"Omega 3: {omega3} rpm", tag="omega3_value")

			dpg.add_text("Angular Velocity (rpm)")
			with dpg.plot():
				dpg.add_plot_legend()
				dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
				dpg.set_axis_limits(dpg.last_item(), 0, 360)
				dpg.add_plot_axis(dpg.mvYAxis, label="Output Angular Velocity (rpm)", tag="angular_velocity_y_axis")
				dpg.add_line_series(self._theta1, self._omega2, label="Omega 2", parent="angular_velocity_y_axis")
				dpg.add_line_series(self._theta1, self._omega3, label="Omega 3", parent="angular_velocity_y_axis")
				dpg.add_drag_line(label="Current Angle", tag="angular_velocity_indicator", default_value=self.current_angle, callback=self.handle_drag_pos_indicator)

			vA = round(magnitude(self._velocityA[self._angle_index]), 2)
			vBA = round(magnitude(self._velocityBA[self._angle_index]), 2)
			vB = round(magnitude(self._velocityB[self._angle_index]), 2)

			vA_list = [ magnitude(v) for v in self._velocityA ]
			vBA_list = [ magnitude(v) for v in self._velocityBA ]
			vB_list = [ magnitude(v) for v in self._velocityB ]

			dpg.add_text(f"Magnitudal Velocity 1 (Input Speed): {vA} m/s", tag="mag_vA_value")
			dpg.add_text(f"Magnitudal Velocity 2: {vBA} m/s", tag="mag_vBA_value")
			dpg.add_text(f"Magnitudal Velocity 3: {vB} m/s", tag="mag_vB_value")

			dpg.add_text("Magnitudal Velocity (m/s)")
			with dpg.plot():
				dpg.add_plot_legend()
				dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
				dpg.set_axis_limits(dpg.last_item(), 0, 360)
				dpg.add_plot_axis(dpg.mvYAxis, label="Output Magnitudal Velocity (m/s)", tag="mag_velocity_y_axis")
				dpg.add_line_series(self._theta1, vA_list, label="Velocity 1", parent="mag_velocity_y_axis")
				dpg.add_line_series(self._theta1, vBA_list, label="Velocity 2", parent="mag_velocity_y_axis")
				dpg.add_line_series(self._theta1, vB_list, label="Velocity 3", parent="mag_velocity_y_axis")
				dpg.add_drag_line(label="Current Angle", tag="mag_velocity_indicator", default_value=self.current_angle, callback=self.handle_drag_pos_indicator)

			tang_vA = round(rotate(self._velocityA[self._angle_index], self._theta1[self._angle_index])[1], 2)
			tang_vBA = round(rotate(self._velocityBA[self._angle_index], self._theta2[self._angle_index])[1], 2)
			tang_vB = round(rotate(self._velocityB[self._angle_index], self._theta3[self._angle_index])[1], 2)

			tang_vA_list = [ rotate(self._velocityA[i], self._theta1[i])[1] for i in range(len(self._velocityA)) ]
			tang_vBA_list = [ rotate(self._velocityBA[i], self._theta2[i])[1] for i in range(len(self._velocityBA)) ]
			tang_vB_list = [ rotate(self._velocityB[i], self._theta3[i])[1] for i in range(len(self._velocityB)) ]

			dpg.add_text(f"Tangential Velocity 1: {tang_vA} m/s^2", tag="tang_vA_value")
			dpg.add_text(f"Tangential Velocity 2: {tang_vBA} m/s^2", tag="tang_vBA_value")
			dpg.add_text(f"Tangential Velocity 3: {tang_vB} m/s^2", tag="tang_vB_value")

			dpg.add_text("Velocity Components (m/s)")
			with dpg.plot():
				dpg.add_plot_legend()
				dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
				dpg.set_axis_limits(dpg.last_item(), 0, 360)
				dpg.add_plot_axis(dpg.mvYAxis, label="Output Velocity Components (m/s)", tag="comp_velocity_y_axis")
				dpg.add_line_series(self._theta1, tang_vA_list, label="Tangential 1", parent="comp_velocity_y_axis")
				dpg.add_line_series(self._theta1, tang_vBA_list, label="Tangential 2", parent="comp_velocity_y_axis")
				dpg.add_line_series(self._theta1, tang_vB_list, label="Tangential 3", parent="comp_velocity_y_axis")
				dpg.add_drag_line(label="Current Angle", tag="comp_velocity_indicator", default_value=self.current_angle, callback=self.handle_drag_pos_indicator)

	# Create acceleration plot window
	def create_acceleration_plot(self):
		dpg.add_collapsing_header(label="Acceleration Analysis", tag="acceleration_analysis", parent="inspector")
		with dpg.group(parent="acceleration_analysis"):
			alpha1 = round(self.alpha1, 2)
			alpha2 = round(self._alpha2[self._angle_index], 2)
			alpha3 = round(self._alpha3[self._angle_index], 2)

			dpg.add_text(f"Alpha 1 (Input Speed): {alpha1} rpm", tag="alpha1_value")
			dpg.add_text(f"Alpha 2: {alpha2} rpm", tag="alpha2_value")
			dpg.add_text(f"Alpha 3: {alpha3} rpm", tag="alpha3_value")

			dpg.add_text("Angular Acceleration (deg/s^2)")
			with dpg.plot():
				dpg.add_plot_legend()
				dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
				dpg.set_axis_limits(dpg.last_item(), 0, 360)
				dpg.add_plot_axis(dpg.mvYAxis, label="Output Angular Acceleration (deg/s^2)", tag="angular_acceleration_y_axis")
				dpg.add_line_series(self._theta1, self._alpha2, label="Alpha 2", parent="angular_acceleration_y_axis")
				dpg.add_line_series(self._theta1, self._alpha3, label="Alpha 3", parent="angular_acceleration_y_axis")
				dpg.add_drag_line(label="Current Angle", tag="angular_acceleration_indicator", default_value=self.current_angle, callback=self.handle_drag_pos_indicator)

			dpg.add_tab_bar(label="Acceleration Analysis", tag="acceleration_tabs")
			dpg.add_tab(label="Joint Acceleration", tag="joint_tab", parent="acceleration_tabs")
			dpg.add_tab(label="CG Acceleration", tag="cg_tab", parent="acceleration_tabs")

			with dpg.group(parent="joint_tab"):
				aA = round(magnitude(self._accelerationA[self._angle_index]), 2)
				aBA = round(magnitude(self._accelerationBA[self._angle_index]), 2)
				aB = round(magnitude(self._accelerationB[self._angle_index]), 2)

				aA_list = [ magnitude(a) for a in self._accelerationA ]
				aBA_list = [ magnitude(a) for a in self._accelerationBA ]
				aB_list = [ magnitude(a) for a in self._accelerationB ]

				dpg.add_text(f"Magnitudal Velocity 1 (Input Speed): {aA} m/s^2", tag="mag_aA_value")
				dpg.add_text(f"Magnitudal Velocity 2: {aBA} m/s^2", tag="mag_aBA_value")
				dpg.add_text(f"Magnitudal Velocity 3: {aB} m/s^2", tag="mag_aB_value")

				dpg.add_text("Magnitudal Acceleration (m/s^2)")
				with dpg.plot():
					dpg.add_plot_legend()
					dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
					dpg.set_axis_limits(dpg.last_item(), 0, 360)
					dpg.add_plot_axis(dpg.mvYAxis, label="Output Magnitudal Acceleration (m/s^2)", tag="mag_acceleration_y_axis")
					dpg.add_line_series(self._theta1, aA_list, label="Acceleration 1", parent="mag_acceleration_y_axis")
					dpg.add_line_series(self._theta1, aBA_list, label="Acceleration 2", parent="mag_acceleration_y_axis")
					dpg.add_line_series(self._theta1, aB_list, label="Acceleration 3", parent="mag_acceleration_y_axis")
					dpg.add_drag_line(label="Current Angle", tag="mag_acceleration_indicator", default_value=self.current_angle, callback=self.handle_drag_pos_indicator)

				tang_aA = round(rotate(self._accelerationA[self._angle_index], self._theta1[self._angle_index])[1], 2)
				radial_aA = round(rotate(self._accelerationA[self._angle_index], self._theta1[self._angle_index])[0], 2)
				tang_aBA = round(rotate(self._accelerationBA[self._angle_index], self._theta2[self._angle_index])[1], 2)
				radial_aBA = round(rotate(self._accelerationBA[self._angle_index], self._theta2[self._angle_index])[0], 2)
				tang_aB = round(rotate(self._accelerationB[self._angle_index], self._theta3[self._angle_index])[1], 2)
				radial_aB = round(rotate(self._accelerationB[self._angle_index], self._theta3[self._angle_index])[0], 2)

				tang_aA_list = [ rotate(self._accelerationA[i], self._theta1[i])[1] for i in range(len(self._accelerationA)) ]
				radial_aA_list = [ rotate(self._accelerationA[i], self._theta1[i])[0] for i in range(len(self._accelerationA)) ]
				tang_aBA_list = [ rotate(self._accelerationBA[i], self._theta2[i])[1] for i in range(len(self._accelerationBA)) ]
				radial_aBA_list = [ rotate(self._accelerationBA[i], self._theta2[i])[0] for i in range(len(self._accelerationBA)) ]
				tang_aB_list = [ rotate(self._accelerationB[i], self._theta3[i])[1] for i in range(len(self._accelerationB)) ]
				radial_aB_list = [ rotate(self._accelerationB[i], self._theta3[i])[0] for i in range(len(self._accelerationB)) ]

				dpg.add_text(f"Tangential Acceleration 1: {tang_aA} m/s^2", tag="tang_aA_value")
				dpg.add_text(f"Radial Acceleration 1: {radial_aA} m/s^2", tag="radial_aA_value")
				dpg.add_text(f"Tangential Acceleration 2: {tang_aBA} m/s^2", tag="tang_aBA_value")
				dpg.add_text(f"Radial Acceleration 2: {radial_aBA} m/s^2", tag="radial_aBA_value")
				dpg.add_text(f"Tangential Acceleration 3: {tang_aB} m/s^2", tag="tang_aB_value")
				dpg.add_text(f"Radial Acceleration 3: {radial_aB} m/s^2", tag="radial_aB_value")

				dpg.add_text("Acceleration Components (m/s^2)")
				with dpg.plot():
					dpg.add_plot_legend()
					dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
					dpg.set_axis_limits(dpg.last_item(), 0, 360)
					dpg.add_plot_axis(dpg.mvYAxis, label="Output Acceleration Components (m/s^2)", tag="comp_acceleration_y_axis")
					dpg.add_line_series(self._theta1, tang_aA_list, label="Tangential 1", parent="comp_acceleration_y_axis")
					dpg.add_line_series(self._theta1, radial_aA_list, label="Radial 1", parent="comp_acceleration_y_axis")
					dpg.add_line_series(self._theta1, tang_aBA_list, label="Tangential 2", parent="comp_acceleration_y_axis")
					dpg.add_line_series(self._theta1, radial_aBA_list, label="Radial 2", parent="comp_acceleration_y_axis")
					dpg.add_line_series(self._theta1, tang_aB_list, label="Tangential 3", parent="comp_acceleration_y_axis")
					dpg.add_line_series(self._theta1, radial_aB_list, label="Radial 3", parent="comp_acceleration_y_axis")
					dpg.add_drag_line(label="Current Angle", tag="comp_acceleration_indicator", default_value=self.current_angle, callback=self.handle_drag_pos_indicator)

			with dpg.group(parent="cg_tab"):
				cg_aA = round(magnitude(self._cgAccelerationA[self._angle_index]), 2)
				cg_aBA = round(magnitude(self._cgAccelerationBA[self._angle_index]), 2)
				cg_aB = round(magnitude(self._cgAccelerationB[self._angle_index]), 2)

				cg_aA_list = [ magnitude(a) for a in self._cgAccelerationA ]
				cg_aBA_list = [ magnitude(a) for a in self._cgAccelerationBA ]
				cg_aB_list = [ magnitude(a) for a in self._cgAccelerationB ]

				dpg.add_text(f"CG Magnitudal Velocity 1 (Input Speed): {cg_aA} m/s^2", tag="mag_cg_aA_value")
				dpg.add_text(f"CG Magnitudal Velocity 2: {cg_aBA} m/s^2", tag="mag_cg_aBA_value")
				dpg.add_text(f"CG Magnitudal Velocity 3: {cg_aB} m/s^2", tag="mag_cg_aB_value")

				dpg.add_text("CG Magnitudal Acceleration (m/s^2)")
				with dpg.plot():
					dpg.add_plot_legend()
					dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
					dpg.set_axis_limits(dpg.last_item(), 0, 360)
					dpg.add_plot_axis(dpg.mvYAxis, label="Output CG Magnitudal Acceleration (m/s^2)", tag="mag_cg_acceleration_y_axis")
					dpg.add_line_series(self._theta1, cg_aA_list, label="CG Acceleration 1", parent="mag_cg_acceleration_y_axis")
					dpg.add_line_series(self._theta1, cg_aBA_list, label="CG Acceleration 2", parent="mag_cg_acceleration_y_axis")
					dpg.add_line_series(self._theta1, cg_aB_list, label="CG Acceleration 3", parent="mag_cg_acceleration_y_axis")
					dpg.add_drag_line(label="Current Angle", tag="mag_cg_acceleration_indicator", default_value=self.current_angle, callback=self.handle_drag_pos_indicator)

				cg_aA_x = round(self._cgAccelerationA[self._angle_index][0], 2)
				cg_aA_y = round(self._cgAccelerationA[self._angle_index][1], 2)
				cg_aBA_x = round(self._cgAccelerationBA[self._angle_index][0], 2)
				cg_aBA_y = round(self._cgAccelerationBA[self._angle_index][1], 2)
				cg_aB_x = round(self._cgAccelerationB[self._angle_index][0], 2)
				cg_aB_y = round(self._cgAccelerationB[self._angle_index][1], 2)

				cg_aA_x_list = [ self._cgAccelerationA[i][0] for i in range(len(self._cgAccelerationA)) ]
				cg_aA_y_list = [ self._cgAccelerationA[i][1] for i in range(len(self._cgAccelerationA)) ]
				cg_aBA_x_list = [ self._cgAccelerationBA[i][0] for i in range(len(self._cgAccelerationBA)) ]
				cg_aBA_y_list = [ self._cgAccelerationBA[i][1] for i in range(len(self._cgAccelerationBA)) ]
				cg_aB_x_list = [ self._cgAccelerationB[i][0] for i in range(len(self._cgAccelerationB)) ]
				cg_aB_y_list = [ self._cgAccelerationB[i][1] for i in range(len(self._cgAccelerationB)) ]

				dpg.add_text(f"CG Acceleration 1 in X: {cg_aA_x} m/s^2", tag="cg_aA_x_value")
				dpg.add_text(f"CG Acceleration 1 in Y: {cg_aA_y} m/s^2", tag="cg_aA_y_value")
				dpg.add_text(f"CG Acceleration 2 in X: {cg_aBA_x} m/s^2", tag="cg_aBA_x_value")
				dpg.add_text(f"CG Acceleration 2 in Y: {cg_aBA_y} m/s^2", tag="cg_aBA_y_value")
				dpg.add_text(f"CG Acceleration 3 in X: {cg_aB_x} m/s^2", tag="cg_aB_x_value")
				dpg.add_text(f"CG Acceleration 3 in Y: {cg_aB_y} m/s^2", tag="cg_aB_y_value")

				dpg.add_text("CG Acceleration Components (m/s^2)")
				with dpg.plot():
					dpg.add_plot_legend()
					dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
					dpg.set_axis_limits(dpg.last_item(), 0, 360)
					dpg.add_plot_axis(dpg.mvYAxis, label="Output CG Acceleration Components (m/s^2)", tag="comp_cg_acceleration_y_axis")
					dpg.add_line_series(self._theta1, cg_aA_x_list, label="CG Acceleration 1 in X", parent="comp_cg_acceleration_y_axis")
					dpg.add_line_series(self._theta1, cg_aA_y_list, label="CG Acceleration 1 in Y", parent="comp_cg_acceleration_y_axis")
					dpg.add_line_series(self._theta1, cg_aBA_x_list, label="CG Acceleration 2 in X", parent="comp_cg_acceleration_y_axis")
					dpg.add_line_series(self._theta1, cg_aBA_y_list, label="CG Acceleration 2 in Y", parent="comp_cg_acceleration_y_axis")
					dpg.add_line_series(self._theta1, cg_aB_x_list, label="CG Acceleration 3 in X", parent="comp_cg_acceleration_y_axis")
					dpg.add_line_series(self._theta1, cg_aB_y_list, label="CG Acceleration 3 in Y", parent="comp_cg_acceleration_y_axis")
					dpg.add_drag_line(label="Current Angle", tag="comp_cg_acceleration_indicator", default_value=self.current_angle, callback=self.handle_drag_pos_indicator)

	# Create performance plots
	def create_performance_plot(self):
		dpg.add_collapsing_header(label="Performance Analysis", tag="performance_analysis", parent="inspector")
		with dpg.group(parent="performance_analysis"):
			transmission_angle = round(self._transmission_angle[self._angle_index], 2)
			velocity_ratio = round(self._velocity_ratio[self._angle_index], 2)
			mechanical_advantage = round(self._mechanical_advantage[self._angle_index], 2)

			dpg.add_text(f"Transmission Angle: {transmission_angle} deg", tag="transmission_angle_value")
			dpg.add_text(f"Range of Transmission Angle: {round(min(self._transmission_angle), 2)} to {round(max(self._transmission_angle), 2)} deg")

			with dpg.plot():
				dpg.add_plot_legend()
				dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
				dpg.set_axis_limits(dpg.last_item(), 0, 360)
				dpg.add_plot_axis(dpg.mvYAxis, label="Transmission Angle (degree)", tag="transmission_y_axis")
				dpg.add_line_series(self._theta1, self._transmission_angle, label="Transmission Angle", parent="transmission_y_axis")
				dpg.add_drag_line(label="Current Angle", tag="transmission_indicator", default_value=self.current_angle, callback=self.handle_drag_pos_indicator)

			dpg.add_text(f"Velocity Ratio: {velocity_ratio}", tag="velocity_ratio_value")
			dpg.add_text(f"Range of Velocity Ratio: {round(min(self._velocity_ratio), 2)} to {round(max(self._velocity_ratio), 2)}")

			with dpg.plot():
				dpg.add_plot_legend()
				dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
				dpg.set_axis_limits(dpg.last_item(), 0, 360)
				dpg.add_plot_axis(dpg.mvYAxis, label="Output Ratio", tag="velocity_ratio_y_axis")
				dpg.add_line_series(self._theta1, self._velocity_ratio, label="Velocity Ratio", parent="velocity_ratio_y_axis")
				dpg.add_drag_line(label="Current Angle", tag="velocity_ratio_indicator", default_value=self.current_angle, callback=self.handle_drag_pos_indicator)

			dpg.add_text(f"Mechanical Advantage: {mechanical_advantage}", tag="mechanical_advantage_value")
			dpg.add_text(f"Range of Mechanical Advantage: {round(min(self._mechanical_advantage), 2)} to {round(max(self._mechanical_advantage), 2)}")

			with dpg.plot():
				dpg.add_plot_legend()
				dpg.add_plot_axis(dpg.mvXAxis, label="Input Angle (degree)")
				dpg.set_axis_limits(dpg.last_item(), 0, 360)
				dpg.add_plot_axis(dpg.mvYAxis, label="Output Ratio", tag="mechanical_advantange_y_axis")
				dpg.add_line_series(self._theta1, self._mechanical_advantage, label="Mechanical Advantage", parent="mechanical_advantange_y_axis")
				dpg.add_drag_line(label="Current Angle", tag="mechanical_advantage_indicator", default_value=self.current_angle, callback=self.handle_drag_pos_indicator)

	# Analysis Settings
	_velocity_scale = VELOCITY_SCALE
	_offset = (WINDOW_WIDTH * 0.25, WINDOW_HEIGHT * 0.55)
	_angle_index = 0
	_all_min_x = 0
	_all_max_x = 0
	_all_min_y = 0
	_all_max_y = 0

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
	_cgAccelerationA = (0, 0)
	_cgAccelerationB = (0, 0)
	_cgAccelerationBA = (0, 0)
	_transmission_angle = []
	_velocity_ratio = []
	_mechanical_advantage = []

App().run()
