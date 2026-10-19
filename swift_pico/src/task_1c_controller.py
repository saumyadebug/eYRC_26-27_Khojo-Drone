#!/usr/bin/env python3

'''
Task 1C - Position (X / pitch, Y / roll) PID controller for the Swift Pico drone.

This node holds the drone at a target position inside the arena target box using
PID loops on the horizontal axes, while keeping the Task 1B altitude (Z /
throttle) behaviour EXACTLY as tuned.

		PUBLICATIONS					SUBSCRIPTIONS
		/drone_command					/whycode_node/markers
		/pos_error						/pitch_pid
		/controller_heartbeat			/roll_pid
											/throttle_pid

COORDINATE / SIGN CONVENTIONS
-----------------------------
Altitude feedback comes from the WhyCode marker on the drone.  The WhyCode
camera looks straight down from z = 15, so ``markers[0].position.position.z`` is
the DEPTH below the camera, not the height above the floor:

	whycode z = 15 (camera height) - world height

so a LARGER whycode z means the drone is LOWER.  The throttle axis therefore
subtracts its controller output from the neutral value (see ``self.cmd``).

The horizontal axes use the WhyCode camera x/y directly, which are already in
the world frame (image right = +x, image down = -y).  The roll/pitch control
directions were established empirically from the simulation and are applied via
``self.roll_sign`` / ``self.pitch_sign``.

THROTTLE (TASK 1B) GAINS ARE PRESERVED
---------------------------------------
The altitude loop is the same one tuned for Task 1B (Kp=35, Ki=2, Kd=240 with a
0.035-altitude low-pass filter that removes the WhyCode velocity noise).  It is
NOT re-tuned here; only the horizontal gains are new.  Throttle gains can also
be overridden live over /throttle_pid exactly as in Task 1B.
'''

from swift_msgs.msg import SwiftMsgs
from whycode_interfaces.msg import MarkerArray
from controller_msg.msg import PIDTune
from error_msg.msg import Error
from std_msgs.msg import Header

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class Task1CController(Node):
	def __init__(self):
		super().__init__('task_1c_controller')

		# ------------------------------------------------------------------
		# Measured and desired state.  Index 2 is WhyCode DEPTH.
		# The horizontal target is the arena centre box (F6 is the grid
		# centre per arena.xml: world_x=(col-5)*1.203414, world_y=(6-row)*1.203414,
		# with F6 == world origin).
		# ------------------------------------------------------------------
		self.current_state = [0.0, 0.0, 15.0]
		self.desired_state = [0.0, 0.0, 13.0]

		self.got_pose = False

		# RC limits: 1000..2000, neutral/hover 1500.
		self.min_value = 1000
		self.max_value = 2000
		self.hover_value = 1500

		# ------------------------------------------------------------------
		# Gains.
		#   index 0 = roll  -> rc_roll
		#   index 1 = pitch -> rc_pitch
		#   index 2 = throttle -> rc_throttle
		#
		# Throttle (Task 1B) is preserved: 35 / 2 / 240.
		# Pitch and Roll start conservatively and were tuned empirically.
		# ------------------------------------------------------------------
		self.Kp = [12.0, 12.0, 35.0]
		self.Ki = [0.5, 0.5, 2.0]
		self.Kd = [8.0, 8.0, 240.0]

		self.tuner_gain_scaling = 1.0

		# Control direction for each horizontal axis.  rc_roll / rc_pitch are
		# ATTITUDE commands on this vehicle (see linear_map in
		# pico_controller_LQR.py: rc = 1500 + (1000/pi) * tilt, clipped to
		# +/- pi/12), so horizontal position control is a CASCADE:
		#
		#   position error -> desired tilt (outer PID) -> rc_pitch / rc_roll
		#
		# The inner attitude loop is handled by the vehicle.
		#
		# AXIS MAP: rc_pitch drives x, rc_roll drives y (the vehicle tilts to
		# translate; verified in the simulation - each axis holds on its own).
		# With error = desired - measured, a negative error must move the
		# drone towards -axis, which is achieved by driving the corresponding
		# RC channel BELOW neutral.  Both signs are therefore +1.
		self.pitch_sign = 1.0     # applies to error[0] (x) -> rc_pitch
		self.roll_sign = 1.0      # applies to error[1] (y) -> rc_roll

		# Outer-loop gains produce a desired tilt angle in radians.
		# The altitude gains (index 2) are kept in RC counts, as in Task 1B.
		self.Kp_outer = [0.12, 0.12]
		self.Ki_outer = [0.02, 0.02]
		self.Kd_outer = [0.10, 0.10]

		# Desired tilt is clamped to the vehicle's usable range.
		self.max_tilt = math.pi / 12.0

		# rc = 1500 + (1000/pi) * tilt   (matches pico_controller_LQR.linear_map)
		self.rc_per_radian = 1000.0 / math.pi

		# Altitude gains preserved verbatim from Task 1B.
		self.Kp = [0.0, 0.0, 35.0]
		self.Ki = [0.0, 0.0, 2.0]
		self.Kd = [0.0, 0.0, 240.0]

		# ------------------------------------------------------------------
		# PID bookkeeping, per axis.
		# ------------------------------------------------------------------
		self.integral = [0.0, 0.0, 0.0]
		self.prev_error = [0.0, 0.0, 0.0]
		self.filtered_derivative = [0.0, 0.0, 0.0]
		self.last_time = None

		self.integral_limit = 6.0
		self.derivative_filter = 0.3

		# WhyCode horizontal measurements are also noisy; a low-pass on the
		# horizontal feedback keeps the derivative term usable (the same
		# problem, and the same fix, as the altitude axis in Task 1B).
		self.horizontal_filter_alpha = 0.10
		self.filtered_xy = [None, None]

		# Task 1B altitude low-pass (preserved exactly).
		self.altitude_filter_alpha = 0.035
		self.filtered_altitude = None

		# Marker-loss failsafe (preserved from Task 1B).
		self.marker_timeout = 0.5
		self.last_marker_time = None
		self.descent_throttle = 1200

		# Usable WhyCode depth envelope: the camera sits at 15 m.
		self.min_depth = 0.5
		self.max_depth = 14.5

		# Target box half-extents (metres) used to declare "inside the box".
		self.box_tolerance = 0.4
		self.hover_hold_seconds = 10.0
		self.hover_accum = 0.0
		self.hover_reported = False

		self.sample_time = 0.033

		# ------------------------------------------------------------------
		# Message containers.
		# ------------------------------------------------------------------
		self.cmd = SwiftMsgs()
		self.cmd.rc_roll = self.hover_value
		self.cmd.rc_pitch = self.hover_value
		self.cmd.rc_yaw = self.hover_value
		self.cmd.rc_throttle = self.hover_value

		self.pos_error = Error()
		self.pos_error.roll_error = 0.0
		self.pos_error.pitch_error = 0.0
		self.pos_error.throttle_error = 0.0
		self.pos_error.yaw_error = 0.0

		# ------------------------------------------------------------------
		# Publishers / subscribers.
		# ------------------------------------------------------------------
		self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
		self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)
		self.heartbeat_pub = self.create_publisher(Header, '/controller_heartbeat', 10)

		self.create_subscription(MarkerArray, '/whycode_node/markers', self.whycode_callback, 1)
		self.create_subscription(PIDTune, '/pitch_pid', self.pitch_set_pid, 1)
		self.create_subscription(PIDTune, '/roll_pid', self.roll_set_pid, 1)
		self.create_subscription(PIDTune, '/throttle_pid', self.throttle_set_pid, 1)

		self.arm()
		self.publish_heartbeat()
		self.create_timer(1.0, self.publish_heartbeat)
		self.timer = self.create_timer(self.sample_time, self.pid)

		self.get_logger().info(
			'Task 1C position PID active: roll Kp=%.2f Ki=%.2f Kd=%.2f | '
			'pitch Kp=%.2f Ki=%.2f Kd=%.2f | throttle Kp=%.2f Ki=%.2f Kd=%.2f | '
			'target=(%.2f, %.2f, %.2f)' % (
				self.Kp[0], self.Ki[0], self.Kd[0],
				self.Kp[1], self.Ki[1], self.Kd[1],
				self.Kp[2], self.Ki[2], self.Kd[2],
				self.desired_state[0], self.desired_state[1], self.desired_state[2]))

	def publish_heartbeat(self):
		hb = Header()
		hb.stamp = self.get_clock().now().to_msg()
		hb.frame_id = 'task_1c_controller_active'
		self.heartbeat_pub.publish(hb)

	def disarm(self):
		self.cmd.rc_roll = self.min_value
		self.cmd.rc_pitch = self.min_value
		self.cmd.rc_yaw = self.min_value
		self.cmd.rc_throttle = self.min_value
		self.cmd.rc_aux4 = self.min_value
		self.command_pub.publish(self.cmd)

	def arm(self):
		self.disarm()
		self.reset_pid()
		self.cmd.rc_roll = self.hover_value
		self.cmd.rc_pitch = self.hover_value
		self.cmd.rc_yaw = self.hover_value
		self.cmd.rc_throttle = self.hover_value
		self.cmd.rc_aux4 = 2000
		self.command_pub.publish(self.cmd)

	def reset_pid(self):
		self.integral = [0.0, 0.0, 0.0]
		self.prev_error = [0.0, 0.0, 0.0]
		self.filtered_derivative = [0.0, 0.0, 0.0]
		self.filtered_xy = [None, None]
		self.filtered_altitude = None
		self.last_time = None

	# ----------------------------------------------------------------------
	# WhyCode callback - pose feedback source.
	# ----------------------------------------------------------------------
	def whycode_callback(self, msg):
		if not msg.markers:
			return

		marker = msg.markers[0]
		self.current_state[0] = marker.position.position.x
		self.current_state[1] = marker.position.position.y
		self.current_state[2] = marker.position.position.z
		self.got_pose = True
		self.last_marker_time = self.get_clock().now().nanoseconds * 1e-9

	# ----------------------------------------------------------------------
	# Live gain updates from the PID tuner.
	# The tuner publishes (gui_integer * gui_multiplier); that product is the
	# actual gain and is applied directly, with no further scaling.
	# ----------------------------------------------------------------------
	def roll_set_pid(self, msg):
		self.Kp[0] = msg.kp * self.tuner_gain_scaling
		self.Ki[0] = msg.ki * self.tuner_gain_scaling
		self.Kd[0] = msg.kd * self.tuner_gain_scaling
		self.get_logger().info('Roll gains updated: Kp=%.3f Ki=%.3f Kd=%.3f' % (
			self.Kp[0], self.Ki[0], self.Kd[0]))

	def pitch_set_pid(self, msg):
		self.Kp[1] = msg.kp * self.tuner_gain_scaling
		self.Ki[1] = msg.ki * self.tuner_gain_scaling
		self.Kd[1] = msg.kd * self.tuner_gain_scaling
		self.get_logger().info('Pitch gains updated: Kp=%.3f Ki=%.3f Kd=%.3f' % (
			self.Kp[1], self.Ki[1], self.Kd[1]))

	def throttle_set_pid(self, msg):
		# Same semantics as Task 1B; the Task 1B gains are preserved when the
		# tuner is not publishing.
		self.Kp[2] = msg.kp * self.tuner_gain_scaling
		self.Ki[2] = msg.ki * self.tuner_gain_scaling
		self.Kd[2] = msg.kd * self.tuner_gain_scaling
		self.get_logger().info('Throttle gains updated: Kp=%.3f Ki=%.3f Kd=%.3f' % (
			self.Kp[2], self.Ki[2], self.Kd[2]))

	# ----------------------------------------------------------------------
	# PID control loop (runs at self.sample_time).
	# ----------------------------------------------------------------------
	def pid(self):
		now = self.get_clock().now().nanoseconds * 1e-9

		# Marker-loss failsafe (preserved from Task 1B): no usable feedback ->
		# stop integrating and descend so the drone re-enters the camera view.
		if self.got_pose and self.last_marker_time is not None:
			if (now - self.last_marker_time) > self.marker_timeout:
				self.integral = [0.0, 0.0, 0.0]
				self.filtered_derivative = [0.0, 0.0, 0.0]
				self.cmd.rc_roll = self.hover_value
				self.cmd.rc_pitch = self.hover_value
				self.cmd.rc_throttle = self.descent_throttle
				self.command_pub.publish(self.cmd)
				self.pos_error_pub.publish(self.pos_error)
				return

		if not self.got_pose:
			self.cmd.rc_roll = self.hover_value
			self.cmd.rc_pitch = self.hover_value
			self.cmd.rc_throttle = self.hover_value
			self.command_pub.publish(self.cmd)
			self.pos_error_pub.publish(self.pos_error)
			return

		if self.last_time is None:
			self.last_time = now
			self.command_pub.publish(self.cmd)
			self.pos_error_pub.publish(self.pos_error)
			return

		dt = now - self.last_time
		self.last_time = now
		if dt <= 0.0 or dt > 0.5:
			dt = self.sample_time

		# ---- smooth the measurements -----------------------------------
		a = self.horizontal_filter_alpha
		for i in (0, 1):
			if self.filtered_xy[i] is None:
				self.filtered_xy[i] = self.current_state[i]
			else:
				self.filtered_xy[i] += a * (self.current_state[i] - self.filtered_xy[i])

		measured_alt = min(max(self.current_state[2], self.min_depth), self.max_depth)
		if self.filtered_altitude is None:
			self.filtered_altitude = measured_alt
		else:
			self.filtered_altitude += self.altitude_filter_alpha * (measured_alt - self.filtered_altitude)

		# ---- errors ----------------------------------------------------
		# Horizontal: positive error => target is ahead in +axis.
		# Altitude:   larger WhyCode z => drone lower (see module docstring).
		error = [0.0, 0.0, 0.0]
		error[0] = self.desired_state[0] - self.filtered_xy[0]
		error[1] = self.desired_state[1] - self.filtered_xy[1]
		error[2] = self.desired_state[2] - self.filtered_altitude

		# ---- PID terms per axis ----------------------------------------
		# Axes 0 (roll) and 1 (pitch) are the OUTER position loop and use
		# Kp_outer/Ki_outer/Kd_outer to produce a desired tilt.
		# Axis 2 (throttle) is the Task 1B altitude loop in RC counts.
		terms = []
		for i in range(3):
			self.integral[i] += error[i] * dt
			self.integral[i] = max(-self.integral_limit, min(self.integral_limit, self.integral[i]))

			raw_d = (error[i] - self.prev_error[i]) / dt
			self.filtered_derivative[i] += self.derivative_filter * (raw_d - self.filtered_derivative[i])

			if i < 2:
				kp, ki, kd = self.Kp_outer[i], self.Ki_outer[i], self.Kd_outer[i]
			else:
				kp, ki, kd = self.Kp[i], self.Ki[i], self.Kd[i]

			terms.append(kp * error[i] + ki * self.integral[i] + kd * self.filtered_derivative[i])

		# ---- commands --------------------------------------------------
		# Horizontal: outer PID produces a desired TILT (rad), converted to RC
		# with the same linear_map the LQR controller uses.
		# Altitude: subtract from neutral, because larger WhyCode z = lower.
		max_dev = min(self.hover_value - self.min_value, self.max_value - self.hover_value)

		# Axis map (measured): error[0] is x and is driven by rc_pitch;
		# error[1] is y and is driven by rc_roll.
		tilt_pitch = self.pitch_sign * terms[0]
		tilt_roll = self.roll_sign * terms[1]
		tilt_pitch = max(-self.max_tilt, min(self.max_tilt, tilt_pitch))
		tilt_roll = max(-self.max_tilt, min(self.max_tilt, tilt_roll))

		pitch_cmd = int(round(max(self.min_value, min(self.max_value,
			self.hover_value + self.rc_per_radian * tilt_pitch))))
		roll_cmd = int(round(max(self.min_value, min(self.max_value,
			self.hover_value + self.rc_per_radian * tilt_roll))))

		throttle_out = -terms[2]
		throttle_out = max(-max_dev, min(max_dev, throttle_out))
		throttle_cmd = int(round(max(self.min_value, min(self.max_value, self.hover_value + throttle_out))))

		# Conditional integration on the altitude axis to prevent windup.
		if ((throttle_cmd >= self.max_value and throttle_out > 0)
				or (throttle_cmd <= self.min_value and throttle_out < 0)):
			self.integral[2] -= error[2] * dt
			self.integral[2] = max(-self.integral_limit, min(self.integral_limit, self.integral[2]))

		self.cmd.rc_roll = roll_cmd
		self.cmd.rc_pitch = pitch_cmd
		self.cmd.rc_throttle = throttle_cmd
		self.prev_error = list(error)

		# ---- publish ---------------------------------------------------
		# Error fields: roll_error follows x, pitch_error follows y, matching
		# the index convention used throughout the project.
		self.pos_error.roll_error = error[0]
		self.pos_error.pitch_error = error[1]
		self.pos_error.throttle_error = error[2]

		# ---- hover-in-box reporting -------------------------------------
		inside = (abs(error[0]) <= self.box_tolerance
				  and abs(error[1]) <= self.box_tolerance
				  and abs(error[2]) <= self.box_tolerance)
		if inside:
			self.hover_accum += dt
			if self.hover_accum >= self.hover_hold_seconds and not self.hover_reported:
				self.hover_reported = True
				self.get_logger().info(
					'HOVER: inside target box (+/-%.2f m) continuously for %.0f s' % (
						self.box_tolerance, self.hover_accum))
		else:
			self.hover_accum = 0.0
			self.hover_reported = False

		self.command_pub.publish(self.cmd)
		self.pos_error_pub.publish(self.pos_error)


def main(args=None):
	import argparse
	parser = argparse.ArgumentParser(description='Task 1C position PID controller')
	parser.add_argument('--kp', type=float, default=None, help='roll Kp')
	parser.add_argument('--ki', type=float, default=None, help='roll Ki')
	parser.add_argument('--kd', type=float, default=None, help='roll Kd')
	parser.add_argument('--kp-pitch', type=float, default=None)
	parser.add_argument('--ki-pitch', type=float, default=None)
	parser.add_argument('--kd-pitch', type=float, default=None)
	parsed, ros_args = parser.parse_known_args(args)

	rclpy.init(args=ros_args)
	controller = Task1CController()
	if parsed.kp is not None:
		controller.Kp[0] = parsed.kp
	if parsed.ki is not None:
		controller.Ki[0] = parsed.ki
	if parsed.kd is not None:
		controller.Kd[0] = parsed.kd
	if parsed.kp_pitch is not None:
		controller.Kp[1] = parsed.kp_pitch
	if parsed.ki_pitch is not None:
		controller.Ki[1] = parsed.ki_pitch
	if parsed.kd_pitch is not None:
		controller.Kd[1] = parsed.kd_pitch
	controller.reset_pid()
	controller.get_logger().info(
		'Final gains: roll Kp=%.2f Ki=%.2f Kd=%.2f | pitch Kp=%.2f Ki=%.2f Kd=%.2f' % (
			controller.Kp[0], controller.Ki[0], controller.Kd[0],
			controller.Kp[1], controller.Ki[1], controller.Kd[1]))

	try:
		rclpy.spin(controller)
	except KeyboardInterrupt:
		controller.get_logger().info('KeyboardInterrupt, shutting down.\n')
	except ExternalShutdownException:
		pass
	finally:
		try:
			controller.destroy_node()
		except Exception:
			pass
		if rclpy.ok():
			rclpy.shutdown()


if __name__ == '__main__':
	main()