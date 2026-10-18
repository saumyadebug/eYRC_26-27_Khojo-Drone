#!/usr/bin/env python3

'''
Task 1B - Altitude (Z / throttle) PID controller for the Swift Pico drone.

This node holds the drone at an altitude setpoint using a PID loop on the
throttle axis only.  X/pitch and Y/roll are intentionally not controlled.

		PUBLICATIONS					SUBSCRIPTIONS
		/drone_command					/whycode_node/markers
		/pos_error						/throttle_pid
		/controller_heartbeat

COORDINATE / SIGN CONVENTION
-----------------------------
Altitude feedback comes from the WhyCode marker carried by the drone.  The
WhyCode camera looks straight down from z = 15, so the reported
``markers[0].position.position.z`` is the *depth* below the camera, not the
height above the floor:

	whycode z  = 15 (camera height) - world height

A drone resting on the floor therefore reads roughly 15, and the Task 1B
setpoint of 13.0 corresponds to a hover 2 m above the floor.  Because a LARGER
whycode z means the drone is LOWER, the controller output is SUBTRACTED from
the neutral throttle:

	error = setpoint - measured      # positive => drone is too low, climb
	out   = Kp*e + Ki*I + Kd*de      # climb => increase throttle
	rc_throttle = 1500 + out        # ...equivalently 1500 - Kp*e for large e

The generic form used below is ``rc_throttle = 1500 + out`` with
``out = -(Kp*e + Ki*I + Kd*de)``, which makes the direction explicit and keeps
the usual PID sign convention readable.
'''

from swift_msgs.msg import SwiftMsgs
from whycode_interfaces.msg import MarkerArray
from controller_msg.msg import PIDTune
from error_msg.msg import Error
from std_msgs.msg import Header

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class Task1BController(Node):
	def __init__(self, kp=None, ki=None, kd=None):
		super().__init__('task_1b_controller')

		# ------------------------------------------------------------------
		# Measured and desired state.  Only index 2 (Z / altitude) is used by
		# this controller; x and y are recorded from the marker for context.
		# ------------------------------------------------------------------
		self.current_state = [0.0, 0.0, 15.0]
		self.desired_state = [0.0, 0.0, 13.0]   # altitude setpoint = 13.0

		# Set once the first valid WhyCode marker has been received.  The PID
		# loop stays idle until then so it never integrates a fake altitude.
		self.got_altitude = False

		# Marker-loss failsafe.  If the WhyCode marker leaves the field of
		# view the reported altitude becomes stale; integrating against a stale
		# value while commanding climb is a positive-feedback runaway (measured:
		# the drone climbed past the camera at z=15 m and never recovered).
		# When the marker is lost we therefore stop integrating and command a
		# gentle descent so the drone falls back into the camera's view.
		self.marker_timeout = 0.5        # seconds without a marker
		self.last_marker_time = None
		self.marker_lost = False
		self.descent_throttle = 1200

		# Usable WhyCode depth envelope.  The camera sits at 15 m, so the
		# drone can never be nearer than ~0.5 m (depth 14.5) nor above the
		# camera (depth 0).  Readings outside this band are treated as invalid.
		self.min_depth = 0.5
		self.max_depth = 14.5

		# ------------------------------------------------------------------
		# RC command limits.  Neutral/hover is 1500; the Swift interface maps
		#   rc_throttle = 1000 -> 0 thrust
		#   rc_throttle = 1500 -> hover weight
		#   rc_throttle = 2000 -> 2x hover weight
		# ------------------------------------------------------------------
		self.min_value = 1000
		self.max_value = 2000
		self.hover_value = 1500

		# ------------------------------------------------------------------
		# PID gains for the throttle axis.
		#
		# NOTE on the PID-tuner scaling: /throttle_pid carries the value the
		# tuner computed as (gui_integer * gui_multiplier).  The template
		# controller then applied a SECOND factor on top of that.  Applying
		# both multipliers in this node would attenuate the gains by two extra
		# orders of magnitude (Kp_effective = 0.03 * 0.03 = 9e-4 per GUI
		# unit, i.e. a GUI value of 5000 would yield Kp = 4.5), so the tuner
		# value is used as-is here.
		#
		# The defaults below are sized from the command range: Kp * 1 m of
		# altitude error commands ~120 throttle counts, which is roughly a
		# quarter of the usable half-range, so a 2 m initial error saturates
		# gracefully instead of slamming to the limit.
		# ------------------------------------------------------------------
		self.Kp = [0.0, 0.0, 120.0]
		self.Ki = [0.0, 0.0, 12.0]
		self.Kd = [0.0, 0.0, 8.0]

		# Gains are overwritten live by /throttle_pid when the tuner runs.
		self.tuner_gain_scaling = 1.0

		# Optional CLI overrides.  The task delivers gains through the PID
		# tuner GUI (/throttle_pid); these exist so the same values can be
		# reproduced head-lessly for the official clean recording run and for
		# automated tuning.  They are applied after __init__() builds the
		# default gains, so they take precedence unless the tuner publishes.
		self._override_gains = None

		# ------------------------------------------------------------------
		# PID bookkeeping.
		# ------------------------------------------------------------------
		self.integral = [0.0, 0.0, 0.0]      # accumulated error (anti-windup clamped)
		self.prev_error = [0.0, 0.0, 0.0]
		self.last_time = None

		# Integral term is clamped so windup cannot build unbounded.  The limit
		# is deliberately small: the integral's only job here is to trim out the
		# small constant hover offset, and large authority makes the climb
		# overshoot badly (measured: peak 4.3 m with a 40-unit clamp).
		self.integral_limit = 6.0

		# Derivative is low-pass filtered to suppress marker jitter, which is
		# the dominant noise source on the derivative term.  Measured: with Kd
		# around 70 an unfiltered derivative swings the throttle by hundreds of
		# counts on marker noise alone, which shows up as a limit cycle.
		self.derivative_filter = 0.3

		# Altitude low-pass filter.  Measured on this platform the raw WhyCode
		# depth jumps ~0.027 m between samples at ~133 Hz, which implies a
		# velocity noise of std ~2.9 m/s with peaks over 40 m/s.  Feeding that
		# straight into the derivative term produced full-scale (1000-2000)
		# throttle swings and a bang-bang limit cycle, so the measured altitude
		# is smoothed before the error and derivative are formed.
		self.altitude_filter_alpha = 0.035
		self.filtered_altitude = None
		self.filtered_derivative = [0.0, 0.0, 0.0]

		self.sample_time = 0.033           # ~30 Hz control loop

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
		self.create_subscription(PIDTune, '/throttle_pid', self.altitude_set_pid, 1)

		# ------------------------------------------------------------------
		# Arm and start the control loop.
		# ------------------------------------------------------------------
		self.arm()
		self.publish_heartbeat()
		self.create_timer(1.0, self.publish_heartbeat)

		self.timer = self.create_timer(self.sample_time, self.pid)

		self.get_logger().info(
			'Task 1B altitude PID active: Kp=%.2f Ki=%.2f Kd=%.2f, setpoint z=%.2f' % (
				self.Kp[2], self.Ki[2], self.Kd[2], self.desired_state[2]))

	def apply_gain_overrides(self, kp=None, ki=None, kd=None):
		"""Apply CLI-supplied throttle gains (used for tuning and for a
		headless reproduction of the values saved in the PID tuner)."""
		if kp is not None:
			self.Kp[2] = kp
		if ki is not None:
			self.Ki[2] = ki
		if kd is not None:
			self.Kd[2] = kd
		self.reset_pid()
		self.get_logger().info(
			'Throttle gains set to Kp=%.4f Ki=%.4f Kd=%.4f' % (
				self.Kp[2], self.Ki[2], self.Kd[2]))

	def publish_heartbeat(self):
		hb = Header()
		hb.stamp = self.get_clock().now().to_msg()
		hb.frame_id = 'task_1b_controller_active'
		self.heartbeat_pub.publish(hb)

	def disarm(self):
		self.cmd.rc_roll = self.min_value
		self.cmd.rc_yaw = self.min_value
		self.cmd.rc_pitch = self.min_value
		self.cmd.rc_throttle = self.min_value
		self.cmd.rc_aux4 = self.min_value
		self.command_pub.publish(self.cmd)

	def arm(self):
		# Publish a disarmed command first so the sim starts from a safe state.
		self.disarm()
		self.reset_pid()
		self.cmd.rc_roll = self.hover_value
		self.cmd.rc_yaw = self.hover_value
		self.cmd.rc_pitch = self.hover_value
		self.cmd.rc_throttle = self.hover_value
		self.cmd.rc_aux4 = 2000          # aux4 high = armed
		self.command_pub.publish(self.cmd)

	def reset_pid(self):
		"""Clear integral/derivative history, e.g. on (re)arming."""
		self.integral = [0.0, 0.0, 0.0]
		self.prev_error = [0.0, 0.0, 0.0]
		self.filtered_derivative = [0.0, 0.0, 0.0]
		self.last_time = None

	# ----------------------------------------------------------------------
	# WhyCode callback - altitude feedback source.
	# ----------------------------------------------------------------------
	def whycode_callback(self, msg):
		if not msg.markers:
			return

		marker = msg.markers[0]
		self.current_state[0] = marker.position.position.x
		self.current_state[1] = marker.position.position.y
		# Z is the depth below the overhead WhyCode camera.
		self.current_state[2] = marker.position.position.z
		self.got_altitude = True
		self.last_marker_time = self.get_clock().now().nanoseconds * 1e-9
		self.marker_lost = False

	# ----------------------------------------------------------------------
	# /throttle_pid callback - live gain update from the PID tuner.
	# The tuner publishes (gui_integer * gui_multiplier); that product is the
	# actual gain and is applied directly, with no further scaling.
	# ----------------------------------------------------------------------
	def altitude_set_pid(self, alt):
		self.Kp[2] = alt.kp * self.tuner_gain_scaling
		self.Ki[2] = alt.ki * self.tuner_gain_scaling
		self.Kd[2] = alt.kd * self.tuner_gain_scaling
		self.get_logger().info(
			'Throttle gains updated: Kp=%.3f Ki=%.3f Kd=%.3f' % (
				self.Kp[2], self.Ki[2], self.Kd[2]))

	# ----------------------------------------------------------------------
	# PID control loop (runs at self.sample_time).
	# ----------------------------------------------------------------------
	def pid(self):
		now = self.get_clock().now().nanoseconds * 1e-9

		# ---- marker-loss failsafe ----------------------------------------
		# No usable feedback -> do not integrate, command a descent so the
		# drone re-enters the WhyCode field of view.
		if self.got_altitude and self.last_marker_time is not None:
			if (now - self.last_marker_time) > self.marker_timeout:
				self.marker_lost = True
				self.integral[2] = 0.0
				self.filtered_derivative[2] = 0.0
				self.cmd.rc_throttle = self.descent_throttle
				self.command_pub.publish(self.cmd)
				self.pos_error_pub.publish(self.pos_error)
				return

		if not self.got_altitude:
			# No valid altitude feedback yet: hold neutral throttle and do not
			# integrate a fabricated error.
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

		# Guard against a zero/negative/huge dt (first tick, or a stalled clock).
		if dt <= 0.0:
			dt = self.sample_time
		elif dt > 0.5:
			dt = self.sample_time

		# ---- error -------------------------------------------------------
		# positive error => measured depth smaller than setpoint => drone too
		# low => climb (increase throttle)
		#
		# The measured depth is clamped to the usable camera envelope: the
		# drone can never legitimately be at or above the camera plane
		# (depth <= 0), and chasing such a reading is what drove the runaway.
		measured = min(max(self.current_state[2], self.min_depth), self.max_depth)

		# Smooth the measurement before forming error/derivative (see
		# altitude_filter_alpha above).  Without this the derivative term is
		# dominated by marker noise.
		if self.filtered_altitude is None:
			self.filtered_altitude = measured
		else:
			self.filtered_altitude += self.altitude_filter_alpha * (measured - self.filtered_altitude)

		error = self.desired_state[2] - self.filtered_altitude

		# ---- integral with anti-windup clamp ------------------------------
		self.integral[2] += error * dt
		self.integral[2] = max(-self.integral_limit, min(self.integral_limit, self.integral[2]))

		# ---- derivative, low-pass filtered -------------------------------
		raw_derivative = (error - self.prev_error[2]) / dt
		self.filtered_derivative[2] += \
			self.derivative_filter * (raw_derivative - self.filtered_derivative[2])

		# ---- PID terms ---------------------------------------------------
		p_term = self.Kp[2] * error
		i_term = self.Ki[2] * self.integral[2]
		d_term = self.Kd[2] * self.filtered_derivative[2]

		# Larger whycode z = lower drone, so negate the controller output.
		out = -(p_term + i_term + d_term)

		# ---- output saturation -------------------------------------------
		# Limit the deviation from hover before adding it to the RC value.
		max_deviation = self.hover_value - self.min_value          # 500
		max_deviation = min(max_deviation, self.max_value - self.hover_value)  # 500
		out = max(-max_deviation, min(max_deviation, out))

		throttle = int(round(self.hover_value + out))
		throttle = max(self.min_value, min(self.max_value, throttle))

		# Conditional integration: if the command is already saturated and the
		# error would push it further into saturation, unwind the integrator.
		if (throttle >= self.max_value and out > 0) or (throttle <= self.min_value and out < 0):
			self.integral[2] -= error * dt
			self.integral[2] = max(-self.integral_limit, min(self.integral_limit, self.integral[2]))

		self.cmd.rc_throttle = throttle
		self.prev_error[2] = error

		# ---- publish ------------------------------------------------------
		self.pos_error.throttle_error = error
		self.pos_error.roll_error = self.desired_state[0] - self.current_state[0]
		self.pos_error.pitch_error = self.desired_state[1] - self.current_state[1]

		self.command_pub.publish(self.cmd)
		self.pos_error_pub.publish(self.pos_error)


def main(args=None):
	import argparse
	parser = argparse.ArgumentParser(description='Task 1B altitude PID controller')
	parser.add_argument('--kp', type=float, default=None)
	parser.add_argument('--ki', type=float, default=None)
	parser.add_argument('--kd', type=float, default=None)
	parsed, ros_args = parser.parse_known_args(args)

	rclpy.init(args=ros_args)
	controller = Task1BController()
	controller.apply_gain_overrides(parsed.kp, parsed.ki, parsed.kd)

	try:
		rclpy.spin(controller)
	except KeyboardInterrupt:
		controller.get_logger().info('KeyboardInterrupt, shutting down.\n')
	except ExternalShutdownException:
		# Context already torn down (e.g. SIGTERM); exit quietly.
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