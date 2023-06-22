"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2023
Code for robot ""
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

# This is to help vscode
from typing import TYPE_CHECKING

if TYPE_CHECKING:
	from robot import Robot  # type: ignore

import wpilib
import wpilib.interfaces
from commands2 import ParallelCommandGroup, Subsystem, WaitCommand
from commands2.button import Button
from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import math

import const


# Controls
from commands2.button import XboxCommandController

###  IMPORTS ###

class OI:
	"""
	Operator Input - This class ties together controls and commands.
	"""

	def __init__(self, robot: "Robot"):
		self.robot = robot

		# Controllers
		self.driver1 = XboxCommandController(0)

		# self.driver1.LEFT_JOY_Y.setInverted(True)

		self.driver1.LEFT_JOY_X.setDeadzone(0.02)
		self.driver1.LEFT_JOY_Y.setDeadzone(0.02)
		self.driver1.RIGHT_JOY_X.setDeadzone(0.05)
		self.driver1.RIGHT_JOY_Y.setDeadzone(0.05)

		### Driving ###
		self.cardinal = 0
		self.cardinal_directing = False

		@self.rumble_button.whenPressed
		def _():
			timer = Timer()
			timer.start()
			self.driver1.setRumble(1)
			while not timer.hasElapsed(0.2):
				yield
			self.driver1.setRumble(0)

		@self.robot.intake.setDefaultCommand
		def _():
			robot.intake.intake(0.3)
			while True:
				yield

		@self.robot.drivetrain.setDefaultCommand
		def _():
			target_angle = robot.drivetrain.get_yaw().degrees()

			while True:
				yield
				def square(x):
					return abs(x) * x
				forward_back = -square(self.driver1.LEFT_JOY_Y())
				left_right = -square(self.driver1.LEFT_JOY_X())
				if self.driver1.RIGHT_BUMPER():
					forward_back *= 0.5
					left_right *= 0.5
				rotate = -self.driver1.RIGHT_JOY_X()
				if self.cardinal_directing and rotate == 0:
					self.robot.drivetrain.drive_with_pid(
						Translation2d(forward_back, left_right) * 6,
						self.cardinal
					)
				else:
					self.cardinal_directing = False
					self.robot.drivetrain.drive(
						Translation2d(forward_back, left_right) * 6,
						rotate * 6,
						field_relative=True,
						is_open_loop=True,
					)
				# y = -self.driver1.RIGHT_JOY_Y()
				# x = self.driver1.RIGHT_JOY_X()
				# if x**2 + y**2 > 0.8:
				# 	target_angle = math.degrees(math.atan2(y,x)) - 90
				# self.robot.drivetrain.drive_with_pid(
				# 	Translation2d(forward_back, left_right) * const.SWERVE_MAX_SPEED,
				# 	target_angle
				# )

		@self.driver1.LEFT_BUMPER.whenPressed
		def _():
			robot.drivetrain.set_yaw(0)

		@self.driver1.LEFT_TRIGGER_AS_BUTTON.whenPressed
		def _():
			self.cardinal_directing = True
			self.cardinal = 180

		@self.driver1.RIGHT_TRIGGER_AS_BUTTON.whenPressed
		def _():
			self.cardinal_directing = True
			self.cardinal = 0

		# Allows running auto mode in teleop
		@self.driver1.START.whenPressed(requirements=[robot.arm, robot.intake, robot.drivetrain, robot.leds])
		def _():
			yield from self.robot.auto_chooser.getSelected()(self.robot) # type: ignore

	def log(self):
		pass
