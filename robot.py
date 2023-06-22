#! python3
"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2023
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/Team364/BaseFalconSwerve/
"""

DEBUG = True

import logging

# Import our files

import commands2
import wpilib
from wpilib import Timer, DataLogManager, DriverStation
import wpimath.geometry
import const
import oi
import math
import ntcore
import subsystems.drivetrain
import subsystems.limelight

import inspect
import autoroutines

log = logging.getLogger("robot")

class Robot(commands2.TimedCommandRobot):
	"""
	Main robot class.

	This is the central object, holding instances of all the robot subsystem
	and sensor classes.

	It also contains the methods for autonomous and
	teloperated modes, called during mode changes and repeatedly when those
	modes are active.

	The one instance of this class is also passed as an argument to the
	various other classes, so they have full access to all its properties.
	"""

	def robot_start(self):
		# Networktables
		nt_inst = ntcore.NetworkTableInstance.getDefault()
		nt_inst.startServer()
		self.nt_robot = nt_inst.getTable("SmartDashboard")
		#self.nt_robot.putString('led_mode', 'off')

		# Match Stuff
		self.match_time = -1
		const.IS_SIMULATION = self.isSimulation()

		# Command scheduler
		self.scheduler = commands2.CommandScheduler.getInstance()

		# subsystems
		self.drivetrain = subsystems.drivetrain.Drivetrain(self)
		self.limelight = subsystems.limelight.Limelight_Wrapper()
		self.leds = subsystems.leds.LEDs(self)

		self.subsystems = [
			self.drivetrain,
			self.leds,
		]

		self.scheduler.registerSubsystem(self.subsystems)

		### OTHER ###
		self.driverstation = wpilib.DriverStation
		self.oi = oi.OI(self)

		self.match_time = -1

		### LOGGING ###

		DataLogManager.start()
		DriverStation.startDataLog(DataLogManager.getLog())

		@self.addPeriodic(period=0.25, offset=0)
		def _():
			self.log()

		@self.addPeriodic(period=0.05, offset=-.01)
		def _leds():
			self.leds.periodicX()

		self.auto_chooser = wpilib.SendableChooser()

		for name, value in inspect.getmembers(autoroutines):
			if inspect.isfunction(value) and value.__module__ == autoroutines.__name__:
				published_name = name#.replace("_", " ")
				if not len(inspect.signature(value).parameters) == 1 or published_name.startswith("_"):
					continue
				self.auto_chooser.addOption(published_name, value)

				self.auto_chooser.setDefaultOption(name, value)


		# self.auto_chooser.addOption("NAME", COMMAND)
		wpilib.SmartDashboard.putData("Auto Chooser", self.auto_chooser)

		while True:
			yield
			self.scheduler.run()

			#print(self.oi.driver1.RIGHT_JOY_Y())

	### DISABLED ###

	def disabled_mode(self):
		self.scheduler.cancelAll()
		self.drivetrain.gyro_offset = self.drivetrain.gyro.getRoll()
		self.leds.set_mode(self.leds.MODE_WAITING)

		for subsystem in self.subsystems:
			subsystem.stop()

		while True: # Needs to continuously call while robot is disabled.
			yield

	### AUTONOMOUS ###
	def autonomous_mode(self):
		self.scheduler.cancelAll()
		self.scheduler.schedule(self.auto_chooser.getSelected()(self)) # type: ignore

	### TELEOPERATED ###
	def teleop_mode(self):
		self.scheduler.cancelAll()

		while True:
			yield

	### MISC ###

	def log(self):
		"""
		Logs some info to shuffleboard, and standard output
		"""
		for s in self.subsystems:
			s.log()

		self.match_time = self.driverstation.getMatchTime()
		wpilib.SmartDashboard.putNumber("Match Time", self.match_time)


### MAIN ###

if __name__ == "__main__":
	wpilib.run(Robot)
