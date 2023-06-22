# This is to help vscode
from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from robot import Robot

import math
from limelight import Limelight

from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer
import wpimath.geometry
import const

def _wait(time):
	timer = Timer()
	timer.start()
	while not timer.hasElapsed(time):
		yield

def _stationary_spin(robot: "Robot", rotation: float, abs_tol: float = 2):
	first_time = True
	while not abs(robot.drivetrain.angle_pid.getPositionError()) < abs_tol or first_time:
		yield
		first_time = False
		robot.drivetrain.drive_with_pid(
			wpimath.geometry.Translation2d(0, 0),
			rotation
		)

def score_high(robot: "Robot"):
	robot.drivetrain.set_yaw(180)

def balance(robot: "Robot",towards_front,facing_front):
	yield from robot.drivetrain.balance_coroutine(towards_front=towards_front, facing_front=facing_front)

def go_all_the_way_over(robot: "Robot", towards_front, rotation, timeout = 0.1):
	speed = 2
	def roll():
		roll = robot.drivetrain.roll
		if rotation == 0:
			roll = -roll
		return roll
	while True:
		yield
		robot.drivetrain.drive_with_pid(
			wpimath.geometry.Translation2d(speed if towards_front else -speed, 0),
			rotation
		)
		if towards_front:
			if roll() < -5:
				break
		else:
			if roll() > 5:
				break
	while not math.isclose(roll(),0,abs_tol=2):
		yield
		robot.drivetrain.drive_with_pid(
			wpimath.geometry.Translation2d(speed if towards_front else -speed, 0),
			rotation
		)
	yield from _wait(timeout)
	robot.drivetrain.stop()

def go_over(robot: "Robot", towards_front, rotation):
	def roll():
		roll = robot.drivetrain.roll
		if rotation == 0:
			roll = -roll
		return roll
	while True:
		yield
		robot.drivetrain.drive_with_pid(
			wpimath.geometry.Translation2d(4 if towards_front else -4, 0),
			rotation
		)
		if towards_front:
			if roll() < -5:
				break
		else:
			if roll() > 5:
				break
	yield from _stationary_spin(robot, 0)


def go_all_the_way_over_and_balance(robot: "Robot"):
	yield from go_all_the_way_over(robot,True,0)
	yield from _wait(2)
	yield from balance(robot,towards_front=False,facing_front=True)

def side_to_balance(robot: "Robot",right_side: bool):
	while abs(robot.drivetrain.roll) < 5:
		robot.drivetrain.drive_with_pid(
			wpimath.geometry.Translation2d(-2, 2.5 if right_side else -2.5),
			0
		)
		yield
	robot.drivetrain.stop()

def no_auto(robot: "Robot"):
	yield
