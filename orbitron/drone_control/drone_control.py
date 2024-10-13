

import time
import math
import logging

from dronekit import connect, VechiceMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

class Drone:

	def __init__(self, serial_address: str, baud: int):
		self.vehicle = connect(serial_address, wait_ready=True, baud=baud)

	def arm_and_takeoff(self, target_altitude):
		""" Arms the vehicle and takes off to target_altitude"""
		logging.info("Performing pre-arm checks...")

		while not self.vehicle.is_armable:
			logging.info("Waiting for vechice to initialize...")
			time.sleep(1)

		print("Arming rotors...")
		self.vehicle.mode = VechiceMode("GUIDED")
		self.vehicle.armed = True

		while not self.vehicle.armed:
			logging.info("Waiting for arming...")
			time.sleep(1)

		logging.info("Taking off...")
		self.vehicle.simple_takeoff(target_altitude)

		while True:
			logging.info("Altitude: ", self.vehicle.location.global_relative_frame.alt)
			if (self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95):
				logging.info("Reached target Altitude")
				break
			time.sleep(1)

	def condition_yaw(self, heading, relative=False):
		"""
		Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees)

		This method sets an absolute heading by default, but can be changed by setting the relative param to `True`. By default the yaw of the vehicle will follow the direction of travel. After setting the yaw, there is no way to return to the default
		yaw "follow direction of travel" behaviour
		(https://github.com/diydrones/ardupilot/issues/2427)
		"""
		if relative:
			is_relative = 1
		else:
			is_relative = 0

		msg = self.vehicle.message_factory.command_long_encode(
			0,
			0, # component
			mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
			0, # confirmation
			heading, # yaw in degrees
			0, # yaw speed deg/s
			1, # direction -1 ccw, 1 cw
			is_relative, # relative offset, absolute angle
			0,
			0,
			0,
		)
		self.vehicle.send_mavlink(msg)

	def set_roi(self, location):
		"""
		Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a specified region of interest (LocationGlobal)
		The vehicle may also turn to face the ROI
		"""

		msg = self.vehicle.message_factory.command_long_encode(
			0,
			0,
			mavutil.mavlink.MAV_CMD_DO_SET_ROI,
			0,
			0,
			0,
			0,
			0,
			location.lat,
			location.lon,
			location.alt,
		)
		self.vehicle.send_mavlink(msg)

	def get_location_meters(self, original_location, dNorth, dEast):
		""" Returns a LocationGloal object containing the lat/lon `dNorth` and `dEast` meters from the specified original location
		"""
		earth_radius = 6378137.0
		dLat = dNorth / earth_radius
		dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

		newlat = original_location.lat + (dLat * 180 / math.pi)
		newlon = original_location.lon + (dLon * 180 / math.pi)
		if type(original_location) is LocationGlobal:
			target_location = LocationGlobal(newlat, newlon, original_location.alt)
		elif type(original_location) is LocationGlobalRelative(newlat, newlon, original_location.alt)
		else:
			raise Exception("Invalid location object passed")

		return target_location

	def get_distance_meters(self, location1, location2):
		dlat = location2.lat - location1.lat
		dlon = location2.lon - location1.lon
		return math.sqrt((dlat * dlat) + (dlon * dlon)) * 1.113195e5

	def get_bearing(self, location1, location2):
		off_x = location2.lon - location1.lon
		off_y = location2.lat - location1.lat
		brearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
		if brearing < 0:
			brearing += 360.00
		return brearing

	