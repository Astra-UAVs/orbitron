import time
from pymavlink import mavutil

class MAVLinkInterface:
	def __init__(self, connection_string: str):
		self.connection = mavutil.mavlink_connection(connection_string)

	def wait_for_heartbeat(self):
		print("Waiting for heartbeat...")
		self.connection.wait_heartbeat()
		print("Heartbeat received from system (system %u component %u)" % (self.connection.target_system, self.connection.target_component))

	def send_command_long(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
		self.connection.mav.command_long_send(
			self.connection.target_system,
			self.connection.target_component,
			command,
			0,  # Confirmation
			param1, param2, param3, param4, param5, param6, param7
		)

	def read_messages(self):
		while True:
			msg = self.connection.recv_match(blocking=True)
			if msg:
				print(msg)

	def read_imu_data(self):
		while True:
			msg = self.connection.recv_match(type='RAW_IMU', blocking=True)
			if msg:
				print(f"IMU Data: Time: {msg.time_usec}, xacc: {msg.xacc}, yacc: {msg.yacc}, zacc: {msg.zacc}, xgyro: {msg.xgyro}, ygyro: {msg.ygyro}, zgyro: {msg.zgyro}")

	def read_position_data(self):
		while True:
			msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
			if msg:
				print(f"Position Data: Lat: {msg.lat / 1e7}, Lon: {msg.lon / 1e7}, Alt: {msg.alt / 1e3}, Relative Alt: {msg.relative_alt / 1e3}, Vx: {msg.vx}, Vy: {msg.vy}, Vz: {msg.vz}")

if __name__ == "__main__":
	connection_string = "tcp:127.0.0.1:5760"
	mavlink_interface = MAVLinkInterface(connection_string)
	mavlink_interface.wait_for_heartbeat()
	mavlink_interface.send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=1)
	mavlink_interface.read_imu_data()
	mavlink_interface.read_position_data()
	mavlink_interface.read_messages()