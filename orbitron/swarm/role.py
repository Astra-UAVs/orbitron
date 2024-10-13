

class RoleAssignment:
	def __init__(self, drone_id):
		self.drone_id = drone_id
		self.role = 'follower'

	def assign_leader(self):
		self.role = 'leader'

	def assign_follower(self):
		pass

	def get_role(self):
		return self.role

