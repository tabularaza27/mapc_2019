#!/usr/bin/env python2

import rospy
from mapc_rhbp_manual_player.msg import personal_communication, general_communication

class Communication:
	wait = False
	#_answer = ""

	def __init__(self, agent_name):
		#rospy.Subscriber("synchro", general_communication, self.manage_synchro)
		#self._pub_synchro = rospy.Publisher("synchro", general_communication, queue_size=10)
		self._agent_name = agent_name

	def start_map(self, callback_function, topic_name = "map", message_type = general_communication):
		rospy.Subscriber(topic_name, message_type, callback_function)
		pub_map = rospy.Publisher(topic_name, message_type, queue_size=10)

		return pub_map
	
	def start_agents(self, callback_function, topic_name = "agents", message_type = personal_communication):
		rospy.Subscriber(topic_name, message_type, callback_function)
		pub_agents = rospy.Publisher(topic_name, message_type, queue_size=10)

		return pub_agents

	def send_map(self, publisher, map):
		if self.wait == False:
			msg = general_communication()
			msg.agent_id = self._agent_name
			msg.message = map
			publisher.publish(msg)
			#self._pub_synchro.publish(msg)

	def send_message(self, publisher, id_to, message):
		if self.wait == False:
			msg = personal_communication()
			msg.agent_id_from = self._agent_name
			msg.agent_id_to = id_to
			msg.message = message
			publisher.publish(msg)

	def lock(self):
		self.wait = True

	def unlock(self):
		self.wait = False

	"""
	def wait_for_answer(self, answer):
		self._wait = True
		self._answer = answer
	
	def manage_synchro(self, msg):
		if wait == True and msg.message == answer:
			wait = False
	"""

	