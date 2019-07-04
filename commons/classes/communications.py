#!/usr/bin/env python2

import rospy
import uuid
from mapc_rhbp_manual_player.msg import map_communication, auction_communication, personal_communication, \
    general_communication, subtask_update_communication


class Communication:
    # for synchronization
    wait = False

    ### PUBLIC METHODS ###
    def __init__(self, agent_name):
        """
        Initialization of the communication class
        Args:
            agent_name (string): name of the agent

        Returns: void
        """

        self._agent_name = agent_name

    def start_auction(self, callback_function, topic_name="auction", message_type=auction_communication):
        """
        Initialization of the auction subscriber and publisher
        Args:
            callback_function (function): function to handle the message received in the topic
            topic_name (string): name of the topic
            message_type (ros_msg): type of the message accepted by the topic

        Returns: the publisher handle for the topic
        """

        rospy.Subscriber(topic_name, message_type, callback_function)
        pub_auction = rospy.Publisher(topic_name, message_type, queue_size=10)

        return pub_auction

    def start_map(self, callback_function, topic_name="map", message_type=map_communication):
        """
        Initialization of the map subscriber and publisher
        Args:
            callback_function (function): function to handle the message received in the topic
            topic_name (string): name of the topic
            message_type (ros_msg): type of the message accepted by the topic

        Returns: the publisher handle for the topic
        """

        rospy.Subscriber(topic_name, message_type, callback_function)
        pub_map = rospy.Publisher(topic_name, message_type, queue_size=10)

        return pub_map

    def start_agents(self, callback_function, topic_name="agents", message_type=personal_communication):
        """
        Initialization of the intra agents communication subscriber and publisher
        Args:
            callback_function (function): function to handle the message received in the topic
            topic_name (string): name of the topic
            message_type (ros_msg): type of the message accepted by the topic

        Returns: 
            publisher: the publisher handle for the topic
        """

        rospy.Subscriber(topic_name, message_type, callback_function)
        pub_agents = rospy.Publisher(topic_name, message_type, queue_size=10)

        return pub_agents
    
    def start_subtask_update(self, callback_function, topic_name="subtask_update", message_type=subtask_update_communication):
        """
        Initialization of the task update communication subscriber and publisher
        Args:
            callback_function (function): function to handle the message received in the topic
            topic_name (string): name of the topic
            message_type (ros_msg): type of the message accepted by the topic

        Returns: 
            publisher: the publisher handle for the topic
        """

        rospy.Subscriber(topic_name, message_type, callback_function)
        pub_agents = rospy.Publisher(topic_name, message_type, queue_size=10)

        return pub_agents

    def send_map(self, publisher, map, lm_y, lm_x, rows, columns):
        """
        Send the map through the map topic
        Args:
            publisher (publisher): publisher handle returned from the function start_map
            map (string): string version of the map matrix

        Returns: void
        """

        if self.wait == False:
            msg = map_communication()
            msg.message_id = self.generateID()
            msg.agent_id = self._agent_name
            msg.map = map
            msg.lm_x = lm_x
            msg.lm_y = lm_y
            msg.rows = rows
            msg.columns = columns
            publisher.publish(msg)

    def send_message(self, publisher, id_to, message_type, params):
        """
        Send a personal message through the intra agent communication topic
        Args:
            publisher (publisher): publisher handle returned from the function start_agents
            id_to (string): id of the agent to send the message to
            message_type (string): type of message sent
            params (string): params of the message

        Returns: void
        """

        if self.wait == False:
            msg = personal_communication()
            msg.message_id = self.generateID()
            msg.agent_id_from = self._agent_name
            msg.agent_id_to = id_to
            msg.message_type = message_type
            msg.params = params
            publisher.publish(msg)
            self.lock()

    def send_bid(self, publisher, task_id, bid_value, distance_to_dispenser, closest_dispenser_position_x, closest_dispenser_position_y):
        """
        Send the bid through the auction communication topic
        Args:
            publisher (publisher): publisher handle returned from the function start_agents
            task_id (string): id of the task subject of the bid
            bid_value (int): value of the bid

        Returns: void
        """

        msg = auction_communication()
        msg.message_id = self.generateID()
        msg.agent_id = self._agent_name
        msg.task_id = task_id
        msg.bid_value = bid_value
        msg.distance_to_dispenser = distance_to_dispenser
        msg.closest_dispenser_position_x = closest_dispenser_position_x
        msg.closest_dispenser_position_y = closest_dispenser_position_y
        publisher.publish(msg)

    def send_subtask_update(self, publisher, command, task_id):
        """
        Send the bid through the auction communication topic
        Args:
            publisher (publisher): publisher handle returned from the function start_agents
            task_id (string): id of the task subject of the bid
            bid_value (int): value of the bid

        Returns: void
        """

        msg = subtask_update_communication()
        msg.message_id = self.generateID()
        msg.agent_id = self._agent_name
        msg.command = command
        msg.task_id = task_id
        publisher.publish(msg)

    def lock(self):
        """
        Lock the intra agent communication behaviour to stop sending messages

        Returns: void
        """

        self.wait = True

    def unlock(self):
        """
        Unlock the intra agent communication behaviour to enable sending messages 

        Returns: void
        """

        self.wait = False

    def generateID(self):
        """
        Generate unique ID for the messages

        Returns: void
        """

        return str(uuid.uuid1())
