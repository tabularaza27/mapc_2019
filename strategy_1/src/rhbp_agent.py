#!/usr/bin/env python2
import time
import numpy as np
import random
from collections import OrderedDict
import rospy

from mapc_ros_bridge.msg import RequestAction, GenericAction, SimStart, SimEnd, Bye
from behaviour_components.managers import Manager
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition
from behaviour_components.activators import BooleanActivator, GreedyActivator
from behaviour_components.goals import GoalBase

import global_variables

from agent_commons.behaviour_classes.exploration_behaviour import ExplorationBehaviour
from agent_commons.behaviour_classes.move_to_dispenser_behaviour import MoveToDispenserBehaviour
from agent_commons.behaviour_classes.dispense_behaviour import DispenseBehaviour
from agent_commons.behaviour_classes.attach_behaviour import AttachBehaviour
from agent_commons.behaviour_classes.reach_meeting_point_behaviour import ReachMeetingPointBehaviour
from agent_commons.behaviour_classes.connect_behaviour import ConnectBehaviour
from agent_commons.behaviour_classes.detach_behaviour import DetachBehaviour
from agent_commons.behaviour_classes.reach_goal_area_behaviour import ReachGoalAreaBehaviour
from agent_commons.behaviour_classes.submit_behaviour import SubmitBehaviour
from agent_commons.providers import PerceptionProvider
from agent_commons.agent_utils import get_bridge_topic_prefix
from agent_commons.sensor_manager import SensorManager

from classes.grid_map import GridMap
from classes.tasks.task_decomposition import update_tasks
from classes.communications import Communication
from classes.map_merge import mapMerge

import pickle

from classes.bid import Bid

import random


class RhbpAgent(object):
    """
    Main class of an agent, taking care of the main interaction with the mapc_ros_bridge
    """

    def __init__(self):
        ###DEBUG MODE###

        log_level = rospy.DEBUG if global_variables.DEBUG_MODE else rospy.INFO
        ################
        rospy.logdebug("RhbpAgent::init")

        rospy.init_node('agent_node', anonymous=True, log_level=log_level)

        self._agent_name = rospy.get_param('~agent_name', 'agentA2')  # default for debugging 'agentA1'

        self._agent_topic_prefix = get_bridge_topic_prefix(agent_name=self._agent_name)

        # ensure also max_parallel_behaviours during debugging
        self._manager = Manager(prefix=self._agent_name, max_parallel_behaviours=1)

        self.behaviours = []
        self.goals = []

        self.perception_provider = PerceptionProvider()

        # auction structure
        self.bids = {}
        self.number_of_agents = 2  # TODO: check if there's a way to get it automatically

        self._sim_started = False

        # agent attributes
        self.local_map = GridMap(agent_name=self._agent_name, agent_vision=5)  # TODO change to get the vision
        self.map_messages_buffer = []

        # instantiate the sensor manager passing a reference to this agent
        self.sensor_manager = SensorManager(self)

        # representation of tasks
        self.tasks = {}
        self.assigned_subtasks = []  # personal for the agent. the task at index 0 is the task the agent is currently executing

        # subscribe to MAPC bridge core simulation topics
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)

        rospy.Subscriber(self._agent_topic_prefix + "bye", Bye, self._bye_callback)

        rospy.Subscriber(self._agent_topic_prefix + "generic_action", GenericAction, self._callback_generic_action)

        # start communication class
        self._communication = Communication(self._agent_name)
        # Map topic
        self._pub_map = self._communication.start_map(self._callback_map)
        # Personal message topic
        self._pub_agents = self._communication.start_agents(self._callback_agents)
        # Auction topic
        self._pub_auction = self._communication.start_auction(self._callback_auction)
        # Task update topic
        self._pub_subtask_update = self._communication.start_subtask_update(self._callback_subtask_update)

        self._received_action_response = False

    def task_auctioning(self):
        """ Communicate the bids and assign the subtasks to the agents """
        count = 0
        for task_name, task_object in self.tasks.iteritems():
            if len(task_object.sub_tasks) <= self.number_of_agents and not task_object.auctioned:
                #rospy.loginfo(self._agent_name + "| -- Analyizing: " + task_name)
                # STEP 1: SEND ALL THE BIDS
                for sub in task_object.sub_tasks:
                    if sub.assigned_agent == None:
                        subtask_id = sub.sub_task_name
                        #rospy.loginfo(self._agent_name + "| ---- Bid needed for " + subtask_id)

                        # first calculate the already assigned sub tasks
                        # TODO improve the way of summing the bid value of already assigned tasks
                        bid_value = 0
                        for t in self.assigned_subtasks:
                            bid_value += self.calculate_subtask_bid(t)[0]

                        # add the current
                        current_bid, distance_to_dispenser, closest_dispenser_position = self.calculate_subtask_bid(sub)
                        bid_value += current_bid

                        # transform the coordinates in relative to the top_left
                        if closest_dispenser_position is not None:
                            closest_dispenser_position = self.local_map._from_matrix_to_relative(closest_dispenser_position, self.local_map.goal_top_left)
                        else:
                            # invalid dispenser position
                            closest_dispenser_position = np.array([-1000000, -1000000])
                        self._communication.send_bid(self._pub_auction, subtask_id, bid_value, distance_to_dispenser, closest_dispenser_position[0], closest_dispenser_position[1])
                
                # STEP 2: WAIT FOR ALL THE BIDS OR A TIMEOUT

                current_time = 0
                deadline = 0.3
                while count < len(task_object.sub_tasks) * self.number_of_agents and current_time < deadline:
                    count = 0
                    for key, value in self.bids.items():
                        if task_object.name in key:
                            count += len(self.bids[key])
                            #rospy.loginfo("IL BESTIA DI DIO: " + key + " - count: " + str(count))
                    
                    time.sleep(0.05)
                    current_time += 0.05
                
                # STEP 3: MANAGE ASSIGN

                #rospy.loginfo("ORO BENON")
                ass = self.assign_subtasks(self.bids,task_object.name)

                # STEP 4: ACTUALLY ASSIGN

                for sub in  task_object.sub_tasks:
                    for ass_subtask_name, value in ass.items():
                        if ass_subtask_name == sub.sub_task_name:
                            sub.assigned_agent = ass[ass_subtask_name]["assigned_agent"]
                            sub.distance_to_dispenser = ass[ass_subtask_name]["bid"].distance_to_dispenser
                            sub.closest_dispenser_position = ass[ass_subtask_name]["bid"].closest_dispenser_position

                            if self._agent_name == sub.assigned_agent:
                                self.assigned_subtasks.append(sub)

                            # rospy.loginfo(self._agent_name + "| ---- ALLL DONE: " + sub.sub_task_name)
                            # rospy.loginfo(self._agent_name + "| -------- AGENT: " + sub.assigned_agent)
                            # rospy.loginfo(self._agent_name + "| -------- DTD: " + str(sub.distance_to_dispenser))
                            # rospy.loginfo(self._agent_name + "| -------- CDP: " + str(sub.closest_dispenser_position))

                            del self.bids[sub.sub_task_name] # free memory

    def assign_subtasks(self,bids,current_task_name):
        current_task_name += "_" # to avoid amibiguities in the if (1)
        ret = {}
        assigned = []

        for subtask_name, value in bids.items():
            if current_task_name in subtask_name: # (1)
                ordered_subtask = OrderedDict(sorted(self.bids[subtask_name].items(), key=lambda x: (x[1].bid_value, x[0])))
                #rospy.loginfo(self._agent_name + "| IL PAPA CORRE DIETRO LA LEPRE: " + subtask_name)

                invalid = True                
                for agent_name, bid in ordered_subtask.items():
                    #rospy.loginfo(self._agent_name + "|----- " + agent_name + ": " + str(bid.bid_value))

                    if bid.bid_value != -1 and agent_name not in assigned:
                        invalid = False
                        ret[subtask_name] = {}
                        ret[subtask_name]["assigned_agent"] = agent_name
                        ret[subtask_name]["bid"] = bid
                        assigned.append(agent_name)

                        
                        '''rospy.loginfo("-------- ASSIGNED: " + ret[subtask_name]["assigned_agent"])
                        rospy.loginfo("-------- BID: " + str(ret[subtask_name]["bid"].bid_value))
                        rospy.loginfo("-------- DTD: " + str(ret[subtask_name]["bid"].distance_to_dispenser))
                        rospy.loginfo("-------- CDP: " + str(ret[subtask_name]["bid"].closest_dispenser_position))'''

                        break
                    
                if invalid: # if i find even just one invalid subtask then all the task can't be assigned, so return empty set
                    rospy.loginfo(self._agent_name + "| *** AT THE LEAST ONE GUY INVALID")
                    ret = {}
                    return ret

        return ret # means that all the subtask were valid and the dictionary is returned


    
    def map_merge(self):
        """ Merges the maps received from the other agents that discovered the goal area and this agent did too"""
        # process the maps in the buffer
        for msg in self.map_messages_buffer[:]:
            msg_id = msg.message_id
            map_from = msg.agent_id
            map_value = msg.map
            map_lm_x = msg.lm_x
            map_lm_y = msg.lm_y
            map_rows = msg.rows
            map_columns = msg.columns

            if map_from != self._agent_name and self.local_map.goal_area_fully_discovered:
                # map received
                maps = np.fromstring(map_value, dtype=int).reshape(map_rows, map_columns)
                rospy.logdebug(maps)
                map_received = np.copy(maps)
                # landmark received
                lm_received = np.array([map_lm_y, map_lm_x])
                # own landmark
                lm_own = self.local_map._from_relative_to_matrix(self.local_map.goal_top_left)
                # do map merge
                # Added new origin to function
                origin_own = np.copy(self.local_map.origin)
                merged_map, merged_origin = mapMerge(map_received, self.local_map._representation, lm_received, lm_own,
                                                     origin_own)
                self.local_map._representation = np.copy(merged_map)
                self.local_map.origin = np.copy(merged_origin)

            self.map_messages_buffer.remove(msg)

    def publish_map(self):
        map = self.local_map._representation
        top_left_corner = self.local_map._from_relative_to_matrix(self.local_map.goal_top_left)
        self._communication.send_map(self._pub_map, map.tostring(), top_left_corner[0], top_left_corner[1],
                                     map.shape[0], map.shape[1])  # lm_x and lm_y to get

    def start_rhbp_reasoning(self, start_time, deadline):
        self._received_action_response = False

        # self._received_action_response is set to True if a generic action response was received(send by any behaviour)
        while not self._received_action_response and rospy.get_rostime() < deadline:
            # wait until this agent is completely initialised
            if self._sim_started:  # we at least wait our max time to get our agent initialised

                # action send is finally triggered by a selected behaviour
                self._manager.step(guarantee_decision=True)
            else:
                rospy.sleep(0.1)

        if self._received_action_response:  # One behaviour replied with a decision
            duration = rospy.get_rostime() - start_time
            rospy.logdebug("%s: Decision-making duration %f", self._agent_name, duration.to_sec())

        elif not self._sim_started:  # Agent was not initialised in time
            rospy.logwarn("%s idle_action(): sim not yet started", self._agent_name)
        else:  # Our decision-making has taken too long
            rospy.logwarn("%s: Decision-making timeout", self._agent_name)


    def calculate_subtask_bid(self, subtask):
        """calculate bid value for a subtask based on the distance from the agent to the closest dispenser and the
        distance from that this dispenser to the meeting point ( for now that is always the goal area )

        If the agent has not discovered the goal area, he places an invalid bid of -1

        Args:
            subtask (SubTask): subtask object

        Returns:
            int: bid value of agent for the task
        """
        bid_value = -1
        pos = None
        min_dist = -1

        if self.local_map.goal_area_fully_discovered:
            required_type = subtask.type

            # find the closest dispenser
            pos, min_dist = self.local_map.get_closest_dispenser_position(required_type)

            if pos is not None:  # the distance to the closer dispenser has been calculated
                # add the distance to the goal

                meeting_point = self.local_map.goal_top_left  # TODO change the meeting point with communication
                end = np.array([meeting_point[0], meeting_point[1]], dtype=int)
                distance, path = self.local_map.get_distance_and_path(pos, end, return_path=True)

                bid_value = distance + min_dist  # distance from agent to dispenser + dispenser to goal

                # TODO save task parameters dinamically every step to set sensors
                # TODO uncomment this line and pass the position in coordinates relative to the top left of the goal area
                #subtask.closest_dispenser_position = pos
                subtask.meeting_point = end
                path_id = self.local_map._save_path(path)
                subtask.path_to_dispenser_id = path_id


        return bid_value, min_dist, pos

    def _sim_start_callback(self, msg):
        """
        here we could also evaluate the msg in order to initialize depending on the role etc.
        :param msg:  the message
        :type msg: SimStart
        """

        if not self._sim_started:  # init only once here

            rospy.loginfo(self._agent_name + " started")

            # creating the actual RHBP model
            self._initialize_behaviour_model()

        self._sim_started = True

    def _callback_generic_action(self, msg):
        """
        ROS callback for generic actions
        :param msg: ros message
        :type msg: GenericAction
        """
        self._received_action_response = True

    def _sim_end_callback(self, msg):
        """
        :param msg:  the message
        :type msg: SimEnd
        """
        #rospy.loginfo("SimEnd:" + str(msg))
        for g in self.goals:
            g.unregister()
        for b in self.behaviours:
            b.unregister()
        self._sim_started = False

    def _bye_callback(self, msg):
        """
        :param msg:  the message
        :type msg: Bye
        """
        #rospy.loginfo("Simulation finished")
        rospy.signal_shutdown('Shutting down {}  - Simulation server closed'.format(self._agent_name))

    def _action_request_callback(self, msg):
        """
        here we just trigger the decision-making and planning
        while tracking the available time and behaviour responses
        :param msg: the message
        :type msg: RequestAction
        """


        # calculate deadline for the current simulation step
        start_time = rospy.get_rostime()
        safety_offset = rospy.Duration.from_sec(0.2)  # Safety offset in seconds
        deadline_msg = rospy.Time.from_sec(msg.deadline / 1000.0)
        current_msg = rospy.Time.from_sec(msg.time / 1000.0)
        deadline = start_time + (deadline_msg - current_msg) - safety_offset

        self.perception_provider.update_perception(request_action_msg=msg)


        ### breakpoint after 30 steps to debug task subdivision every 30 steps
        if self.perception_provider.simulation_step % 30 == 0 and self.perception_provider.simulation_step > 0:
            rospy.logdebug('Simulationstep {}'.format(self.perception_provider.simulation_step))

        ###### UPDATE AND SYNCHRONIZATION ######

        # update tasks from perception
        self.tasks = update_tasks(current_tasks=self.tasks, tasks_percept=self.perception_provider.tasks,
                                  simulation_step=self.perception_provider.simulation_step)
        #rospy.loginfo("{} updated tasks. New amount of tasks: {}".format(self._agent_name, len(self.tasks)))

        # task auctioning
        self.task_auctioning()

        # map merging

        self.local_map.update_map(perception=self.perception_provider)
        self.map_merge()
        self.local_map._update_path_planner_representation(perception=self.perception_provider)
        self.local_map._update_distances()

        # send the map if perceive the goal
        if self.local_map.goal_area_fully_discovered:
            self.publish_map()

        # if last action was `connect` and result = 'success' then save the attached block
        if self.perception_provider.agent.last_action == "connect" and self.perception_provider.agent.last_action_result == "success":
            # TODO ADD THE BLOCK CONNECTED
            other_agent_name = self.perception_provider.agent.last_action_params[0]
            self.assigned_subtasks[0].is_connected = True
            # make the other guy's subtask completed and connected
            task_name = self.assigned_subtasks[0].parent_task_name
            current_task = self.tasks.get(task_name, None)
            a = 1
            for sub_task in current_task.sub_tasks:
                # TODO check which is the completed sub_task
                if sub_task.assigned_agent == other_agent_name:
                    sub_task.is_connected = True
                    sub_task.complete = True

        #TODO add attach behaviour effect, add blocks to attached blocks, without that,
        # agents with blocks get stuck

        # if last action was detach, detach the blocks
        if self.perception_provider.agent.last_action == "detach" and self.perception_provider.agent.last_action_result == "success":
            # TODO detach only the blcok in the direction of the detach
            self.local_map._attached_blocks = []
            self.assigned_subtasks.pop()


        # if last action was submit, detach the blocks
        if self.perception_provider.agent.last_action == "submit" and self.perception_provider.agent.last_action_result == "success":
            # TODO detach only the block in the direction of the task
            self.local_map._attached_blocks = []


        '''
        # send personal message test
        if self._agent_name == "agentA1":
            self._communication.send_message(self._pub_agents, "agentA2", "task", "[5,5]")

        '''

        # update the sensors before starting the rhbp reasoning
        self.sensor_manager.update_sensors()

        # dumped class
        if global_variables.DUMP_CLASS:
            test_case_number = '004'
            file_name = 'task_assignment_for_' + str(self.number_of_agents) + '_' + test_case_number
            file_object = open("/home/alvaro/Desktop/AAIP/mapc_workspace/src/group5/strategy_1/src/" \
                               + file_name + '.dat', "wb")
            pickle.dump(self.tasks, file_object)
            file_object.close()

        self.start_rhbp_reasoning(start_time, deadline)

    def _callback_map(self, msg):
        self.map_messages_buffer.append(msg)

    def _callback_agents(self, msg):
        msg_id = msg.message_id
        msg_from = msg.agent_id_from
        msg_type = msg.message_type
        msg_param = msg.params

        if msg.agent_id_to == self._agent_name:
            rospy.loginfo(
                self._agent_name + " received message from " + msg_from + " | id: " + msg_id + " | type: " + msg_type + " | params: " + msg_param)
            self._communication.send_message(self._pub_agents, msg_from, "received", msg_id)

    def _callback_subtask_update(self, msg):
        msg_id = msg.message_id
        msg_from = msg.agent_id
        command = msg.command
        message_subtask_id = msg.task_id

        if command == "done": # if gets a "done" command, then cycle all the subtasks when the task passed in the message is found then is set as complete
            for task_name, task_object in self.tasks.iteritems():
                for sub in task_object.sub_tasks:
                    current_subtask_id = sub.sub_task_name
                    if current_subtask_id == message_subtask_id:
                        sub.complete = True
                        break



    def _callback_auction(self, msg):
        msg_id = msg.message_id
        msg_from = msg.agent_id
        task_id = msg.task_id
        task_bid_value = msg.bid_value
        distance_to_dispenser = msg.distance_to_dispenser
        closest_dispenser_position_x = msg.closest_dispenser_position_x
        closest_dispenser_position_y = msg.closest_dispenser_position_y

        if task_id not in self.bids:
            self.bids[task_id] = OrderedDict()

        bid = Bid(task_bid_value,distance_to_dispenser,np.array([closest_dispenser_position_y,closest_dispenser_position_x]))
        self.bids[task_id][msg_from] = bid


    def _initialize_behaviour_model(self):
        """
        This function initialises the RHBP behaviour/goal model.
        """

        ### Exploration ##
        exploration_move = ExplorationBehaviour(name="exploration_move", agent_name=self._agent_name, rhbp_agent=self)
        self.behaviours.append(exploration_move)

        # assigned to a task
        exploration_move.add_precondition(Condition(sensor=self.sensor_manager.assigned_task_list_empty,
                                          activator=BooleanActivator(desiredValue=True)))

        exploration_move.add_effect(Effect(self.perception_provider.dispenser_visible_sensor.name, indicator=True))
        exploration_move.add_effect(Effect(self.sensor_manager.assigned_task_list_empty.name, indicator=True))

        ### Move to Dispenser ###
        move_to_dispenser = MoveToDispenserBehaviour(name="move_to_dispenser", agent_name=self._agent_name,
                                                     rhbp_agent=self)
        self.behaviours.append(move_to_dispenser)
        # assigned to a task precondition
        move_to_dispenser.add_precondition(
            Condition(self.sensor_manager.assigned_task_list_empty,
                      BooleanActivator(desiredValue=False))
        )
        # block not attached precondition
        move_to_dispenser.add_precondition(
            Condition(self.sensor_manager.attached_to_block,
                      BooleanActivator(desiredValue=False))
        )
        # not at the dispenser precondition
        move_to_dispenser.add_precondition(
            Condition(self.sensor_manager.at_the_dispenser,
                      BooleanActivator(desiredValue=False))
        )
        move_to_dispenser.add_effect(Effect(self.sensor_manager.at_the_dispenser.name, indicator=True))

        # # Our simple goal is to create more and more blocks
        # dispense_goal = GoalBase("dispensing", permanent=True,
        #                          conditions=[
        #                              Condition(self.sensor_manager.at_the_dispenser, GreedyActivator())],
        #                          planner_prefix=self._agent_name)
        # self.goals.append(dispense_goal)

        ### Requeste  block - Dispense ###
        dispense = DispenseBehaviour(name="dispense", agent_name=self._agent_name,
                                     rhbp_agent=self)
        self.behaviours.append(dispense)
        # assigned to a task precondition
        dispense.add_precondition(
            Condition(self.sensor_manager.assigned_task_list_empty,
                      BooleanActivator(desiredValue=False))
        )
        # block not attached precondition
        dispense.add_precondition(
            Condition(self.sensor_manager.attached_to_block,
                      BooleanActivator(desiredValue=False))
        )
        # not at the dispenser precondition
        dispense.add_precondition(
            Condition(self.sensor_manager.at_the_dispenser,
                      BooleanActivator(desiredValue=True))
        )
        # not next to block
        dispense.add_precondition(Condition(self.sensor_manager.next_to_block, BooleanActivator(desiredValue=False)))

        # effect of dispense is that agent is next to block
        dispense.add_effect(Effect(self.sensor_manager.next_to_block.name, indicator=True))


        # Our simple goal is to create more and more blocks
        # dispense_goal = GoalBase("dispensing", permanent=True,
        #                          conditions=[
        #                              Condition(self.sensor_manager.attached_to_block, GreedyActivator())],
        #                          planner_prefix=self._agent_name)
        # self.goals.append(dispense_goal)

        #### Attach to Block ###
        attach = AttachBehaviour(name="attach", agent_name=self._agent_name, rhbp_agent=self)
        self.behaviours.append(attach)

        # Preconditions
        # assigned to a task
        attach.add_precondition(Condition(sensor=self.sensor_manager.assigned_task_list_empty,
                                          activator=BooleanActivator(desiredValue=False)))
        # is not yet attached to a block of type of the current task
        attach.add_precondition(Condition(sensor=self.sensor_manager.attached_to_block, activator=BooleanActivator(desiredValue=False)))
        # is next to a block
        attach.add_precondition(Condition(sensor=self.sensor_manager.next_to_block, activator=BooleanActivator(desiredValue=True)))
        # has free capacity to attach
        attach.add_precondition(Condition(sensor=self.sensor_manager.fully_attached, activator=BooleanActivator(desiredValue=False)))

        # effect of attach is that agent is attached to a block
        attach.add_effect(Effect(self.sensor_manager.attached_to_block.name, indicator=True))

        # attach_goal = GoalBase("attaching", permanent=True,
        #                          conditions=[
        #                              Condition(self.sensor_manager.attached_to_block, GreedyActivator())],
        #                          planner_prefix=self._agent_name)
        # self.goals.append(attach_goal)

        #### Reach meeting point ###
        reach_meeting_point = ReachMeetingPointBehaviour(name="reach_meeting_point", agent_name=self._agent_name, rhbp_agent=self)
        self.behaviours.append(reach_meeting_point)
        # assigned to a task
        reach_meeting_point.add_precondition(Condition(sensor=self.sensor_manager.assigned_task_list_empty,
                                          activator=BooleanActivator(desiredValue=False)))
        # have the block attached
        reach_meeting_point.add_precondition(Condition(sensor=self.sensor_manager.attached_to_block,
                                          activator=BooleanActivator(desiredValue=True)))

        # the shape is not complete
        reach_meeting_point.add_precondition(Condition(sensor=self.sensor_manager.shape_complete,
                                                       activator=BooleanActivator(desiredValue=False)))
        # has not reached the meeting point already
        reach_meeting_point.add_precondition(Condition(sensor=self.sensor_manager.at_meeting_point,
                                                       activator=BooleanActivator(desiredValue=False)))

        # effect is moving till the agent reaches the meeting point
        reach_meeting_point.add_effect(Effect(self.sensor_manager.at_meeting_point.name, indicator=True))

        # at_meeting_point_goal = GoalBase("reach_meeting_point_goal", permanent=True,
        #                        conditions=[
        #                            Condition(self.sensor_manager.at_meeting_point, GreedyActivator())],
        #                        planner_prefix=self._agent_name)
        # self.goals.append(at_meeting_point_goal)

        #### Connect ###
        connect = ConnectBehaviour(name="connect", agent_name=self._agent_name,
                                                         rhbp_agent=self)
        self.behaviours.append(connect)
        # assigned to a task
        connect.add_precondition(Condition(sensor=self.sensor_manager.assigned_task_list_empty,
                                                       activator=BooleanActivator(desiredValue=False)))
        # have the block attached
        connect.add_precondition(Condition(sensor=self.sensor_manager.attached_to_block,
                                                       activator=BooleanActivator(desiredValue=True)))

        # connection not completed
        connect.add_precondition(Condition(sensor=self.sensor_manager.connect_successful,
                                                       activator=BooleanActivator(desiredValue=False)))
        # has reached the meeting point
        connect.add_precondition(Condition(sensor=self.sensor_manager.at_meeting_point,
                                                       activator=BooleanActivator(desiredValue=True)))

        # effect is creating the shape
        connect.add_effect(Effect(self.sensor_manager.connect_successful.name, indicator=True))
        connect.add_effect(Effect(self.sensor_manager.shape_complete.name, indicator=True))
        """
        connect_successful_goal = GoalBase("connect_successful_goal", permanent=True,
                                         conditions=[
                                             Condition(self.sensor_manager.connect_successful, GreedyActivator())],
                                         planner_prefix=self._agent_name)
        self.goals.append(connect_successful_goal)
        """
        #### Detach to Block ###
        detach = DetachBehaviour(name="detach", agent_name=self._agent_name, rhbp_agent=self)
        self.behaviours.append(detach)

        # Preconditions
        # assigned to a task
        detach.add_precondition(Condition(sensor=self.sensor_manager.assigned_task_list_empty,
                                          activator=BooleanActivator(desiredValue=False)))
        # connect action was succesful
        detach.add_precondition(
            Condition(sensor=self.sensor_manager.connect_successful, activator=BooleanActivator(desiredValue=True)))
        # is NOT the agent assigned to submit
        detach.add_precondition(
            Condition(sensor=self.sensor_manager.can_submit, activator=BooleanActivator(desiredValue=False)))

        # effect of attach is that agent is attached to a block
        detach.add_effect(Effect(self.sensor_manager.points.name, indicator=True))

        #### Go to goal area ###
        go_to_goal_area = ReachGoalAreaBehaviour(name="go_to_goal_area", agent_name=self._agent_name, rhbp_agent=self)
        self.behaviours.append(go_to_goal_area)

        # Preconditions
        # assigned to a task
        go_to_goal_area.add_precondition(Condition(sensor=self.sensor_manager.assigned_task_list_empty,
                                          activator=BooleanActivator(desiredValue=False)))
        # connect action was succesful
        go_to_goal_area.add_precondition(
            Condition(sensor=self.sensor_manager.shape_complete, activator=BooleanActivator(desiredValue=True)))

        # is the agent assigned to submit
        go_to_goal_area.add_precondition(
            Condition(sensor=self.sensor_manager.can_submit, activator=BooleanActivator(desiredValue=True)))

        # is the agent assigned to submit
        go_to_goal_area.add_precondition(
            Condition(sensor=self.sensor_manager.at_goal_area, activator=BooleanActivator(desiredValue=False)))

        # effect of attach is that agent is attached to a block
        go_to_goal_area.add_effect(Effect(self.sensor_manager.at_goal_area.name, indicator=True))

        #### Submit ###
        submit = SubmitBehaviour(name="submit", agent_name=self._agent_name, rhbp_agent=self)
        self.behaviours.append(submit)

        # Preconditions
        # assigned to a task
        submit.add_precondition(Condition(sensor=self.sensor_manager.assigned_task_list_empty,
                                                   activator=BooleanActivator(desiredValue=False)))
        # have the block attached
        connect.add_precondition(Condition(sensor=self.sensor_manager.attached_to_block,
                                           activator=BooleanActivator(desiredValue=True)))
        # connect action was succesful
        submit.add_precondition(
            Condition(sensor=self.sensor_manager.shape_complete, activator=BooleanActivator(desiredValue=True)))

        # is the agent assigned to submit
        submit.add_precondition(
            Condition(sensor=self.sensor_manager.can_submit, activator=BooleanActivator(desiredValue=True)))

        # is the agent at the goal area
        submit.add_precondition(
            Condition(sensor=self.sensor_manager.at_goal_area, activator=BooleanActivator(desiredValue=True)))

        # effect of attach is that agent is attached to a block
        submit.add_effect(Effect(self.sensor_manager.points.name, indicator=True))


        make_points_goal = GoalBase("make_points_goal", permanent=True,
                                   conditions=[
                                       Condition(self.sensor_manager.points, GreedyActivator())],
                                   planner_prefix=self._agent_name)
        self.goals.append(make_points_goal)
        """
        HERE
        move_to_dispenser = MoveToDispenserBehaviour()
        self.behaviours.append(move_to_dispenser)
        move_to_dispenser.add_precondition(
            Condition()
        )
        
        # Random Move/Exploration
        random_move = RandomMove(name="random_move", agent_name=self._agent_name)
        self.behaviours.append(random_move)
        random_move.add_effect(Effect(self.perception_provider.dispenser_visible_sensor.name, indicator=True))

        
        # Moving to a dispenser if in vision range
        move_to_dispenser = MoveToDispenser(name="move_to_dispense", perception_provider=self.perception_provider,
                                            agent_name=self._agent_name)
        self.behaviours.append(move_to_dispenser)
        move_to_dispenser.add_effect(
            Effect(self.perception_provider.closest_dispenser_distance_sensor.name, indicator=-1, sensor_type=float))
        move_to_dispenser.add_precondition(
            Condition(self.perception_provider.dispenser_visible_sensor, BooleanActivator(desiredValue=True)))
        move_to_dispenser.add_precondition(Condition(self.perception_provider.closest_dispenser_distance_sensor,
                                            ThresholdActivator(isMinimum=True, thresholdValue=2)))

        # Dispense a block if close enough
        dispense = Dispense(name="dispense", perception_provider=self.perception_provider, agent_name=self._agent_name)
        self.behaviours.append(dispense)
        dispense.add_effect(
            Effect(self.perception_provider.number_of_blocks_sensor.name, indicator=+1, sensor_type=float))

        dispense.add_precondition(Condition(self.perception_provider.closest_dispenser_distance_sensor,
                                            ThresholdActivator(isMinimum=False, thresholdValue=1)))

        # Our simple goal is to create more and more blocks
        dispense_goal = GoalBase("dispensing", permanent=True,
                                 conditions=[Condition(self.perception_provider.number_of_blocks_sensor, GreedyActivator())],
                                 planner_prefix=self._agent_name)
        self.goals.append(dispense_goal)
        """
        # TODO meeting thing
        # if self.assigned_tasks is not None:
        #     task = self.assigned_tasks[self.assigned_tasks[0].parent_task_name]




if __name__ == '__main__':
    try:
        random.seed(30)
        rhbp_agent = RhbpAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
