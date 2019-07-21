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
from classes.block import Block

from classes.grid_map import GridMap
from classes.tasks.update_tasks import update_tasks
from classes.communications import Communication
from classes.map_communication import MapCommunication

import pickle

from classes.bid import Bid
from classes.auction import Auction

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

        # start communication class
        self._communication = Communication(self._agent_name)
        # Task update topic
        self._pub_subtask_update = self._communication.start_subtask_update(self._callback_subtask_update)

        # auction structure

        self.auction = Auction(self)
        self.number_of_agents = 2  # TODO: check if there's a way to get it automatically


        self.map_communication = MapCommunication(self)


        self._sim_started = False

        # agent attributes
        self.local_map = GridMap(agent_name=self._agent_name, agent_vision=5)  # TODO change to get the vision
        

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

        self._received_action_response = False

    

    


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

        # remove assigned subtasks if task is deleted
        for assigned_subtask in self.assigned_subtasks[:]:
            if assigned_subtask.parent_task_name not in self.tasks:
                self.assigned_subtasks.remove(assigned_subtask)
                
        #rospy.loginfo("{} updated tasks. New amount of tasks: {}".format(self._agent_name, len(self.tasks)))

        # task auctioning
        self.auction.task_auctioning()

        # map merging

        self.local_map.update_map(perception=self.perception_provider)
        self.map_communication.map_merge()
        self.local_map._update_path_planner_representation(perception=self.perception_provider)
        self.local_map._update_distances()

        # send the map if perceive the goal
        if self.local_map.goal_area_fully_discovered:
            self.map_communication.publish_map()

        # if last action was `dispense` and result = 'success' then can attach
        if self.perception_provider.agent.last_action == "request" and self.perception_provider.agent.last_action_result == "success":
            #TODO check if the subtask block type is the same of the block just dispensed
            self.assigned_subtasks[0].is_dispensed = True

        # if last action was `connect` and result = 'success' then save the attached block
        if self.perception_provider.agent.last_action == "connect" and self.perception_provider.agent.last_action_result == "success":
            other_agent_name = self.perception_provider.agent.last_action_params[0]
            #new
            task_name = self.assigned_subtasks[0].parent_task_name
            current_task = self.tasks.get(task_name, None)
            # For submitting agent is_connected only when all the others blocks are connected
            if self.assigned_subtasks[0].submit_behaviour:
                is_connected_counter = 0
                number_of_subtasks = len(current_task.sub_tasks)
                for sub_task in current_task.sub_tasks:
                    # Check for other agents
                    if sub_task.assigned_agent != self._agent_name:
                        if sub_task.is_connected:
                            is_connected_counter += 1
                # Check for all the other subtaks connected
                if is_connected_counter == number_of_subtasks - 1:
                    self.assigned_subtasks[0].is_connected = True
            else:
                self.assigned_subtasks[0].is_connected = True
            # make the other guy's subtask completed and connected
            # task_name = self.assigned_subtasks[0].parent_task_name
            # current_task = self.tasks.get(task_name, None)
            for sub_task in current_task.sub_tasks:
                # TODO check which is the completed sub_task if an agent can have more sub_tasks
                # TODO because there is no communication, other agents updating your is_connected it shouldn't affect
                if sub_task.assigned_agent == other_agent_name:
                    self.local_map._attached_blocks.append(Block(block_type=sub_task.type,
                                                                 position=sub_task.position))
                    sub_task.is_connected = True
                    sub_task.complete = True

        # if last action was detach, detach the blocks
        if self.perception_provider.agent.last_action == "detach" and \
                self.perception_provider.agent.last_action_result == "success":
            # TODO detach only the block in the direction of the detach
            self.local_map._attached_blocks = []
            for assigned_subtask in self.assigned_subtasks:
                if assigned_subtask.is_connected == True: # DO NOT KNOW WHY THE PREVIOUS SUBTASK WAS DELETED
                    assigned_subtask.is_complete = True
                    self.assigned_subtasks.remove(assigned_subtask)


        # if last action was submit, detach the blocks
        if self.perception_provider.agent.last_action == "submit" and \
                self.perception_provider.agent.last_action_result == "success":
            # TODO detach only the block in the direction of the task
            self.local_map._attached_blocks = []
            # this shouldn't be necessary because the task is not in the percept,
            # therefore the subtask is removed
            # self.assigned_subtasks.pop()

        '''
        # send personal message test
        if self._agent_name == "agentA1":
            self._communication.send_message(self._pub_agents, "agentA2", "task", "[5,5]")

        '''

        # update the sensors before starting the rhbp reasoning
        self.sensor_manager.update_sensors()

        # dumped class
        if global_variables.DUMP_CLASS:
            test_case_number = 'fixed'
            file_name = 'task_assignment_for_' + str(self.number_of_agents) + '_' + test_case_number
            file_object = open("/home/alvaro/Desktop/AAIP/mapc_workspace/src/group5/strategy_1/src/" \
                               + file_name + '.dat', "wb")
            pickle.dump(self.tasks, file_object)
            file_object.close()

        self.start_rhbp_reasoning(start_time, deadline)



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

        # has already dispensed the block, temporary solution for lack of communication
        attach.add_precondition(
            Condition(sensor=self.sensor_manager.is_dispensed, activator=BooleanActivator(desiredValue=True)))

        # is next to a block
        attach.add_precondition(
            Condition(sensor=self.sensor_manager.next_to_block, activator=BooleanActivator(desiredValue=True)))
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
