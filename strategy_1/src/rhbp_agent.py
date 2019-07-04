#!/usr/bin/env python2
import time
import numpy as np
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
from agent_commons.providers import PerceptionProvider
from agent_commons.agent_utils import get_bridge_topic_prefix
from agent_commons.sensor_manager import SensorManager

from classes.grid_map import GridMap
from classes.tasks.task_decomposition import update_tasks
from classes.communications import Communication
from classes.map_merge import mapMerge


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

        self._agent_name = rospy.get_param('~agent_name', 'agentA1')  # default for debugging 'agentA1'

        self._agent_topic_prefix = get_bridge_topic_prefix(agent_name=self._agent_name)

        # ensure also max_parallel_behaviours during debugging
        self._manager = Manager(prefix=self._agent_name, max_parallel_behaviours=1)

        self.behaviours = []
        self.goals = []

        self.perception_provider = PerceptionProvider()

        # auction structure
        self.bids = {}
        self.number_of_agents = 1   # TODO: check if there's a way to get it automatically

        self._sim_started = False

        # agent attributes
        self.local_map = GridMap(agent_name=self._agent_name, agent_vision=5)  # TODO change to get the vision
        self.map_messages_buffer = []

        # instantiate the sensor manager passing a reference to this agent
        self.sensor_manager = SensorManager(self)

        # representation of tasks
        self.tasks = {}
        self.assigned_tasks = []  # personal for the agent. the task at index 0 is the task the agent is currently executing

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
        to_delete_tasks = []
        for task_name, task_object in self.tasks.iteritems():

            # TODO: possible optimization to free memory -> while we cycle all the tasks, check for if complete and if yes remove from the task list?
            if len(task_object.sub_tasks) <= self.number_of_agents:
                rospy.loginfo("-- Analyizing: " + task_name)
                assigned = []
                # STEP 1: WAIT FOR BID OF ALL SUBTASKS
                for sub in task_object.sub_tasks:
                    # TODO DO IT EVERY_TIME FOR ROBUSTNESS OR CHECK IF ALL THE OTHERS AGREED
                    if sub.assigned_agent == None:
                        subtask_id = sub.sub_task_name
                        rospy.loginfo("---- Bid needed for " + subtask_id)

                        # check if the agent is already assigned to some subtasks of the same parent
                        if self._agent_name in assigned:
                            bid_value = -1
                        else:
                            # first calculate the already assigned sub tasks
                            bid_value = 0
                            for t in self.assigned_tasks:
                                bid_value += self.calculate_subtask_bid(t)[0]

                            # add the current
                            current = self.calculate_subtask_bid(sub)
                            bid_value += current[0]
                            distance_to_dispenser = current[1]
                            closest_dispenser_position = self.local_map._from_relative_to_matrix(current[2])

                        self._communication.send_bid(self._pub_auction, subtask_id, bid_value, distance_to_dispenser, closest_dispenser_position[0], closest_dispenser_position[1])

                        # wait until the bid is done

                        while subtask_id not in self.bids:
                            pass
                        # TODO AGENTS GET STUCK IN THIS WHILE
                        # ???
                        current_time = 0
                        deadline = 0.3
                        while self.bids[subtask_id]["done"] == None:
                        # while self.bids[subtask_id]["done"] == None and current_time < deadline:
                            time.sleep(0.05)
                            current_time += 0.05

                        if self.bids[subtask_id]["done"] != "invalid":  # was a valid one
                            rospy.loginfo(
                                "------ DONE: " + str(self.bids[subtask_id]["done"]))
                            sub.assigned_agent = self.bids[subtask_id]["done"]
                            sub.distance_to_dispenser = self.bids[subtask_id]["distance_to_dispenser"]
                            sub.closest_dispenser_position = self.bids[subtask_id]["closest_dispenser_position"]

                            assigned.append(sub.assigned_agent)

                            if sub.assigned_agent == self._agent_name:
                                self.assigned_tasks.append(sub)
                        else:
                            rospy.loginfo(
                                "------ INVALID: " + str(self.bids[subtask_id]["done"]) + " with bid value: " + str(
                                    bid_value))

                        del self.bids[sub.sub_task_name]  # free memory

                # STEP 2: ELIMINATE NOT FULLY AUCTIONED TASKS
                fully_auctioned = task_object.check_auctioning()
                if not fully_auctioned:
                    rospy.loginfo("--------- NEED TO REMOVE: " + str(task_object.auctioned))
                    # delete the task
                    to_delete_tasks.append(task_name)

                    # delete all the subtasks assigned
                    for sub in task_object.sub_tasks:
                        if sub in self.assigned_tasks:
                            self.assigned_tasks.remove(sub)

        for task_name in to_delete_tasks:
            del self.tasks[task_name]

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
                lm_received = (map_lm_y, map_lm_x)
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
        pos = -1
        min_dist = -1

        if self.local_map.goal_area_fully_discovered:
            required_type = subtask.type

            # find the closest dispenser
            pos, min_dist = self.local_map.get_closest_dispenser_position(required_type)

            if pos is not None:  # the distance to the closer dispenser has been calculated
                # add the distance to the goal
                meeting_point = self.local_map.goal_top_left # TODO change the meeting point with communication
                end = np.array([meeting_point[0], meeting_point[1]], dtype=int)
                distance, path = self.local_map.get_distance_and_path(pos, end, return_path=True)

                bid_value = distance + min_dist  # distance from agent to dispenser + dispenser to goal

                # TODO save task parameters dinamically every step to set sensors
                subtask.set_closest_dispenser_position(pos)
                subtask.set_meeting_point(meeting_point)
                path_id = self.local_map._save_path(path)
                subtask.set_path_to_dispenser_id(path_id)

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
        rospy.loginfo("SimEnd:" + str(msg))
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
        rospy.loginfo("Simulation finished")
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

        # update map
        #self.local_map.update_map(agent=msg.agent, perception=self.perception_provider)
        # best_point, best_path, current_high_score = self.local_map.get_point_to_explore()
        # rospy.logdebug("Best point: " + str(best_point))
        # rospy.logdebug("Best path: " + str(best_path))
        # rospy.logdebug("Current high score: " + str(current_high_score))


        # update tasks from perception
        self.tasks = update_tasks(current_tasks=self.tasks, tasks_percept=self.perception_provider.tasks,
                                  simulation_step=self.perception_provider.simulation_step)
        rospy.loginfo("{} updated tasks. New amount of tasks: {}".format(self._agent_name, len(self.tasks)))

        # task auctioning
        self.task_auctioning()


        

        # map merging
        self.map_merge()
        self.local_map.update_map(agent=msg.agent, perception=self.perception_provider)
        self.local_map._update_distances()

        # send the map if perceive the goal
        if self.local_map.goal_area_fully_discovered:
            self.publish_map()

        

        # test of task update
        #self._communication.send_subtask_update(self._pub_subtask_update,"done","task0_-1_0")

        '''
        # send personal message test
        if self._agent_name == "agentA1":
            self._communication.send_message(self._pub_agents, "agentA2", "task", "[5,5]")

        '''

        # update the sensors before starting the rhbp reasoning
        self.sensor_manager.update_sensors()

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

        # 1 BIDDING
        if task_id not in self.bids:
            self.bids[task_id] = OrderedDict()
            self.bids[task_id]["done"] = None
            self.bids[task_id]["distance_to_dispenser"] = None
            self.bids[task_id]["closest_dispenser_position"] = None

        if self.bids[task_id]["done"] is None:
            if msg_from not in self.bids[task_id]:
                self.bids[task_id][msg_from] = task_bid_value

            if len(self.bids[task_id]) == self.number_of_agents + 3:  # count the done, distance_to_dispenser, closeset_dispenser_position
                # order the dictionary first for value and than for key
                self.bids[task_id]
                ordered_task = OrderedDict(sorted(self.bids[task_id].items(), key=lambda x: (x[1], x[0])))

                '''

                This in case we want to extend it to the possibility of more than one agent assigned to a sub task
                duplicate = -999
                i = 0
                for key, value in ordered_task.items():
                    if (i > 0):  # skip done
                        if (i == self.task_subdivision[task_id]["agents_needed"] + 1):
                            break

                        available = (len(ordered_task) - 1) - len(self.task_subdivision[task_id]["agents_assigned"]) - i
                        # rospy.loginfo(self._agent_name + " |1: " + str(len(ordered_task) - 1) + " | 2: " + str(len(self.task_subdivision[task_id]["agents_assigned"])) + "i: " + str(i) + " | current:" + key)
                        if (value != duplicate or available <= 0):
                            self.task_subdivision[task_id]["agents_assigned"].append(key)

                        duplicate = value

                    i += 1
                '''


                for key, value in ordered_task.items():
                    if key != 'done' and key != 'distance_to_dispenser' and key != 'closest_dispenser_position':  # skip done
                        if (value == -1):
                            self.bids[task_id]["done"] = "invalid"
                        else:
                            self.bids[task_id]["done"] = key
                            self.bids[task_id]["distance_to_dispenser"] = distance_to_dispenser
                            self.bids[task_id]["closest_dispenser_position"] = [closest_dispenser_position_x,closest_dispenser_position_y]
                            break



    def _initialize_behaviour_model(self):
        """
        This function initialises the RHBP behaviour/goal model.
        """

        # Exploration
        exploration_move = ExplorationBehaviour(name="exploration_move", agent_name=self._agent_name, rhbp_agent=self)
        self.behaviours.append(exploration_move)
        exploration_move.add_effect(Effect(self.perception_provider.dispenser_visible_sensor.name, indicator=True))
        exploration_move.add_effect(Effect(self.sensor_manager.assigned_task_list_empty.name, indicator=True))

        # Move to Dispenser
        move_to_dispenser = MoveToDispenserBehaviour(name="move_to_dispenser", agent_name=self._agent_name,rhbp_agent=self)
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

        """
        # Our simple goal is to create more and more blocks
        dispense_goal = GoalBase("dispensing", permanent=True,
                                 conditions=[
                                     Condition(self.sensor_manager.at_the_dispenser, GreedyActivator())],
                                 planner_prefix=self._agent_name)
        self.goals.append(dispense_goal)
        """

        # Requeste  block - Dispense
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
        dispense.add_effect(Effect(self.sensor_manager.attached_to_block.name, indicator=True))
        
        # Our simple goal is to create more and more blocks
        dispense_goal = GoalBase("dispensing", permanent=True,
                                 conditions=[
                                     Condition(self.sensor_manager.attached_to_block, GreedyActivator())],
                                 planner_prefix=self._agent_name)
        self.goals.append(dispense_goal)

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


if __name__ == '__main__':
    try:
        rhbp_agent = RhbpAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
