""" This module contains the class that manages the auction system """

import time
import numpy as np
import random
import rospy
from collections import OrderedDict
from classes.bid import Bid


class Auction:

    """ Every subtask gets auctioned. The bid value is calculated based on the distance from the agent to the closest dispenser and the
        distance from that this dispenser to the meeting point. The bids are published in a shared ros topic, and all the agents wait for all the bids to be published.
        To recover from failure, a timeout system is implemented.

        The bids are then sorted and the subtask assigned. A task is completely assigned only if all the bids are valid.
    """

    bids = {}

    def __init__(self,rhbp_agent_istance):
        """
        Args:
            rhbp_agent_istance: an istance of the rhbp agent

        Returns: void
        """

        self.agent = rhbp_agent_istance
        self._pub_auction = self.agent._communication.start_auction(self.callback_auction)

    def task_auctioning(self):
        """ Does the task auctioning process. The whole algorithm is divided in 4 steps. 
        (1) First the bids are all calculated and sent. Then (2) the agent waits for all the bids or the timeout event.
        (3) The subtask assignment is then calculated and (4) the subtasks updated. 

        Returns: void
        """

        count = 0
        for task_name, task_object in self.agent.tasks.iteritems():
            if len(task_object.sub_tasks) <= self.agent.number_of_agents and not task_object.auctioned:
                rospy.logdebug(self.agent._agent_name + "| -- Analyizing: " + task_name)
                # STEP 1: SEND ALL THE BIDS
                for sub in task_object.sub_tasks:
                    if sub.assigned_agent == None:
                        subtask_id = sub.sub_task_name
                        rospy.logdebug(self.agent._agent_name + "| ---- Bid needed for " + subtask_id)

                        # first calculate the already assigned sub tasks
                        bid_value = 0
                        for t in self.agent.assigned_subtasks:
                            bid_value += self.calculate_subtask_bid(t)[0]

                        # add the current
                        current_bid, distance_to_dispenser, closest_dispenser_position = self.calculate_subtask_bid(sub)
                        bid_value += current_bid

                        # transform the coordinates in relative to the top_left
                        if closest_dispenser_position is not None:
                            closest_dispenser_position = self.agent.local_map._from_matrix_to_relative(closest_dispenser_position, self.agent.local_map.goal_top_left)
                        else:
                            # invalid dispenser position
                            closest_dispenser_position = np.array([-1000000, -1000000])
                        self.agent._communication.send_bid(self._pub_auction, subtask_id, bid_value, distance_to_dispenser, closest_dispenser_position[0], closest_dispenser_position[1])
                
                # STEP 2: WAIT FOR ALL THE BIDS OR A TIMEOUT

                current_time = 0
                deadline = 0.3
                while count < len(task_object.sub_tasks) * self.agent.number_of_agents and current_time < deadline:
                    count = 0
                    for key, value in self.bids.items():
                        if task_object.name in key:
                            count += len(self.bids[key])
                    
                    time.sleep(0.05)
                    current_time += 0.05
                
                # STEP 3: MANAGE ASSIGN

                ass = self.assign_subtasks(self.bids,task_object.name)

                # STEP 4: ACTUALLY ASSIGN

                for sub in  task_object.sub_tasks:
                    for ass_subtask_name, value in ass.items():
                        if ass_subtask_name == sub.sub_task_name:
                            sub.assigned_agent = ass[ass_subtask_name]["assigned_agent"]
                            sub.distance_to_dispenser = ass[ass_subtask_name]["bid"].distance_to_dispenser
                            sub.closest_dispenser_position = ass[ass_subtask_name]["bid"].closest_dispenser_position

                            if self.agent._agent_name == sub.assigned_agent:
                                self.agent.assigned_subtasks.append(sub)

                            rospy.logdebug(self.agent._agent_name + "| ---- ALLL DONE: " + sub.sub_task_name)
                            rospy.logdebug(self.agent._agent_name + "| -------- AGENT: " + sub.assigned_agent)
                            rospy.logdebug(self.agent._agent_name + "| -------- DTD: " + str(sub.distance_to_dispenser))
                            rospy.logdebug(self.agent._agent_name + "| -------- CDP: " + str(sub.closest_dispenser_position))

                            del self.bids[sub.sub_task_name] # free memory
    
    def assign_subtasks(self,bids,current_task_name):
        """ Given a list of bids and the task, assigns (if possible) the different agents to the sub tasks.

        Args:
            bids (dictionary): dictionary with all the bids
            current_task_name (string): name of the considered task name 

        Returns:
            dictionary: dictionary with the subtasks that needs to be modified with the assignment
        """
        
        current_task_name += "_" # to avoid amibiguities in the if (1)
        ret = {}
        assigned = []

        for subtask_name, value in bids.items():
            if current_task_name in subtask_name: # (1)
                ordered_subtask = OrderedDict(sorted(bids[subtask_name].items(), key=lambda x: (x[1].bid_value, x[0]))) #sort the bids for value and for agent name

                invalid = True                
                for agent_name, bid in ordered_subtask.items():

                    if bid.bid_value != -1 and agent_name not in assigned:
                        invalid = False
                        ret[subtask_name] = {}
                        ret[subtask_name]["assigned_agent"] = agent_name
                        ret[subtask_name]["bid"] = bid
                        assigned.append(agent_name)

                        break
                    
                if invalid: # if i find even just one invalid subtask then all the task can't be assigned, so return empty set
                    rospy.logdebug(self.agent._agent_name + "| *** AT THE LEAST ONE GUY INVALID")
                    ret = {}
                    return ret

        return ret # means that all the subtask were valid and the dictionary is returned

    def calculate_subtask_bid(self, subtask):
        """ Calculate bid value for a subtask based on the distance from the agent to the closest dispenser and the
        distance from that this dispenser to the meeting point 

        If the agent has not discovered the goal area, he places an invalid bid of -1

        Args:
            subtask (SubTask): subtask object

        Returns:
            int: bid value of agent for the task
        """

        bid_value = -1
        pos = None
        min_dist = -1

        if self.agent.local_map.goal_area_fully_discovered:
            required_type = subtask.type

            # find the closest dispenser
            pos, min_dist = self.agent.local_map.get_closest_dispenser_position(required_type)

            if pos is not None:  # the distance to the closer dispenser has been calculated
                # add the distance to the goal

                meeting_point = self.agent.local_map.goal_top_left  # TODO change the meeting point with communication
                end = np.array([meeting_point[0], meeting_point[1]], dtype=int)
                distance, path = self.agent.local_map.get_distance_and_path(pos, end, return_path=True)

                bid_value = distance + min_dist  # distance from agent to dispenser + dispenser to goal

                # TODO save task parameters dinamically every step to set sensors
                # TODO uncomment this line and pass the position in coordinates relative to the top left of the goal area
                #subtask.closest_dispenser_position = pos
                subtask.meeting_point = end
                path_id = self.agent.local_map._save_path(path)
                subtask.path_to_dispenser_id = path_id


        return bid_value, min_dist, pos

    def callback_auction(self, msg):
        """ Gets the bid message from the agents and appends it to the dictionary representation

        Args:
            msg (auction_communication): the bid message

        Returns: void
        """

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