from classes.map_merge import mapMerge
import numpy as np
import rospy


class MapCommunication:

    map_messages_buffer = []

    def __init__(self,rhbp_agent_istance):
        self.agent = rhbp_agent_istance
        self._pub_map = self.agent._communication.start_map(self._callback_map)

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

            if map_from != self.agent._agent_name and self.agent.local_map.goal_area_fully_discovered:
                # map received
                maps = np.fromstring(map_value, dtype=int).reshape(map_rows, map_columns)
                rospy.logdebug(maps)
                map_received = np.copy(maps)
                # landmark received
                lm_received = np.array([map_lm_y, map_lm_x])
                # own landmark
                lm_own = self.agent.local_map._from_relative_to_matrix(self.agent.local_map.goal_top_left)
                # do map merge
                # Added new origin to function
                origin_own = np.copy(self.agent.local_map.origin)
                merged_map, merged_origin = mapMerge(map_received, self.agent.local_map._representation, lm_received, lm_own,
                                                     origin_own)
                self.agent.local_map._representation = np.copy(merged_map)
                self.agent.local_map.origin = np.copy(merged_origin)

            self.map_messages_buffer.remove(msg)

    def publish_map(self):
        map = self.agent.local_map._representation
        top_left_corner = self.agent.local_map._from_relative_to_matrix(self.agent.local_map.goal_top_left)
        self.agent._communication.send_map(self._pub_map, map.tostring(), top_left_corner[0], top_left_corner[1],
                                     map.shape[0], map.shape[1])  # lm_x and lm_y to get

    def _callback_map(self, msg):
        self.map_messages_buffer.append(msg)