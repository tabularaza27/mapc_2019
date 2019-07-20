""" This module contains the class that manages the map merge between the agents """

from classes.mapping.map_merge import mapMerge
import numpy as np
import rospy


class MapCommunication:
    """ Once the goal area is discovered, the map is sent to a shared ros topic. The maps are saved in a buffer when received.
    As first step the agents will empty the map buffer and merge the maps with their personal one.
    """

    map_messages_buffer = []

    def __init__(self,rhbp_agent_istance):
        self.agent = rhbp_agent_istance
        self._pub_map = self.agent._communication.start_map(self._callback_map)

    def map_merge(self):
        """ Merges the maps received from the other agents that discovered the goal area and this agent did too

        Returns: void
        """

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
                origin_own = np.copy(self.agent.local_map.origin)
                merged_map, merged_origin = mapMerge(map_received, self.agent.local_map._representation, lm_received, lm_own,
                                                     origin_own)
                self.agent.local_map._representation = np.copy(merged_map)
                self.agent.local_map.origin = np.copy(merged_origin)

            # remove map from the buffer as it has been processed
            self.map_messages_buffer.remove(msg)

    def publish_map(self):
        """ Send the map to the shared ros topic

        Returns: void
        """

        map = self.agent.local_map._representation
        top_left_corner = self.agent.local_map._from_relative_to_matrix(self.agent.local_map.goal_top_left) # top left corner of the goal area is used as common landmark
        self.agent._communication.send_map(self._pub_map, map.tostring(), top_left_corner[0], top_left_corner[1],
                                     map.shape[0], map.shape[1])  

    def _callback_map(self, msg):
        """ Add the received maps in the buffer

        Returns: void
        """

        self.map_messages_buffer.append(msg)