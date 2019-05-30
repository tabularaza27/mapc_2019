from __future__ import division  # force floating point division when using plain /
import rospy
import random

from behaviour_components.behaviours import BehaviourBase

from diagnostic_msgs.msg import KeyValue
from mapc_ros_bridge.msg import GenericAction

from agent_common.agent_utils import get_bridge_topic_prefix, pos_to_direction

from agent_common.providers import PerceptionProvider

from std_msgs.msg import String
from geometry_msgs.msg import Twist

direzione = 'n'
action = "move"

def action_generic_simple(publisher, action_type, params=[]):
    """
    Generic helper function for publishing GenericAction msg
    :param publisher: publisher to use
    :param action_type: the type of the action msg
    :param params: optional parameter for the msg
    """
    action = GenericAction()
    action.action_type = action_type
    action.params = params
    publisher.publish(action)

def callback_direzioni(data):
    global direzione
    global action

    if (data.data == "n" or data.data == "s" or data.data == "e" or data.data == "w"):
        direzione = data.data
        action = "move"
        rospy.loginfo("MOVE | new direction: " + direzione)
    elif (data.data == "d"):
        action = "dispense"
        rospy.loginfo("DISPENSO DIO")
    elif (data.data == "a"):
        action = "attach"
        rospy.loginfo("ATTACH GOD")
    elif (data.data == "z"):
        action = "detach"
        rospy.loginfo("detaching")
    elif (data.data == "s"):
        action = "submit"
        rospy.loginfo("submitting")
    elif (data.data == "l"):
        action = "rotate_left"
        rospy.loginfo("rotating_left")
    elif (data.data == "r"):
        action = "rotate_right"
        rospy.loginfo("rotating_right")
    elif (data.data == "c"):
        action = "connect"
        rospy.loginfo("connecting")

class GenericActionBehaviour(BehaviourBase):
    """
    A simple behaviour for triggering generic MAPC actions that just need a action type and static parameters
    """

    def __init__(self, name, agent_name, action_type, params=[], **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param action_type: type of the MAPC action
        :param params: optional static parameters for the MAPC action
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(GenericActionBehaviour, self) \
            .__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

        self._agent_name = agent_name

        self._action_type = action_type
        self._params = params
        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    def do_step(self):
        rospy.loginfo(self._agent_name + "::" + self._name + " executing: " + self._action_type)
        action_generic_simple(publisher=self._pub_generic_action, action_type=self._action_type, params=self._params)

class ManualMove(BehaviourBase):
    """
    Move in randomly chosen directions
    """

    def __init__(self, name, perception_provider, agent_name, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(ManualMove, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

        self._perception_provider = perception_provider

        self._agent_name = agent_name

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        #rospy.Subscriber("/directions", String, callback_direzioni)
        rospy.Subscriber("/key_player", String, callback_direzioni)

    def do_step(self):
        global action
        if (action == "move"):
            params = [KeyValue(key="direction", value=direzione)]
            rospy.loginfo(self._agent_name + "::" + self._name + " executing move to " + str(params) + "\n" + 
                        "Current perception:\n" +
                        "\tDispensers: " + str(self._perception_provider.dispensers)+"\n"+
                        "\tObstacles: " + str(self._perception_provider.obstacles)+"\n"+
                        "\tGoals: " + str(self._perception_provider.goals))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE, params=params)
            action = "wait"
        elif (action == "dispense"):
            #random_move = ['n', 's', 'e', 'w']
            donde = 'n'
            params = [KeyValue(key="direction", value=donde)]

            rospy.loginfo(self._agent_name + "::" + self._name + " request dispense " + str(params) + "\n" + 
                        "Current perception:\n" +
                        "\tDispensers: " + str(self._perception_provider.dispensers)+"\n"+
                        "\tObstacles: " + str(self._perception_provider.obstacles)+"\n"+
                        "\tGoals: " + str(self._perception_provider.goals))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_REQUEST,
                                  params=params)

            action = "wait"
        elif (action == "attach"):
            #random_move = ['n', 's', 'e', 'w']
            donde = 'n'
            params = [KeyValue(key="direction", value=donde)]

            rospy.loginfo(self._agent_name + "::" + self._name + " request attach " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_ATTACH,
                                  params=params)

            action = "wait"
        elif (action == "connect"):
            #random_move = ['n', 's', 'e', 'w']
            donde = 'TO DOOOOOOO'
            params = [KeyValue(key="direction", value=donde)]

            rospy.loginfo(self._agent_name + "::" + self._name + " request connect " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_CONNECT,
                                  params=params)

            action = "wait"
        elif (action == "detach"):
            #random_move = ['n', 's', 'e', 'w']
            donde = 'n'
            params = [KeyValue(key="direction", value=donde)]

            rospy.loginfo(self._agent_name + "::" + self._name + " request detach " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_DETACH,
                                  params=params)

            action = "wait"
        elif (action == "submit"):
            #random_move = ['n', 's', 'e', 'w']
            donde = 1
            params = [KeyValue(key="task", value=donde)]

            rospy.loginfo(self._agent_name + "::" + self._name + " request submit " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_SUBMIT,
                                  params=params)

            action = "wait"
        elif (action == "rotate_left"):
            #random_move = ['n', 's', 'e', 'w']
            donde = 'ccw'
            params = [KeyValue(key="direction", value=donde)]

            rospy.loginfo(self._agent_name + "::" + self._name + " request rotate_left " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_ROTATE,
                                  params=params)

            action = "wait"
        elif (action == "rotate_right"):
            #random_move = ['n', 's', 'e', 'w']
            donde = 'cw'
            params = [KeyValue(key="direction", value=donde)]

            rospy.loginfo(self._agent_name + "::" + self._name + " request rotate_right " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_ROTATE,
                                  params=params)

            action = "wait"


'''
class RandomMove(BehaviourBase):
    """
    Move in randomly chosen directions
    """

    def __init__(self, name, agent_name, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(RandomMove, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

        self._agent_name = agent_name

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        rospy.Subscriber("/lepredidio", String, callback_direzioni)

    def do_step(self):
        params = [KeyValue(key="direction", value=direzione)]
        rospy.loginfo(self._agent_name + "::" + self._name + " executing move to " + str(params))
        action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE, params=params)

class Dispense(BehaviourBase):
    """
    Dispenses a new block from the dispenser nearby
    """

    def __init__(self, name, agent_name, perception_provider, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param perception_provider: the current perception
        :type perception_provider: PerceptionProvider
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(Dispense, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

        self._agent_name = agent_name

        self._perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    def do_step(self):

        if self._perception_provider.closest_dispenser:

            direction = pos_to_direction(self._perception_provider.closest_dispenser.pos)
            # random_move = ['n', 's', 'e', 'w']
            # direction = random.choice(random_move)
            params = [KeyValue(key="direction", value=direction)]

            rospy.loginfo(self._agent_name + "::" + self._name + " request dispense " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_REQUEST,
                                  params=params)

        else:
            rospy.logerr("Behaviour:%s: no dispenser in range", self.name)


class MoveToDispenser(BehaviourBase):
    """
    Move to closest dispenser nearby
    """

    def __init__(self, name, agent_name, perception_provider, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param perception_provider: the current perception
        :type perception_provider: PerceptionProvider
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(MoveToDispenser, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

        self._agent_name = agent_name

        self._perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    def do_step(self):

        if self._perception_provider.closest_dispenser:

            direction = pos_to_direction(self._perception_provider.closest_dispenser.pos)

            params = [KeyValue(key="direction", value=direction)]
            rospy.loginfo(self._agent_name + "::" + self._name + " request dispense " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE,
                                  params=params)

        else:
            rospy.logerr("Behaviour:%s: no dispenser in range", self.name)
'''
