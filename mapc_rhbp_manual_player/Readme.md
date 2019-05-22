# mapc_rhbp_manual_player

This is a implementation adapted from the mapc_rhbp_example package. It implements a manual keyboard control of the agents.


## Execution

The manual player implementation can be executed with `roslaunch mapc_rhbp_manual_player rhbp_agents_manual_player.launch`
The configuration in above launch file is made for 10 agent scenario of default team A executed on localhost.

start the ROS topic for controlling the agents in a new terminal `rosrun manual_player_package key_teleop.py`

Now you can input arrow keys for controlling the agents

