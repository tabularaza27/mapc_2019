# mapc_rhbp_manual_player

This is a implementation adapted from the mapc_rhbp_example package. It implements of our first strategy to solve the MAPC 2019.
* Right now this strategy is not defined yet.
* RIght now this is just a copy of the mapc_rhbp_example package


## Execution

`roslaunch strategy_1 rhbp_agents_strategy_1.launch`
The configuration in above launch file is made for 10 agent scenario of default team A executed on localhost.


## TODO RHBP strategy
- We need a way to synchronize agents. They must wait to decide what to do till
all the agents have updated their perception of the map and the tasks. A very 
crucial thing to implement would be to go on if the time for a certain task is 
taking too much(__timeout__). E.g. an agent is crashed and not answering, the system must be
__robust__!
- Then we need to establish different communication for different porpuses,
we may need to synchronize also this tasks:
    - communication to say: "I have the goal area in my map"
    - communication to receive the actual global map 
    - communication to share the agents new perception in the one shared map
    - FUTURE communication for map similarities to try to explore smartly
    - communication to assign the management of a task to 1 agent
    - communication to bid for the different substasks of a task
- With these information we will update the RHBP sensors
- Then we need to understand what to do with the symbolic planner, to 
give more importance to some behaviors in certain conditions to reach
long term goals faster
- Then the RHBP manager is going to run and decide what to do