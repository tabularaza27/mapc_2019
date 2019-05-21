# mapc_rhbp_example

This is a very basic example that shows how RHBP can be used within the Multiagent Programming Contest Scenario of 2019.

The implemented agents are exploring there environment by randomly moving around. Once they are able to perceive a dispenser 
in their vision range they approach it and start dispensing.


## Execution

The example can be executed with `roslaunch mapc_rhbp_example rhbp_agents_example.launch`
The configuration in above launch file is made for 10 agent scenario of default team A executed on localhost.

## Exercises

Possible exercises to get used to the frameworks:

* Only create a particular number of blocks (per type).
* Implement a more systematic exploration.
* Search for particular dispenser types.
* Implement a more sophisticated path planning once a dispenser is perceivable e.g. going around other entities.


