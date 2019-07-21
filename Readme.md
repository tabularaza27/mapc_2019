# Group5 Implementation for MAPC 2019

This code is the final implementation of Group 5 for MAPC 19

## Setup

1. Clone mapc_workspace repo recursivley  
```git clone --recursive git@gitlab.tubit.tu-berlin.de:aaip-ss19/mapc_workspace.git```

2. Follow [setup & build procedure](https://gitlab.tubit.tu-berlin.de/aaip-ss19/mapc_workspace#clone-and-build) of mapc_workspace repo

3. Clone our repository into the ```/src``` directory  
```cd /src```  
```git clone git@gitlab.tubit.tu-berlin.de:aaip-ss19/group5.git```

4. Copy the server configuration and map contained in data/FINAL_SERVER_SETUP in the massim server configuration folder massim/server/conf/
cp group  
``` cp group5/data/FINAL_SERVER_SETUP/final_config.json ../third-party/massim/server/conf/ ```  
``` cp group5/data/FINAL_SERVER_SETUP/test.bmp ../third-party/massim/server/conf/maps/ ```

5. Run `catkin_make` in the root folder of the mapc workspace

## Start the simulation

1. Run the server:  
``` sh ../scripts/start_massim_src.sh ```   (Choose ```conf/final_config.json``` as server configuration)


2. Source the workspace:
in another terminal window go to the mapc_workspace folder and source the setup file:  
``` source devel/setup.bash ```

3. Launch just 2 agents:  
``` roslaunch strategy_1 rhbp_agents_strategy_2_agents.launch ```

The amount of agents in the simulation needs to be spceified in the ```__init__()``` (line 73) of ```src/group5/strategy_1/src/rhbp_agent.py```.
It needs to match the amount of launched agents. The reason is that during the auctioning process the agents wait for all other agents to place their bids.


## Code Structure
In our Repo with have subfolders for different versions of the implementations, so we can later test different versions against each other

* *manual_player_package* (is used by *mapc_rhbp_manual_player*)
* *mapc_rhbp_manual_player* (agents subscribe to the ros topic created by the manual_player_package to control the agents with the keyboard) --> this implementation is not very clean yet
* *strategy_1* this is the current implementation of our strategy

All classes and functionalities can be found in ```/commons```. The reason why it is outsourced, is that it can now easily be imported to different strategies.

## Map Live Plotting

For Debugging purposes current representation of the maps of each agent can be plotted (live) via
```python commons/map_live_plotting.py```

### GIT Workflow

* Use of IDE (e.g. PyCharm) is recommended, it makes handling the remotes of the different repos and submodules very easy
* Check out this [Guide](https://nvie.com/posts/a-successful-git-branching-model/) for GIT Branching workflow 


## Code Style and Documentation
* Have a look at the [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html), especially chapter 3.8 on Comments and Docstrings
* [Example of Docstrings](https://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_google.html)
