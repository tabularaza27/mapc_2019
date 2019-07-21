# Group5 Implementation for MAPC 2019

This code is adapted from the mapc_rhbp_example package from Christopher

>This is a very basic example that shows how RHBP can be used within the Multiagent Programming Contest Scenario of 2019.
>The implemented agents are exploring there environment by randomly moving around. Once they are able to perceive a dispenser 
>in their vision range they approach it and start dispensing.

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

## Code Structure
In our Repo with have subfolders for different versions of the implementations, so we can later test different versions against each other

* *manual_player_package* (is used by *mapc_rhbp_manual_player*)
* *mapc_rhbp_manual_player* (agents subscribe to the ros topic created by the manual_player_package to control the agents with the keyboard) --> this implementation is not very clean yet
* *strategy_1* for now it is just a copy of the example

### GIT Workflow

* Use of IDE (e.g. PyCharm) is recommended, it makes handling the remotes of the different repos and submodules very easy
* Check out this [Guide](https://nvie.com/posts/a-successful-git-branching-model/) for GIT Branching workflow 

## Execution
*Attention*: Don't forget to run ```catkin_make``` after cloning our repo

See the Readme.md files of the corresponding packages for instructions on execution

## Exercises

Possible exercises to get used to the frameworks:

* Only create a particular number of blocks (per type).
* Implement a more systematic exploration.
* Search for particular dispenser types.
* Implement a more sophisticated path planning once a dispenser is perceivable e.g. going around other entities.

## Code Style and Documentation
* Have a look at the [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html), especially chapter 3.8 on Comments and Docstrings
* [Example of Docstrings](https://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_google.html)
