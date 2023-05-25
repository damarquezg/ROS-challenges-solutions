# puppet_master_challenge solution

## puppet_master_challenge_solution package

This package consists of a single node called puppet-master-controller. That node
connects to an action client and move the robot to a given destination. 
The action client is defined in the provided `puppet_master_challenge` package.

## Dependences
`puppet_master_challenge` package

## Installation
1. Create a catkin workspace

`mkdir -p ~/catkin_ws/src`

`cd ~/catkin_ws/src`

`catkin_init_workspace # Make sure you are in the src directory`

2. navigate to the src directory and copy this package (`puppet_master_challenge_solution` package) 
and the `puppet_master_challenge` package

3. source the workspace and compile

`cd ~/catkin_ws/ # Takes you back to the catkin_ws directory`

`source devel/setup.bash`

`catkin_make`

## Launch files 
The `puppet_master_challenge_solution` package have 2 launch files:
1. `run_solution.launch`: runs the complete solution  challenge 
(Puppet Master, Turtlesim and puppet-master-controller)

2. `run_server.launch`: runs the puppet-master-controller action server

## Running the package
To launch the Puppet Master node, the TurtleSim simulation node, and the puppet-master-controller node,
run the following commands:

0. source the catkin workpsace

`source devel/setup.bash`

1. Option 1 -
In a terminal:

`roslaunch puppet_master_challenge_solution run_solution.launch`

2. Option 2 - 
In two separate terminals:

`roslaunch puppet_master_challenge run.launch`

`roslaunch puppet_master_challenge_solution run_server.launch`

3. Option 3 - 
In two separate terminals:

`roslaunch puppet_master_challenge run.launch`

`rosrun puppet_master_challenge_solution puppet_master_controller`


## Snap

A snap for the puppet_master_challenge_solution is provided
A single snapcraft config file (`snapcraft.yaml`)
is located at the root of the puppet_master_challenge_solution package folder.

This snap only execute the puppet-master-controller node - i.e. only launch
the `puppet_master_challenge_solution` launch file `run_server.launch`. 
The `puppet_master_challenge` node should be executed separately outside the snap during evaluation.
