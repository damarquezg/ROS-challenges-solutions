# Hackathon Challenge

We'd like to get a feeling for your ROS, programing, robotics and problem solving 
skills, but do so in a setting that makes you comfortable and provides the opportunity 
for you to do any necessary research.

This README is contained within a single ROS1 package.
Create a new workspace, place this package within it, satisfy its dependencies, 
and build it.

## The challenge

This challenge consists of a single node called Puppet Master. That node
connects to an action server and requests that action server move the robot to a
given destination. The action description is contained within the `puppet_master_challenge` 
package. The "robot" is simulated using Turtlesim, and both Turtlesim and Puppet 
Master are properly configured using the `run.launch` file in the `puppet_master_challenge` 
package.

Your job is to create a new package containing a node that moves the Turtlesim
robot to the proper destination. The Puppet Master will put your node through
its paces using a number of different destinations.

The deliverable for this challenge is a package, which should at a minimum contain 
the code for your node as well as a launch file that runs the challenge (including 
Puppet Master and Turtlesim). 

## Rules

1. You're welcome to read this package to your heart's content, but modifying
   any part of it is not an acceptable solution to the challenge. If you have issues
   with the package, please reach out to your point of contact at ORI.
2. Your solution will be evaluated using ROS Noetic, so you might want to develop in 
   that environment.
3. Your solution can be written in C++ or Python.
4. You may not use any of turtlesim's services (only its topics).


## Getting results

After a successful run, Puppet Master's output will look like this:

    [ INFO] [1551731748.111257879]: Sending goal (0.000000, 5.000000)
    [ INFO] [1551731750.413010494]: Goal reached
    [ INFO] [1551731750.511485080]: Sending goal (4.000000, 2.000000)
    [ INFO] [1551731752.712767517]: Goal reached
    [ INFO] [1551731752.811583177]: Sending goal (9.000000, 1.000000)
    [ INFO] [1551731754.813269547]: Goal reached
    [ INFO] [1551731754.911743025]: Sending goal (2.000000, 8.000000)
    [ INFO] [1551731757.213968323]: Goal reached
    [ INFO] [1551731757.311347601]: Sending goal (8.000000, 8.000000)
    [ INFO] [1551731759.613065134]: Goal reached
    [ INFO] [1551731759.711099466]: Success: all destinations complete!

Failure is defined to be anything that doesn't result in that "Success"
message. It may be a segfault. It may simply never progress. It may look like
this:

    [ INFO] [1551733774.710989357]: Sending goal (0.000000, 5.000000)
    [FATAL] [1551733775.912321155]: Too far from the goal!

Code quality and programing best practices are also important.

An example result can be seen here:

https://youtu.be/F2pNoa-9_Lo

