Note, you will find a better description on the course website :)

https://canvas.kth.se/courses/56459/pages/assignment-1

The practical part of the assignment

Now we will start with the practical part of the assignment.

What we use in this course
Ubuntu 20.04
ROS 2 Foxy 
Python 3.8

Everything is installed for you in computer labs:

Ubuntu computer labs found here. Check the maps for listings of installed OS.
Install, source, and create ROS workspace
Own computer
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

The last line fixes an OpenGL problem that students with an old account would otherwise encounter. It is fine if it gives an error and says that the file or folder does not exist.
For all

Environment setup


echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc

Create a ROS workspace:

mkdir -p ~/ros_ws/src
cd ~/ros_ws/
colcon build
source install/setup.bash
Recommended editor

I recommend that you use VS Code with the extensions:

ROS

Python

Other that you find useful...

You can of course just whichever editor you want. Here you can find more information about different integrated development environments (IDEs) and how they work with ROS: https://wiki.ros.org/IDEs.

Tutorials
Basic ROS tutorials

In order for you to get a practical understanding of ROS you should now do the beginner level core ROS tutorials that you can find here.

We recommend you do all the Beginner and Intermediate tutorials before starting the assignment:

https://docs.ros.org/en/foxy/Tutorials.html


Additionally, during assignment 1 you will be working with a popular framework nav2 and a TurtleBot3 (mobile robot platform). To better understand how they work, we recommend you do Turtlebot3 tutorials that you can find here: 
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/


as well as look at nav2 documentation and tutorials 

https://docs.nav2.org/tutorials/index.html




Mini-project
Installation
Description

You will now do a mini-project where you should help a TurtleBot3 robot explore an unknown environment. The robot is called Burger and you can see a picture of Burger below.




Image taken from: http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#data-of-turtlebot3-burger


Project Tutorial: Building a Frontier-Based Explorer in ROS 2 (Foxy)

Welcome! In this project you’ll build a simple but real autonomous explorer that finds “frontiers” (the boundary between known and unknown space) and drives to them using Nav2. You’ll wire together publishers, subscribers, services, and action clients—core ROS 2 concepts you’ll use everywhere.

What you’ll build

FrontierService – a node that receives a map and returns a set of frontier points.

Explorer – a node that:

subscribes to the map,

calls FrontierService to get frontiers,

picks a target frontier,

sends a NavigateToPose action to Nav2,

waits until navigation finishes, then repeats.

You’ll run this alongside SLAM (e.g., slam_toolbox), Nav2, and a simulator (e.g., TurtleBot3 in Gazebo or a bag file).

Learning goals

Set up a ROS 2 workspace and packages.

Define and use a custom service (srv) across two nodes.

Use a ROS 2 action client (Nav2’s NavigateToPose).

Manage asynchronous flow correctly (timers, callbacks, non-blocking service calls).

Debug common ROS 2 issues (QoS, TF, costmaps, lifecycle).

The ability to perform autonomous exploration is essential for an autonomous system operating in unstructured or unknown environments where it is hard or even impossible to describe the environment beforehand.

System description
Project Overview (Nav2 + SLAM Toolbox)

Good news: you don’t have to implement the whole exploration pipeline from scratch. You’ll build a lightweight Explorer node that decides where to go next, and you’ll hand off the how to get there safely to the Nav2 stack.

We’ll run everything in simulation with Gazebo (TurtleBot3), and use SLAM Toolbox for real-time mapping + localization.

What components are we using?

SLAM Toolbox
Provides on-line SLAM, publishes the occupancy grid on /map, and maintains the map → odom transform. This replaces any Hector SLAM mention.

Nav2 (Navigation 2) Stack
Handles global path planning, local control, obstacle avoidance, and recovery behaviors. Concretely:

Planner Server (e.g., NavFn or Smac Planner) computes paths.

Controller Server (e.g., Regulated Pure Pursuit) follows paths and performs local collision avoidance.

Costmaps (global + local) are owned by Nav2; no separate costmap_2d node is launched manually.

BT Navigator coordinates the behavior tree for NavigateToPose.

Lifecycle Manager brings Nav2 nodes to the active state.

robot_state_publisher
Publishes the robot’s kinematic transforms (URDF → TF), so everyone agrees on frames: map → odom → base_link.

Gazebo
The simulator environment and robot model.

What do you build?

FrontierService (provided or implemented by you):
A simple ROS 2 service that takes the current nav_msgs/OccupancyGrid and returns frontier points (cells on the boundary between known and unknown space).

Explorer (your node):

Subscribes to /map.

Get candidate frontier points.

Chooses a goal (e.g., the closest unvisited frontier).

Converts grid indices → world coordinates.

Sends a NavigateToPose action goal to Nav2.

Waits until Nav2 reports completion (or failure), then repeats

In other words, you decide the next best place to explore; Nav2 plans and drives the robot there safely using its planners, controllers, and costmaps.

Let's start

We strongly recommend beginning this section only after completing tutorials 1 through 10,  as well as the suggested TF tutorials, to ensure a better understanding.

Note, you may need to change some paths in the launch/python files and/or install some python packages.

You need to change two things before you start (i.e. two paths).

1. In src/irob_assignment_1/config/navigation.yaml line 59

2. set "pkg_irob_assignment_1" in src/irob_assignment_1/launch/simulator.launch.py and src/irob_assignment_1/launch/simulator.launch.py to your path.




Open three terminals.

In each of them you need to source your environment setup.bash as well as your general setup.bash from /opt... file and set the robot model `export TURTLEBOT3_MODEL=burger`

1. Terminal 1 

ros2 launch irob_assignment_1 simulator.launch.py

2. Terminal 2 

ros2 launch irob_assignment_1 start_navigation.launch.py

3. Terminal 3

Terminal 3 start the assignemnt, until you run it you will see a bunch of warnings and errors. Note, the lunch file starts the explorer that you need to implement (that's your task for this assignment) :) 

ros2 launch irob_assignment_1 start.launch.py

You will see a window called RViz and Gazebo open:




After you run the third script you shoud be able to see the map and the cost map 







NOTE: you can disable Gazebo if you are running out of memory/your running too slow. 







In the main view of the RViz window you can see a small Turtlebot3 Burger robot in the middle of the white area. The white area of the map is called free space, it is space where the robot knows there is nothing. The large gray area is unknown space, it is space that the robot knowns nothing about. It can be either free space or occupied space. Occupied space is the cerise colored space. The cyan colored space is called C-space, it is space that are a distance from the occupied space such that the robot would collied with the occupied space if it would move into it. Take a look at the image below if you are interested. You can read more about it here. Your job will be to help Burger explore as much of the unknown space as possible.




If you open up RQT:

rqt

Then in the topbar select Plugins->Introspection->Node Graph and uncheck Leaf topics, you will see which nodes publishing and subscribe to (hard to see on this screenshot but you need to do it yourself). 




So now the moment you've been waiting for:
Open explorer.py and start coding :) 

Here is a short description of what you need to implement (hopefully you've already get a sense of it from the description above) but one recommendation before you start (and haven't done TurtleBot3 tutorials) is to do this one at least https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/.

In this simple tutorial you explore the map by controlling the robot yourself. What you need to implement in this assignemnt is very similar, but now the robot has to move autonomously !  

Ok, now let's jump to the description:

As part of the assignment, you are required to implement the core exploration behavior of a robot using the ROS 2 framework. This process begins with creating a subscriber to the costmap topic—typically `/map`—which publishes `OccupancyGrid` messages representing the robot's current understanding of its environment. This subscriber must be initialized within the node's constructor and configured to store the incoming map data for further processing.

When the last launch file runs, the `ExplorerNode` is initialized. This setup phase includes not only subscribing to the `/map` topic, but also establishing an action client for the `NavigateToPose` action, which will be used to send navigation goals. A timer is also set up to trigger the robot’s core exploration logic at fixed intervals.

Once a map message is received, the `map_callback()` function is invoked, storing the occupancy grid for use during frontier detection. Every few seconds—typically 5—the `explore()` method is called. This function acts as the core loop of the exploration behavior. It begins by checking if the map data is available; if it is, the map is converted into a 2D NumPy array for efficient processing.

Students must then implement the `find_frontiers()` function, which analyzes the grid and identifies frontiers—cells marked as free (value `0`) that border unknown cells (value `-1`). These frontiers represent unexplored areas and form the next potential goals for the robot.

Once a list of frontiers has been detected, the `choose_frontier()` function selects the most appropriate one to explore next. This typically involves finding the frontier closest to the robot's current position—information that students must update from a localization source such as `/amcl_pose`. The function also filters out any frontiers that were previously visited.

After selecting a suitable frontier, students must convert its grid coordinates into real-world positions using the map's resolution and origin. These coordinates are then used to construct a `PoseStamped` message, representing the target pose in the map frame.

The navigation goal is then sent using the `navigate_to()` method. This method packages the target pose into a `NavigateToPose.Goal` object and sends it to the Nav2 action server. It also attaches the `goal_response_callback()` to monitor whether the goal is accepted or rejected by the server.

If the goal is accepted, the `goal_response_callback()` sets up a final callback—`navigation_complete_callback()`—which waits for the navigation to finish. This callback logs the result once the robot reaches the goal or if the navigation fails.

In this way, the robot incrementally explores its environment by navigating from one frontier to the next. After each navigation attempt, the loop continues: a new map is received, new frontiers are found, and a new goal is selected. This process continues until no more valid frontiers are found, indicating that the entire environment has been explored.


The presentation

The important part of this assignment is not that you write superb code. What we want you to focus on is to understand ROS.

You should be able to explain the different ROS concepts, nodes, topics, services, messages, actionlib, ect. You should be able to explain all lines in your code: why do you need a rospy.init_node(...)? What does pub = rospy.Publisher(...) and pub.publish(...) do? What is the queue_size parameter used for when you are calling rospy.Publisher(...)?

(Will be) Frequently Asked Questions
Which TF functions should I use to complete the mini-project?

I did it with tf_buffer.lookup_transform(...) and tf2_geometry_msgs.do_transform_point(...).




lookup_transform is not working, why?

It is probably because you initialized the TF buffer (and listener) just before you called tf_buffer.lookup_transform(...). You have to initialize them early so that there is time to fill up the buffer with transforms.




My robot moves weird in the mini-project, why?

Did you set a maximum linear and angular velocity? 0.5 and 1.0, respectively, should work. Also, consider setting the linear velocity to 0 if the angular velocity is high (close to your maximum).




The TF says something about extrapolating into the past, why?

Probably because you did not initilize the TF buffer such that it has the transforms like this:

# Imports

# Global variables

tf_buffer = None
listener = None

# Your other functions

if __name__ == "__main__":
    rospy.init_node("controller")

    # Other stuff you want to init

    # Create TF buffer
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Function calls and such

    rospy.spin()


You should do this in the beginning (right after rospy.init_node(...)) such that it has time to buffer the transforms.




The TF says something about extrapolating into the future, why?

Probably because you are trying to get a transform that is not yet available. TF2 cannot extrapolate, only interpolate.

You can use these inside tf_buffer.lookup_transform(...): rospy.Time(0) to get the most recent transform and rospy.Duration(1) to wait for 1 second if that is needed.




Why is the exploration so slow?

You probably did not use the Callback-Based SimpleActionClient. Implement it using Callback-Based SimpleActionClient instead. In the feedback callback you can see if the current gain is over, for example, 3 (or some other value that you find is good), and then cancel the goal if that is the case.




I am trying to cancel the goal but it is not working :(

Try to cancel all the goals using goal_client.cancel_all_goals().




I implemented the Callback Based SimpleActionClient, but it is not working :(

Did you actually cancel the request after you got a feedback with high enough gain? When you cancel the request you will get a result actionlib.TerminalState.PREEMPTED, so be sure that in the result callback that you only do something if the state is actionlib.TerminalState.SUCCEEDED.




I get an error when launching the simulation

If you get an error like this when starting the simulation:

[Err] [Server.cc:98] SDF is not valid, see errors above. This can lead to an unexpected behaviour.

or:




[ERROR] [1566203746.351834242, 923.389000000]: Transform failed during publishing of map_odom transform: Lookup would require extrapolation into the past. Requested time 922.885000000 but the earliest data is at time 922.951000000, when looking up transform from frame [base_footprint] to frame [odom]

Do not worry about it. It is fine. The first one will not influence the overall pipeline, while the second one is because the SLAM system is trying to use a transform that does not exist yet.




I get another error when launching the simulation

If you get an error like this when starting the simulation:

[gazebo-1] process has died [pid 6639, exit code 255, cmd /opt/ros/noetic/lib/gazebo_ros/gzserver -e ode /home/dduberg/catkin_ws/src/irob_assignment_1/worlds/office.world __name:=gazebo __log:=/home/dduberg/.ros/log/18af60ee-c01f-11e9-98f6-b06ebf6030aa/gazebo-1.log].
log file: /home/dduberg/.ros/log/19af60ee-c01f-11e9-98f6-b06ebf6030aa/gazebo-1*.log

This means that Gazebo crashed, and you have to restart the simulation. Sadly, this happens sometimes.




It takes a long time for RViz and/or Gazebo to load/start

Run these commands:

cd ~
mkdir not
mv .nv .nvidia-settings-rc not
