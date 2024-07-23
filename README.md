# Introduction
The ROS package **assignment** implement a pick and place task from scratch. It has the following requirements:
- TableA and TableB are automatically spawned (position and orientation) randomly inside the robot workspace. At the meantime, their reachability, and the minimum distance between them are guranteed.
- Pickup pose and place pose of the target cube are automatically generated based on the Table positions. The relative position between cube and Tables meets the predefined requirement.
- Approaching poses of cube are automatically generated based onnnnn pickup pose and place pose. With an offset backward on the gripper openning direction, to ease the pick&place in a real scenario.
- Cube information is published periodically in another thread, including start position expected desitation position, current position, start up-vector and current up-vector.
- Easy design to reset everything and repeat the demo periodically
- With minimum external depandency or resource (only official Doosan-robot, and a transformation file)


# Build
1. Install ROS(tested with noetic), create a ROS workspace
    
        mkdir -p catkin_ws/src

2. Clone the offcial doosan-robot in src/, and follow the steps to finish its installation

        git clone https://github.com/doosan-robotics/doosan-robot

3. Unzip the assignment.zip under src/ and rebuild ROS workspace.

        cd catkin_ws
        catkin_make

4. Read to launch some fancy stuff!


# How to run
Since the DRCF of Doosan-robot relies on docker, please install docker when following the doosan-ros setup process. For each terminal, source before typing commands

    devel/setup.bash

*Terminal1*: 

    roslaunch dsr_launcher single_robot_gazebo.launch

*Terminal2*: 

    rosrun assignment poc_pick_cube.py

Keep in mind that the docker and gazebo may take some time. If anything happens, try to rerun the commands above. To check if docker images is loaded successfully, the following command should print out dsr01_emulator.

    docker container list


# Issues
The offical doosan-ros has some problem with its robot model with grippers on it. This can be observed by the:

    roslaunch dsr_launcher single_robot_gazebo.launch gripper:=robotiq_2f

Therefore, the simulation is shown without the gripper. I assumed the TCP aligns with the wrist flange, added funtionality to update cube position while being grasped accordingly. This issue should not cause any problem except visual aspect.


# Final notes
There are different ways to achieve the assignment. A weekend is too tight to realize all the thoughts. Many other approaches can be implemented, such as using MoveIt, integrating IK soler based on constrains and optimization, applying virtual force in the servoing control for more advanced motion planning, an interactive way to relocate tables in the work scene, etc. I am looking forward to further discussions on these topics.

Since I have quite less experience on Doosan robot, than many other familiar robots (such as UR, Kinova, etc.). I would like to take the chance to learn their ros packages and technical documents. It is a challenge to acomplish all of them in just a coupe of days, but I like it.

To deliver the assignment on time, there is no extra time to debug the problems in the offical dooson-robot repository, or design the optional GUI. I'm still interested in these topics, and would go for them when I have more free time.
