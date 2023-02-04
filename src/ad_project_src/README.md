How to run
Kuruppu Arachchige Sasanka 2021227162

1. Run 'roscore'
2. cd to Copellia sim root directory './coppeliaSim.sh'
3. Open scene in vrepscene/dronet_1.ttt
4. 'cd catkin_ws/src' and copy contents in the folder ad_project_src
5. 'cd .. && catkin_make' Build the package
6. 'source devel/setup.bash' 
7. Run the simulation in Copellia Sim
8. Launch the dronet perception node 'roslaunch dronet_perception dronet_launch.launch'
9. Launch the robot control node 'rosrun dronet_perception robot_node.py'
