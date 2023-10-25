# deep_steer
Steering using deep learning

""
How to run Kuruppu Arachchige Sasanka 2021227162

Run 'roscore'
cd to Copellia sim root directory './coppeliaSim.sh'
Open scene in vrepscene/dronet_1.ttt
'cd catkin_ws/src' and copy contents in the folder ad_project_src
'cd .. && catkin_make' Build the package
'source devel/setup.bash'
Run the simulation in Copellia Sim
Launch the dronet perception node 'roslaunch dronet_perception dronet_launch.launch'
Launch the robot control node 'rosrun dronet_perception robot_node.py'

""
