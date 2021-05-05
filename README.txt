
# ENPM809E
# Group 2
# Alec Lahr, Dhyey Patel, Jeet Patel, Mrugesh Shah

--------------------

Run the following commands in the order given
Each rosrun/roslaunch needs its own terminal

 1. roslaunch final_project_2 multiple_robots.launch

 2. roslaunch final_project_2 navigation.launch
    - wait for multiple_robots.launch to fully finish loading or you may have localization issues

 3. rosrun final_project_2 spawn_marker_on_robot

 4. rosrun final_project_2 follow

 5. cd /src/final_project_2/maps/
    rosrun map_server map_server small_house_map.yaml

 6. rosrun final_project_2 lead

--------------------

Note:   You can verify that follower is correctly identifying where it needs to go to
        follow the leader by viewing the /follower_tf/marker_goal tf in Rviz. After manually
        moving the leader in Gazebo, you should see the marker_goal tf update to be 0.7 world
        distance units behind the leader.
