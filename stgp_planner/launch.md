# Launch the simulation environment
source devel/setup.bash && roslaunch exploration_manager sim_env.launch  world:=Scene4

# Launch the exploration policy
source devel/setup.bash && roslaunch exploration_manager explorer.launch world:=Scene4

# Launch the robot movement control
source devel/setup.bash && roslaunch exploration_manager robot_move.launch
