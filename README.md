# A Skeleton-Based Topological Planner for Exploration in Complex Unknown Environments
This repository contains the implementation of the paper "[A Skeleton-Based Topological Planner for Exploration in Complex Unknown Environments](https://arxiv.org/abs/2412.13664)", which is accepted by ICRA 2025.

## Getting Started

The project has been tested on **Ubuntu 20.04 (ROS Noetic)**.

```shell
sudo apt install python3-catkin-tools
mkdir -p ${YOUR_WORKSPACE_PATH}/src
cd ${YOUR_WORKSPACE_PATH}/src
git clone git@github.com:Haochen-Niu/STGPlanner.git
cd ..
catkin init
catkin build
```

After building, source your workspace and launch the simulation and exploration nodes:

```shell
# Launch the simulation environment
source devel/setup.zsh && roslaunch sim_env.launch  world:=Scene4

# Launch the exploration policy
source devel/setup.zsh && roslaunch exploration_manager explorer.launch world:=Scene4

# Launch the robot movement control
source devel/setup.zsh && roslaunch exploration_manager robot_move.launch
```

**Note:** To open the Gazebo GUI window during simulation, add the parameter `gazebo_gui:=true` to the launch commands, for example:
`roslaunch sim_env.launch world:=Scene4 gazebo_gui:=true`

## Citation

> @ARTICLE{2024arXiv241213664N,
>        author = {{Niu}, Haochen and {Ji}, Xingwu and {Zhang}, Lantao and {Wen}, Fei and {Ying}, Rendong and {Liu}, Peilin},
>         title = "{A Skeleton-Based Topological Planner for Exploration in Complex Unknown Environments}",
>       journal = {arXiv e-prints},
>      keywords = {Computer Science - Robotics},
>          year = 2024,
>         month = dec,
>           eid = {arXiv:2412.13664},
>         pages = {arXiv:2412.13664},
>           doi = {10.48550/arXiv.2412.13664},
> archivePrefix = {arXiv},
>        eprint = {2412.13664},
>  primaryClass = {cs.RO},
>        adsurl = {https://ui.adsabs.harvard.edu/abs/2024arXiv241213664N},
>       adsnote = {Provided by the SAO/NASA Astrophysics Data System}
> }
