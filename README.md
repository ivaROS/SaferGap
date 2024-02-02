# Safer Gap: A Gap-based Local Planner for Safe Navigation with Nonholonomic Mobile Robots
This paper extends the gap-based navigation technique in Potential Gap by guaranteeing safety for nonholonomic robots for all tiers of the local planner hierarchy, so-called Safer Gap. The first tier generates a Bezier-based collision-free path through gaps. A subset of navigable free space from the robot to a gap, called the keyhole, is defined to be the union of the largest collision-free disc centered on the robot and a trapezoidal region directed through the gap. The keyhole free space is encoded by a shallow neural network-based zeroing barrier function (ZBF). In the next tier, Nonlinear Model Predictive Control (NMPC), with Keyhole ZBF constraints and output tracking of the Bezier path, synthesizes a safe kinematically feasible trajectory. Low-level use of projection operator, serves as a last action to enforcing safety if NMPC and Keyhole ZBF fail to converge to a solution within the prescribed time. Simulation and experimental validation of Safer Gap confirm its collision-free navigation properties.

<!--[[**Demo Video**]](https://youtu.be/hOzgUqUTOxY), [[**Arxiv Preprint**]]()-->

<!-- <img src="https://github.com/ivaROS/PotentialGap/blob/main/assets/coverImg.png" width = 55% height = 55%/> -->

## Supplementary materials

- [Parameters](https://github.com/ivaROS/SaferGap/blob/master/SuppMat/parameters.md)
- [Ablation Study Results](https://github.com/ivaROS/SaferGap/blob/master/SuppMat/assets/ablation_study_results.pdf)
- [3D Visualization of ZBF](https://github.com/ivaROS/SaferGap/blob/master/SuppMat/ZBF_3D_visual.md)

# Dependencies and Installation

- ROS (Noetic Ubuntu 20.04) [Installation Link](http://wiki.ros.org/noetic/Installation/Ubuntu)

- CasADi: Need to [compile from source](https://github.com/casadi/casadi/wiki/InstallationLinux) for usage within ROS.

- Google or-tools: [download the binary package](https://developers.google.com/optimization/install/cpp) on your system.

- Safer Gap: See the [installation instructions](https://github.com/ivaROS/SaferGap/blob/master/installation_guide.md)

# Run safer gap in simulation

## STDR

Start simulation
```
roslaunch bezier_gap_benchmark stdr_nonholonomic_sector_laser_world.launch
```

Start safer gap
```
roslaunch bezier_gap_benchmark bgap_mpc_casadi_nonholonomic_controller
```

Open Rviz, then send goal points
```
rviz -d PATH_TO_SAFER_GAP_BENCHMARK/safer_gap_benchmark/rviz/bgap_mpc_stdr.rviz
```

## Gazebo

Start simulation environment
```
roslaunch nav_configs gazebo_sector_world.launch
```

Spawn robot in the environment
```
roslaunch nav_configs spawn_turtlebot.launch
```

Start safer gap
```
roslaunch nav_scripts turtlebot_sgap_mpc_casadi_controller.launch
```

Open Rviz, then send goal points
```
rviz -d PATH_TO_SAFER_GAP_BENCHMARK/safer_gap_benchmark/rviz/bgap_mpc.rviz
```

# License
The source code is released under [MIT](https://opensource.org/licenses/MIT) license. 

