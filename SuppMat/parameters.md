# Safer Gap Parameters and Values Used

The implementation has been configured for use by a Turtlebot 2 robot. Many of the parameter values are geometric quantities that should scale with the Turtlebot's radius and its other properties. The parameters are organized according to the different sections within the Safer Gap manuscript.

### Composite Bezier Path Planning

| Parameter |        Value      | Meaning |
| --------- | :---------------: | ------- |
| s<sub>inf</sub> | 0.22 | Inflation size to generate the inflated collision free space. It is a function of robot radius. |
| w<sub>1</sub> | 20 | Terminal weight in the path scoring function. It determines how close the end pose of path is to local waypoint. |
| w<sub>2</sub> | 20 | Orientation weight in the path scoring function. It penalizes the orientation difference between end pose and initial pose.  |
| w<sub>3</sub> | 3 | Decay factor on penalty for getting close to an obstacle point. |
| c<sub>obs</sub> | -1 | Amplification factor and sign correction for the obstacle proximity penalty. |
| r<sub>ins</sub> | 0.22 | Inscribed radius of the robot. Determined by geometry of the robot. Value here is for the Turtlebot. |
| r<sub>max</sub> | 0.3 | Maximum distance to decide if the cost is zero. If the distance to obstacle is large enough, there is no need to include in the cost computing. This value should be larger than r<sub>ins</sub>. |

### Keyhole ZBF Synthesis
| Parameter |        Value      | Meaning |
| --------- | :---------------: | ------- |
|  |  |  |

### NMPC Trajectory Tracking

| Parameter |        Value      | Meaning |
| --------- | :---------------: | ------- |
| v<sub>d</sub> | 0.4 | The desired velocity for generating trajectories. |
| Q | [10, 10, 0] | Diagonal matrix for state weight. |
| R | [1, 1] | Diagonal matrix for control weight. |
| u<sub>lb</sub> | [0 ; -4] | Lower bounds for controls |
| u<sub>ub</sub> | [0.5 ; 4] | Upper bounds for controls |
| a<sub>lb</sub> | [-5 ; -1.75] | Lower bounds for acceleration of controls |
| a<sub>ub</sub> | [5 ; 1.78] | Upper bounds for acceleration of controls |
| &Delta;t | 0.1 | Time different between two consecutive steps in NMPC |

### Projection Operator

| Parameter |        Value      | Meaning |
| --------- | :---------------: | ------- |
| r<sub>min</sub> | 0.2 | Potential field minimal radius, at which distance the level-set value is 1. Should be related to the inscribed radius. |
| r<sub>nom</sub> | 0.36 | Potential field nominal radius, at which distance the level-set value is 0. Determines when correction response starts to reshape the command vector to preserve forward invariance. Should be some multiple of the circumscribed radius of the robot. The difference between r<sub>min</sub> and r<sub>nom</sub> defines a band in space where command reshaping happens. A smaller difference gives a smaller region and more sensitive reshaping.  A larger difference gives a larger region and less aggressive reshaping. The value given is the one used for the Turtlebot. |
| k<sub>po,x</sub> | 1.2 | Conversion gain for mapping projection operator modification in x direction to linear velocity |
| k<sub>po,w</sub> | 1 | Conversion gain for mapping projection operator modification in y direction to angular velocity |

### Ablation Study Parameters

Feedback gains for PG<sup>-</sup>, SG<sup>-</sup>, PG, and SG. Other parameters are the same as above.

| Planner | Parameter |        Value      | Meaning |
| ------- | --------- | :---------------: | ------- |
| PG<sup>-</sup>/SG<sup>-</sup> | k<sub>x</sub> | STDR: 4, Gazebo: 3 | Conversion gain for mapping longitudinal feedback to forward speed. |
|  | k<sub>y</sub> | STDR: 4, Gazebo: 3.5 | Conversion gain for mapping lateral feedback to turn rates for standard nonholonomic vehicles. |
|  | k<sub>w</sub> | STDR: 0.5, Gazebo: 2 | Conversion gain for mapping orientation feedback to turn rates for standard nonholonomic vehicles. |
| PG/SG | k<sub>x</sub> | STDR: 2, Gazebo: 2 | Conversion gain for mapping longitudinal feedback to forward speed. |
|  | k<sub>y</sub> | STDR: 3.5, Gazebo: 3.5 | Conversion gain for mapping lateral feedback to turn rates for standard nonholonomic vehicles. |
|  | k<sub>w</sub> | STDR: 0.5, Gazebo: 0.5 | Conversion gain for mapping orientation feedback to turn rates for standard nonholonomic vehicles. |
