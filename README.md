# root
[![Website shields.io](https://img.shields.io/website-up-down-green-red/http/shields.io.svg)](https://root-org.github.io/) [![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

<img align="right" width="350" height="220" src="./root_description/docs/root_labeled.png">

- Root is an open-source indoor simulated robot.
- It is built with the intention to test different algorithms related to:
	- Motion planning
	- Dynamics and Control
	- Perception
	- State estimation
- It contains a varity of commonly used sensor that can be easily enabled and disabled as per the requirement.
- With root, there comes root_home and its friends as workspace for it to operate in:
 	1. root_home
 		- <img width="300" height="250" src="./root_gazebo/screenshots/root_home.png"> 
	2. root_home_realistic
		- <img src="./root_gazebo/screenshots/root_home_realistic.png" width="300" height="250"/>

## Ground truth
- When comparing different state estimation algorithm (eg, sensor fusion of different odometry source, localization with a known map, simultaneous localization and mapping(SLAM) etc) ground truth information becomes very helpful.

- So for this purpos:
	1. ground truth layout has been made from the cad file of root_home
		- <img src="./root_gazebo/layouts/root_home/root_home.png" width="200" height="200"/>
	2. gt_pub node has beed made which publishes different ground truth information.
		- <img src="./root_gazebo/docs/gt_pub frames.png" width="500" height="500"/>
		- Example:
			- <img src="./root_gazebo/docs/gt_pub_example.png" width="500" height="400"/>
## Reated package stacks
1. [root_bringup](https://github.com/root-org/root_bringup)
2. [root_state_estimation](https://github.com/root-org/root_state_estimation)
3. [root_navigation](https://github.com/root-org/root_navigation)
