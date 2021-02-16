# root
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
<img align="right" width="350" height="220" src="./root_description/doc/images/root_labeled.png">

- Root is an open-source indoor simulated robot.
- It is built with the intention to test different algorithms related to:
	- Motion planning
	- Dynamics and Control
	- Perception
	- State estimation
- It contains a varity of commonly used sensor that can be easily enabled and disabled as per the requirement.
- With root, there comes root_home and its friends as workspace for it to operate in:
  
  1. root_home <br>
    <img width="300" height="250" src="./root_gazebo/screenshots/root_home.png"> 
  
  2. root_home_realistic <br>
    <img src="./root_gazebo/screenshots/root_home_realistic.png" width="300" height="250"/>

## Ground truth
- When comparing different state estimation algorithms (eg, sensor fusion of different odometry source, localization with a known map, simultaneous localization and mapping(SLAM) etc) ground truth information becomes very helpful.

- So for this purpos:
  1. ground truth layout has been made from the cad file of root_home <br>
     <img src="./root_gazebo/layouts/root_home/root_home.png" width="200" height="200"/>
  2. gt_pub node has beed made which publishes different ground truth informations:
      
      * Transforms <br>
    <img src="./root_gazebo/doc/images/gt_pub_frames.png" width="310" height="350"/>
      
      * Paths <br>
    <img src="./root_gazebo/doc/images/gt_pub_paths.png" width="450" height="350"/>
