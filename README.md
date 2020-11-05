# GDAM - Goal Driven Autonomous Mapping

A goal-driven autonomous mapping and exploration system that combines reactive and planned robot navigation. First, a navigation policy is learned through a deep reinforcement learning framework in a simulated environment. This policy guides an autonomous agent towards a goal while avoiding obstacles. We develop a navigation system where this learned policy is integrated into a motion planning stack as the local navigation layer to move the robot towards the intermediate goals. A global path planner is used to mitigate the local optimum problem and guide the robot towards the global goal. Possible intermediate goal locations are extracted from the environment and used as local goals according to the navigation system heuristics. The fully autonomous navigation is performed without any prior knowledge while mapping is performed as the robot moves through the environment.


Main dependencies: 

* [ROS Melodic](http://wiki.ros.org/melodic/Installation)
* [Tensorflow 1.14](https://www.tensorflow.org/)
* [Tflearn](http://tflearn.org/)
* [ROSARIA](http://wiki.ros.org/action/show/ROSARIA?action=show&redirect=rosaria)
* [Move_base](http://wiki.ros.org/move_base)
* [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

Code of the experimental implementation is available in the "Code" folder.
Videos, images, and gifs of experiments are available in the "Experiments" folder.


*Extended video of experiments: 
[![EXPERIMENTS](https://img.youtube.com/vi/MhuhsSdzZFk/0.jpg)](https://www.youtube.com/watch?v=MhuhsSdzZFk)

*Examples of experimental results:
Robot World 2020 Expo
Goal location (80, 25)

<p align="center">
    <img width=40% src="https://github.com/reiniscimurs/GDAM/blob/main/experiments/Gifs/robotworldexp3.gif">
</p>
Final map
<p align="center">
    <img width=40% src="https://github.com/reiniscimurs/GDAM/blob/main/experiments/images/robotworldexp3.png">
</p>
