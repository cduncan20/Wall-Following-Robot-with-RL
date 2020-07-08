<h1>Human Centered Robotics: Robot Reinforcement Learning</h1>

**Author:** Casey Duncan <br />
**Date:** 04/27/2020 <br />
**Course:** CSCI 573 - Human Centered Robotics <br />
**Assignment:** Reinforcement Learning for Robot Wall Following <br />

<h2>OVERVIEW</h2>

The goal of this assignment was to write a ROS node that allows the robot to follow a wall using Q-tables that it populated through the Q-Learning & SARSA Reinforcement Learning training methods. The states must be the robot's sensor data and based on this state and the Q-table, the the published action should allow for the robot to follow a wall. Within the same folder as this *README.md* file are the ROS packages containing the code to execute the solutions to this assignment. Additionally, demo videos of the working robot for each algorithm have been posted to youtube. The ROS package names and video links are provided below:


1. Q-Learning Algorithm <br />
    - ROS Package Folder Name: *wall_follow_qlearning* <br />
    - Demo Video Link: https://youtu.be/DFaiHgMJvXc
2. SARSA Algorithm <br />
    - ROS Package Folder Name: *wall_follow_sarsa* <br />
    - Demo Video Link: https://youtu.be/RI4NhKOeDt8

Within each ROS package folder is a *README.md* file. Follow the steps within these *README.md* files for compiling and running the code yourself. See the *Final Report.pdf* file for a detailed description on the solution for this project.
