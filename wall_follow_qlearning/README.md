<h1>Human Centered Robotics: Q-Learning Robot Reinforcement Learning</h1>

**Author:** Casey Duncan <br />
**Date:** 04/27/2020 <br />
**Course:** CSCI 573 - Human Centered Robotics <br />
**Assignment:** Reinforcement Learning for Robot Wall Following <br />

<h2>OVERVIEW</h2>

The goal of this assignment was to write a ROS node that allows the robot to follow a wall using a Q-table that it populated through the Q-Learning Reinforcement Learning training method. The states must be the robot's sensor data and based on this state and the Q-table, the the published action should allow for the robot to follow a wall. Within the same folder as this *README.md* file is the ROS package containing the code to execute the solution to this assignment. Additionally, a demo video of the working robot has been posted to youtube. The ROS package name and video link are provided below:
- ROS Package Folder Name: *wall_follow_qlearning* <br />
- Demo Video Link: https://youtu.be/DFaiHgMJvXc

Follow the steps below for compiling and running the code yourself.


<h2>HOW TO RUN CODE</h2>

1. Copy and paste the ROS Package Folder folder (see folder name above) into the src folder within the catkin_ws on your Ubuntu 18.04 machine. The location on my Ubuntu 18.04 machine can be seen below:
    - Location: */home/casey/catkin_ws/src* <br />
2. Open a new terminal on your Ubuntu 18.04 machine. This can be done by either navigating to the navigating to the dashboard, typing in `Terminal`, and hitting enter, or by just hitting the keyboard shortcut `Ctrl-Atl-T`.
3. Navigate to the catkin workspace by typing in `cd ~/catkin_ws/`
4. Build catkin workspace by typing in `catkin_make`
5. Check that the make created a local setup file by typing in the following commands:<br />
	`source /opt/ros/melodic/setup.bash`<br />
	`source ~/catkin_ws/devel/setup.bash`<br />
	`source ~/catkin_ws/src/wall_follow_qlearning/wall_follow_qlearning.bash`
6. Train the Q-Table:
	1. Go the the following file location and open the following file:
	    - File Location: */home/casey/catkin_ws/src/wall_follow_qlearning/src/* <br />
	    - File Name: *triton_control_train_qlearning.cpp* <br />
	2. Change the path for where the trained q-table will be saved and the q-table file name by changing the `q_table_file_save_path` and `q_table_write_file_name` variable values (Lines 60 & 61). For example, on my computer the values are as follows:<br />
	    `string q_table_file_save_path = "/home/casey/catkin_ws/src/wall_follow_qlearning/q_tables/";`<br />
	    `string q_table_write_file_name = "qVals_qlearning_final_v2";`
	    - If you would like to start from a q-table initialized to be all zeros, comment out lines 65 & 66 and uncomment out lines 50 - 56. 
	    - If you would like to use a pre-trained q-table from an existing *.txt* file, comment out lines 50 - 56 and uncomment out lines 65 & 66. Also, update the `q_table_read_file_name` variable name (line 65) to be the name of the pre-trained q-table file. For example, on my computer the value is: <br />
			`string q_table_read_file_name = "qVals_qlearning_final";`
	3. Change the training time to the disired number of seconds by changing the value of the `total_time` variable (line 93). It is currently set to 3600.0 seconds, which is 1 hour.
	4. Run the launch file within the ROS package. This will open the gazebo world and simulate the robot wall following algorithm. This can be done by typing in the following command:<br />
		`roslaunch wall_follow_qlearning wall_following_train_qlearning.launch`
7. Test the Q-Table:
	1. Go the the following file location and open the following file:
	    - File Location: */home/casey/catkin_ws/src/wall_follow_qlearning/src/* <br />
	    - File Name: *triton_control_test_qlearning.cpp* <br />
	2. Change the path for where the trained q-table is located and the q-table file name by changing the `q_table_file_path` and `q_table_file_name` variable values (Lines 47 & 48). For example, on my computer the values are as follows:<br />
	    `string q_table_file_name = "qVals_qlearning_final";`<br />
	    `string q_table_write_file_path = "/home/casey/catkin_ws/src/wall_follow_qlearning/q_tables/";`
	3. Change the training time to the disired number of seconds by changing the value of the `total_time` variable (line 84). It is currently set to 300.0 seconds, which is 5 minutes.
	4. Run the launch file within the ROS package. This will open the gazebo world and simulate the robot wall following algorithm. This can be done by typing in the following command:<br />
		`roslaunch wall_follow_qlearning wall_following_test_qlearning.launch`

After completing the steps above, the triton robot should find a wall within the gazebo world and begin following it using the trained Q-Table. Please note that there is a 5 second delay before it begins.


<h2>SOLUTION DESCRIPTION</h2>

To follow a wall, I used the stanley control method, which basically tried to drive towards a user set point infront of the robot. I set the point to always be a distance of 0.6 from the closest detected wall and a distance of 0.3 infront of the robot. I wrote the code to do this using C++ and the solution training & test code can be found within the src folder within the ROS Package Folder. The psuedocode to solve this problem is shown below:

1. Using the robots LIDAR data, detect the smallest distance to the closest wall and the angle between the robot's heading and the wall.
2. Based on robots distance from the closest wall, the user set desired distance from wall (0.6), and the user set distance to goal point infront of robot (0.3) that is also at the desired distance from the wall, calculate the desired angle between the robot's heading and the closest wall.
3. Based on the robot's current heading and the desired heading, calculate whether the robot needs to turn clockwise or counter clockwise. Additonally calculate the robot's heading error, which is the absolute value in difference between the robot's current heading and it's desired heading.
4. Based on the robot's heading error, calculate the current state of the robot. The states in the Q-table are currently defined as the following heading errors:
	- State 1 --> Between 180 & 120 degrees error
	- State 2 --> Between 120 &  80 degrees error
	- State 3 --> Between  80 &  40 degrees error
	- State 4 --> Between  40 &  20 degrees error
	- State 5 --> Between  20 &   5 degrees error
	- State 6 --> Between   5 &   0 degrees error
5. Based on the robot's current state, find the largest value within that state (row) in the Q-Table and what column index this value is in. The index value is associated with the action applied to the robot, which is defined robot's angular velocity in degrees / second. Current action angular velocities are set up as follows:
	- Action  1 -->   0 degrees/second
	- Action  2 -->  15 degrees/second
	- Action  3 -->  30 degrees/second
	- Action  4 -->  45 degrees/second
	- Action  5 -->  60 degrees/second
	- Action  6 -->  75 degrees/second
	- Action  7 -->  90 degrees/second
	- Action  8 --> 110 degrees/second
	- Action  9 --> 130 degrees/second
	- Action 10 --> 150 degrees/second
6. Publish linear velocity, which is constantly set to be 0.3 units per second in the x-direction and 0.0 units per second in the y-direction, and the angular velocity (defined by action above) to the robot.
7. Repeat steps 1-6 above until maximum time limit has been reached.

See the *Final Report.pdf* file within the main repository folder for a more detailed description on the solution. Also, see the C++ code within the *src* folder to see how this was implemented. 
