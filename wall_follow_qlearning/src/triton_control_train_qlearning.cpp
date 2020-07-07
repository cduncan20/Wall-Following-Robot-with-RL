#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <stdio.h>
#include <tuple>
#include <iostream>
#include <fstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber scan_subscriber;
sensor_msgs::LaserScan triton_scan;

const double PI = 3.14159265359;
typedef struct point_t {
    double x;
    double y;
} point_t;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr & laser_message);
bool cw_or_ccw(double desired_angle, double cur_angle);
tuple<double, double, double> find_robot_pose();
int find_current_state(vector<double> stateDivisors, double theta_err);
int find_action_e_greedy(vector<vector<double>> qVals, int nActions, int current_state);
int find_action_in_qVals(vector<vector<double>> qVals, int nActions, int current_state);
void apply_action(vector<double> action_angular_vel, int current_action, int current_state, double theta_des, double theta_err, double theta_robot_x, double x_speed);
vector<vector<double>> update_q_table(vector<vector<double>> qVals, int current_state, int new_state, int current_action, int nActions, double theta_err);
int write_qVals_to_file(vector<vector<double>> qVals, string file_path, string file_name);
int save_data(vector<vector<double>> qVals, int epochs, string file_path, string file_name);
vector<vector<double>> read_qVals_to_var(string file_path, string file_name);

int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "triton_control");
    ros::NodeHandle n;

    // Define Publisher & Subscriber
    velocity_publisher = n.advertise<geometry_msgs::Pose2D>("/triton_lidar/vel_cmd", 10);
    scan_subscriber = n.subscribe("/scan", 10, scanCallback);

    // /** test your code here **/
    ROS_INFO("\n\n\n*****START TRAINING*****\n");

    ros::Duration(5.0).sleep();

    // ***INITIALIZE Q-TABLE***
    // Q-Table of States (rows) and their corresponding Actions (columns)
	vector<vector<double>> qVals{
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },      // State 1 --> Between 180 & 120 degrees error
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },      // State 2 --> Between 120 &  80 degrees error
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },      // State 3 --> Between  80 &  40 degrees error
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },      // State 4 --> Between  40 &  20 degrees error
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },      // State 5 --> Between  20 &   5 degrees error
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, };   // State 6 --> Between   5 &   0 degrees error
    //1  2  3  4  5  6  7  8  9  10 <--------- Actions

    // Name of file where Q-Table will be saved
    string q_table_file_save_path = "/home/casey/catkin_ws/src/wall_follow_qlearning/q_tables/";
    string q_table_write_file_name = "qVals_qlearning_final_v1";

    // ***OR READ Q-TABLE FROM FILE***
    // Read Q-Table from .txt file. See below for definition of Q-Table
    // string q_table_read_file_name = "qVals_qlearning_final";
    // vector<vector<double>> qVals = read_qVals_to_var(q_table_file_save_path, q_table_read_file_name);

    // States separated into bins of robot angle error (theta_err)
	vector<double> stateDivisors{ PI, 120*PI/180, 80*PI/180, 40*PI/180, 20*PI/180, 5*PI/180, 0 };

    // Angular Velocity Actions based on percentage of robot angle error
    vector<double> action_angular_vel{ 0, 15, 30, 45, 60, 75, 90, 110, 130, 150};
    // Action -----------------------> 1   2   3   4   5   6   7    8    9   10

    int nStates = stateDivisors.size()-1;
    int nActions = action_angular_vel.size();

    // Move to starting point
    geometry_msgs::Pose2D vel_msg;
    double total_time = 2.0;
    double t0 =ros::Time::now().toSec();
    double t1 =ros::Time::now().toSec();
    while (t1-t0 < total_time)
    {
        vel_msg.x = 0.5;
        vel_msg.y = 0.0;
        vel_msg.theta = 0.0;
        velocity_publisher.publish(vel_msg);
        t1 = ros::Time::now().toSec();
    }

    // Folow wall for total_time in seconds
    total_time = 60.0*30.0;
    t0 =ros::Time::now().toSec();
    t1 =ros::Time::now().toSec();
    int current_state, current_action, new_state;
    double theta_des, theta_err, theta_robot_x, x_speed = 0.2;
    int iteration_count = 0;
    while (t1-t0 < total_time)
    {
        if (iteration_count == 0)
        {
            save_data(qVals, iteration_count, q_table_file_save_path, q_table_write_file_name);
        }
        else if (iteration_count % 10 == 0)
        {
            save_data(qVals, iteration_count, q_table_file_save_path, q_table_write_file_name);
        }

        // ***FIND CURRENT STATE***
        // Find what state robot is currently in
        if (iteration_count < 1)
        {
            tie(theta_des, theta_err, theta_robot_x) = find_robot_pose();
            current_state = find_current_state(stateDivisors, theta_err);
        }
        cout << "Current State = " << current_state << " which is within " << stateDivisors[current_state]*180/PI << " and " << stateDivisors[current_state+1]*180/PI << " degree error" << endl;
        
        // ***FIND ACTION***
        // Find what action robot should take using epsilon-greedy policy
        current_action = find_action_e_greedy(qVals, nActions, current_state);

        // ***APPLY ACTION***
        apply_action(action_angular_vel, current_action, current_state, theta_des, theta_err, theta_robot_x, x_speed);
        t1 = ros::Time::now().toSec();
        cout << "Total Time: " << t1 - t0 << " seconds." << endl;
        cout << endl;

        // ***FIND NEW STATE***
        tie(theta_des, theta_err, theta_robot_x) = find_robot_pose();
        new_state = find_current_state(stateDivisors, theta_err);

        // ***UPDATE Q-TABLE***
        qVals = update_q_table(qVals, current_state, new_state, current_action, nActions, theta_err);
        current_state = new_state;

        iteration_count += 1;
    }

    // Stop moving after time is complete
    vel_msg.x = 0.0;
    vel_msg.y = 0.0;
    vel_msg.theta = 0.0;
    velocity_publisher.publish(vel_msg);

    write_qVals_to_file(qVals, q_table_file_save_path, q_table_write_file_name);

    cout << "Press 'Ctrl+C to End" << endl;
    ros::spin();
    return 0;
}


// Calculate the robot's current positon with respect to the wall and the disired point infront of robot
tuple<double, double, double> find_robot_pose(){
    // Define desired distance between robot and closest wall
    double d_des, d_stan;
    d_des = 0.6; // Desired distance between robot and wall
    d_stan = 0.3; // Goal point ahead of robot set at desired distance from wall

    // Find distance to closest wall using lidar data
    double d_min = 1000; // Initialize dummy variable to be very large when finding minimum
    double d_index; // Index of smallest distance within all scanned distances
    for (int i = 0; i < triton_scan.ranges.size(); i++) 
    {
        if (triton_scan.ranges[i] < d_min) 
        {
            d_min = triton_scan.ranges[i];
            d_index = i;
        }
    }

    // Calculate Angle from Robot x-axis to closest wallint find_current_state(vector<double> stateDivisors, int nStates)
    double theta_robot_x = 2*PI - d_index*triton_scan.angle_increment;

    // Calculate desired angle (stanly angle)
    double theta_des = atan2(d_stan, d_min - d_des);

    // Calculate Angle to desired point (angle error)
    double theta_err = abs(theta_des - theta_robot_x);
    if (theta_err > PI)
    {
        theta_err = 2*PI - theta_err;
    }

    return make_tuple(theta_des, theta_err, theta_robot_x);
}


// Find the current state based on the theta_err
int find_current_state(vector<double> stateDivisors, double theta_err){
    int nStates = stateDivisors.size()-1;
    int current_state;
    for (int i = 0; i < nStates; ++i)
    {
        if (theta_err <= stateDivisors[i] && theta_err >= stateDivisors[i+1])
        {
            current_state = i;
        }
    }
    return current_state;
}


// Find action to take using e-greedy method
int find_action_e_greedy(vector<vector<double>> qVals, int nActions, int current_state){
    // Generate random number between 1 and 100
    int rand ();
    int rand_num = rand() % 100 + 1;

    // Define Explore Rate
    double explore_rate = 0.9;

    int current_action; // Initialize current_action variable

    // Find action using e-greedy method
    if (rand_num < explore_rate*100)
    {
        current_action = find_action_in_qVals(qVals, nActions, current_state);
    }
    else
    {
        current_action = rand() % nActions;
        cout << "RANDOM ACTION CHOSEN! Action Value = " << current_action << endl;
    }
    return current_action;
}


// Find the action with the largest Q-Table value within the current state.
int find_action_in_qVals(vector<vector<double>> qVals,  int nActions, int current_state){
    int current_action;
    double highest_reward = -10000; // Initialize action with highest award with an very small number.
    for (int i = 0; i < nActions; ++i)
    {
        if (highest_reward < qVals[current_state][i])
        {
            highest_reward = qVals[current_state][i];
            current_action = i;
        }
    }
    return current_action;
}


// Apply linear and angular velocities to robot
void apply_action(vector<double> action_angular_vel, int current_action, int current_state, double theta_des, double theta_err, double theta_robot_x, double x_speed){
    geometry_msgs::Pose2D vel_msg;
    ros::Rate loop_rate(10);

    // linear velocity in the x-axis & y-axis
    vel_msg.x = x_speed;
    vel_msg.y = 0.0;

    // Based on angle, turn clockwise of counter-clockwise
    bool clockwise = cw_or_ccw(theta_des, theta_robot_x);

    // Define angular speed depending on whether turtle will robot cw or ccw
    double angular_speed = action_angular_vel[current_action]*PI/180;
    if (clockwise){
        vel_msg.theta = -abs(angular_speed);
    }
    else{
        vel_msg.theta = abs(angular_speed);
    }

    cout << "Current Action = " << current_action << " which is " << vel_msg.theta * 180/PI << " degrees/sec." << endl;

    // Publish velocities to turtle
    velocity_publisher.publish(vel_msg);

    ros::spinOnce();
    loop_rate.sleep();
}


// Decide whether robot rotates clockwise or counter clockwise
bool cw_or_ccw(double desired_angle, double cur_angle){
    // Convert angles to non-negative angles if negative
    double desired_angle_2PI, real_angle_2PI;
    desired_angle_2PI = desired_angle;
    real_angle_2PI = cur_angle;
    if (desired_angle_2PI < 0){
        desired_angle_2PI = 2*PI + desired_angle;
    }
    if (real_angle_2PI < 0){
        real_angle_2PI = 2*PI + real_angle_2PI;
    }
    
    // Rotate clockwise or counterclockwise based on smallest rotation angle
    bool clockwise;
    if (abs(real_angle_2PI - desired_angle_2PI) < PI){
        clockwise = ((real_angle_2PI - desired_angle_2PI > 0)?true:false); // if positive -> cw. otherwise -> ccw
    }
    else {
        clockwise = ((real_angle_2PI - desired_angle_2PI < 0)?true:false); // if negative -> cw. otherwise -> ccw
    }
    return clockwise;
}


// Update Q-Table Values using Q-Learning Method
vector<vector<double>> update_q_table(vector<vector<double>> qVals, int current_state, int new_state, int current_action, int nActions, double theta_err){
    double relative_value = 1.0;
    double learn_rate = 1-abs(theta_err)/PI;
    double reward = PI-abs(theta_err); // Reword defined as the angle difference between the robots heading and where it's heading should be

    double highest_reward = -10000; // Initialize action with highest award with an very small number.
    for (int i = 0; i < nActions; ++i)
    {
        if (highest_reward < qVals[new_state][i])
        {
            highest_reward = qVals[new_state][i];
        }
    }

    qVals[current_state][current_action] = qVals[current_state][current_action] + learn_rate * (reward + relative_value*highest_reward - qVals[current_state][current_action]);

    return qVals;
}


// Save Q-Table values to a .txt file
int write_qVals_to_file(vector<vector<double>> qVals, string file_path, string file_name){
    ofstream myfile;
    myfile.open(file_path + file_name + ".txt");
    cout << "Writing File" << endl;
    if (myfile.is_open())
    {
        cout << "Writing To File" << endl;
        for (int state = 0; state < qVals.size(); state++)
        {
           for (int action = 0; action < qVals[0].size(); action++)
           {
               double qVal_rounded = roundf(qVals[state][action] * 1000) / 1000;
               myfile << qVal_rounded;
               myfile << "\t";
           }
           myfile << endl;
        }
        cout << "Closing File" << endl;
        myfile.close();
        cout << "File Closed" << endl;
    }
    return 0;
}


int save_data(vector<vector<double>> qVals, int epoch, string file_path, string file_name){
    // Write to File
    for (int state = 0; state < qVals.size(); state++)
    {
        stringstream ss;
        ss << state+1;
        string state_num;
        ss >> state_num;
        string state_file_name = file_path + file_name + "_state" + state_num + ".txt";
        ofstream myfile;
        if (epoch == 0)
        {
            myfile.open(state_file_name);
        }
        else
        {
            myfile.open(state_file_name, std::ios::app);
        }
        if (myfile.is_open())
        {
            for (int action = 0; action < qVals[0].size(); action++)
            {
                double qVal_rounded = roundf(qVals[state][action] * 1000) / 1000;
                myfile << qVal_rounded;
                myfile << "\t";
            }
            myfile << "\n";
        }
    }
    return 0;
}


// Import Q-Table from .txt file
vector<vector<double>> read_qVals_to_var(string file_path, string file_name){
    // Initialize Q-Table to populate from .txt file
	vector<vector<double>> qVals{
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },      // State 1 --> Between 180 & 120 degrees error
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },      // State 2 --> Between 120 &  80 degrees error
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },      // State 3 --> Between  80 &  40 degrees error
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },      // State 4 --> Between  40 &  20 degrees error
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },      // State 5 --> Between  20 &   5 degrees error
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, };   // State 6 --> Between   5 &   0 degrees error
    //1  2  3  4  5  6  7  8  9  10 <--------- Actions
    
    // Read each value within .txt file and save to qVals variable
    ifstream myfile;
    myfile.open(file_path + file_name + ".txt");
    cout << "Reading File" << endl;
    double value;
    int state = 0;
    int action = 0;
    while (myfile >> value)
    {
        qVals[state][action] = value;
        action += 1;
        if (action > qVals[0].size())
        {
            state += 1;
            action = 0;
        }
    }
    cout << "File Closed" << endl;
    return qVals;
}


// Read robot sensor information
void scanCallback(const sensor_msgs::LaserScan::ConstPtr & laser_message){
    triton_scan.angle_min = laser_message -> angle_min;
    triton_scan.angle_max = laser_message -> angle_max;
    triton_scan.angle_increment = laser_message -> angle_increment;
    triton_scan.time_increment = laser_message -> time_increment;
    triton_scan.scan_time = laser_message -> scan_time;
    triton_scan.range_min = laser_message -> range_min;
    triton_scan.range_max = laser_message -> range_max;
    triton_scan.ranges = laser_message -> ranges;
}