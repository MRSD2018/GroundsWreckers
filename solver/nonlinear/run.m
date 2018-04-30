close all; clc; 

robot1_odom_data =     '../../csv/odom.csv';
robot1_landmark_data = '../../csv/landmarks.csv';
robot2_odom_data =     '../../csv/odom.csv';
robot2_landmark_data = '../../csv/landmarks.csv';

addpath('../util');
odom1         = csvread ( robot1_odom_data     );
observations1 = csvread ( robot1_landmark_data );
odom2         = csvread ( robot2_odom_data     );
observations2 = csvread ( robot2_landmark_data );

prior.x = 69;
prior.y = 6969;
prior.theta = 696969;
prior.pose_id = 3;
prior.active = true;


%1 solve robot 1 pose
slam_2D_nonlinear ( odom1 , observations1 , prior )
%slam_2D_nonlinear ( odom1 , observations1 )
%2 solve robot 2 
slam_2D_nonlinear ( odom1 , observations2 )
%3 compute robot 2 transform prior

%{
%Setup system to pass in robot 2 prior info
prior = struct
prior.x = 0;
prior.y = 0;
prior.theta = 0;
prior.pose_id = 0; % pose at which robot_2 starts
%}

%solve combined map
%slam_2D_nonlinear ( odom1 , observations2 )
