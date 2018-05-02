close all; clc;  clear;


robot1_odom_data =     '../../csv/firstfloor_hallway/odom.csv';
robot1_landmark_data = '../../csv/firstfloor_hallway/landmarks.csv';
robot2_odom_data =     '../../csv/firstfloor_loop/odom.csv'; % global?
robot2_landmark_data = '../../csv/firstfloor_loop/landmarks.csv';

%hallway is global
%loop origin is at , like x=35 , y = 5



addpath('../util');
odom1         = csvread ( robot1_odom_data     );
observations1 = csvread ( robot1_landmark_data );
odom2         = csvread ( robot2_odom_data     );
observations2 = csvread ( robot2_landmark_data );


%%%%
%od_cap = 2
%odom1 = odom1 ( 1:od_cap , : );
%observations1 = observations1 ( observations1(:,1) <= od_cap , : )
%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Solve Robot 1 and Robot 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%slam_2D_nonlinear ( odom1 , observations1 )
%slam_2D_nonlinear ( odom2, observations2 )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute robot 2 transform prior %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load ('H21')
%ransac_transform = inv ( ransac_transform )

prior = struct
prior.x = ransac_transform ( 1 , 3 );
prior.y = ransac_transform ( 2 , 3 );
prior.theta = acos ( ransac_transform ( 1 , 1 ) ) ;
prior.x_cov  = 10;
prior.y_cov  = 10;
prior.th_cov = 10;
prior.od_id = size ( odom1 , 1 ) + 1; % pose at which robot_2 starts

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% For test porpoises %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
prior.x = 35
prior.y = 5
prior.theta = 0.3475

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% HACK For test porpoises %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
prior.od_id = 3;
odom1 = odom1( 1:prior.od_id , : );
observations1 = observations1 ( observations1 ( : , 1 ) < prior.od_id , : )
%}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Adjust pose ids for robot 2 to append to robot 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
odom2 ( : , 1 ) = odom2 ( : , 1 )                 + size ( odom1 , 1 ); 
observations2 ( : , 1 ) = observations2 ( : , 1 ) + size ( odom1 , 1 );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Merge robot 1 and robot 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
odom_full = [ odom1 ; odom2];
observations_full = [ observations1 ; observations2 ];

%solve combined map
slam_2D_nonlinear ( odom_full , observations_full , prior )


%{
%%%%%%%%
%global
fl = 4 % fake length
poses1 = 0:fl; %id
poses1 ( 2 , : ) = 0:fl; %x
poses1 ( 3 , : ) = 0; %y
dt = 1/4.0;
odom1 ( 1 , : ) = 1 : fl
odom1 ( 2 , : ) = poses1(2,2:fl+1 ) - poses1 (2,1:fl);
odom1 ( 3 , : ) = poses1(2,2:fl+1 ) - poses1 (2,1:fl);
odom1 ( 2:3,:) = odom1 ( 2:3,:) / dt;


l1( 1 , : ) = 1 : fl + 1;
l1( 2 , : ) = 69; % id
l1( 3 , : ) = 3 -  poses1 (2 , : );
l1( 4 , : ) = 0; %y 
l1( 5 , : ) = 0 %z
observations1 = l1
observations2 = observations1
observations2 ( 4 , : ) = observations2 ( 4 , : ) + 1
pause

%l1 = 20 - poses1 (2:3 , : )

odom2 = odom1;

ransac_transform = eye ( 3 );
ransac_transform ( 1, 3 ) = 1;

prior = struct
prior.x = 0;
prior.y = 1;
prior.theta = 0;
%prior.pose_id = size ( odom1 , 1 ) + 1; % pose at which robot_2 starts
prior.od_id = fl + 1;
odom1 = [ odom1' ; odom1' ]
observations1 = [ observations1' ; observations2' ];
observations1 ( : , 1 ) = 1 : size ( observations1 , 1 )

slam_2D_nonlinear ( odom1 , observations1 , prior )
return
%%%%%%%%%%%%%%%
%}

