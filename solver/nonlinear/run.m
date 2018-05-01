close all; clc; clear;

% % global
% robot1_odom_data =     '../Results/firstfloor_hallway/odom.csv';
% robot1_landmark_data = '../Results/firstfloor_hallway/landmarks.csv';
% 
% %local
% robot2_odom_data =     '../Results/firstfloor_loop/odom.csv'; 
% robot2_landmark_data = '../Results/firstfloor_loop/landmarks.csv';
% 
% %%%%%%%%
% %global
% % clear; clc;
% % fl = 3 % fake length
% % poses1 = 0:fl; %id
% % poses1 ( 2 , : ) = 0:fl; %x
% % poses1 ( 3 , : ) = 0; %y
% % dt = 1/4.0;
% % odom1 ( 1 , : ) = 1 : fl
% % odom1 ( 2 , : ) = poses1(2,2:fl+1 ) - poses1 (2,1:fl);
% % odom1 ( 3 , : ) = poses1(2,2:fl+1 ) - poses1 (2,1:fl);
% % odom1 ( 2:3,:) = odom1 ( 2:3,:) / dt;
% % 
% % 
% % l1( 1 , : ) = 1 : fl + 1;
% % l1( 2 , : ) = 69; % id
% % l1( 3 , : ) = 20 -  poses1 (2 , : );
% % l1( 4 , : ) = 0; %y 
% % l1( 5 , : ) = 0 %z
% % observations1 = l1
% % 
% % l2 = l1;
% % 
% % %l1 = 20 - poses1 (2:3 , : )
% % 
% % poses2 = poses1;
% % %poses2 ( 3 , : ) = 1 ;
% % %poses2(2,2:11 ) - poses2 (2,1:10);
% % odom2  = poses2(2,2:fl+1 )- poses2 (2,1:fl);
% % odom2 ( 2 , : ) = poses2(3,2:fl+1 ) - poses2 (3,1:fl);
% % 
% % ransac_transform = eye ( 3 );
% % ransac_transform ( 1, 3 ) = 1;
% % 
% % prior = struct
% % prior.x = fl;
% % prior.y = 0;
% % prior.theta = 0;
% % %prior.pose_id = size ( odom1 , 1 ) + 1; % pose at which robot_2 starts
% % prior.od_id = fl + 1;
% % odom1 = [ odom1' ; odom1' ]
% % observations1 = [ observations1' ; observations1' ];
% % observations1 ( : , 1 ) = 1 : size ( observations1 , 1 )
% % % pause
% % slam_2D_nonlinear ( odom1 , observations1 , prior )
% % return
% %%%%%%%%
% 
% addpath('../util');
% odom1         = csvread ( robot1_odom_data     );
% observations1 = csvread ( robot1_landmark_data );
% odom2         = csvread ( robot2_odom_data     );
% observations2 = csvread ( robot2_landmark_data );


% %%%%
% pose_id = 30
% odom1 = odom1 ( 1:pose_id , : );
% observations1 = observations1 ( observations1(:,1) <= pose_id , : )
% %%%
% 
% 
% %1 solve robot 1 pose
% slam_2D_nonlinear ( odom1 , observations1 )
% %slam_2D_nonlinear ( odom1 , observations1 )
% %2 solve robot 2 
% slam_2D_nonlinear ( odom2, observations2 )
% %3 compute robot 2 transform prior
% %load ('H21')


global_odom = load('../Results/firstfloor_hallway/odom.csv');
global_lm = load('../Results/firstfloor_hallway/landmarks.csv');

local_odom = load('../Results/firstfloor_loop/odom.csv');
local_lm = load('../Results/firstfloor_loop/landmarks.csv');


load('../Results/firstfloor_ransac/ransac_transform.mat')

%ransac_transform = inv ( ransac_transform )

prior = struct
prior.x = ransac_transform ( 1 , 3 );
prior.y = ransac_transform ( 2 , 3 );
prior.theta = acos ( ransac_transform ( 1 , 1 ) ) ;
prior.pose_id = size ( global_odom , 1 ) + 1; % pose at which robot_2 starts
% prior.pose_id = pose_id+1;

local_odom ( : , 1 ) = local_odom ( : , 1 ) + size ( global_odom , 1 );
local_lm ( : , 1 ) = local_lm ( : , 1 ) + size ( global_odom , 1 );

odom_full = [ global_odom ; local_odom];
observations_full = [ global_lm ; local_lm ];

%solve combined map
slam_2D_nonlinear ( odom_full , observations_full , prior )
