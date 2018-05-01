close all; clc; 


robot1_odom_data =     '../../csv/firstfloor_hallway/odom.csv';
robot1_landmark_data = '../../csv/firstfloor_hallway/landmarks.csv';
robot2_odom_data =     '../../csv/firstfloor_loop/odom.csv'; % global?
robot2_landmark_data = '../../csv/firstfloor_loop/landmarks.csv';


robot1_odom_data =     '../../csv/fuck/odom1.csv';
robot1_landmark_data = '../../csv/fuck/landmarks1.csv';
robot2_odom_data =     '../../csv/fuck/odom2.csv'; % global?
robot2_landmark_data = '../../csv/fuck/landmarks2.csv';
%%%%%%%%
%global
clear; clc;
fl = 3 % fake length
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
l1( 3 , : ) = 20 -  poses1 (2 , : );
l1( 4 , : ) = 0; %y 
l1( 5 , : ) = 0 %z
observations1 = l1

l2 = l1;

%l1 = 20 - poses1 (2:3 , : )

poses2 = poses1;
%poses2 ( 3 , : ) = 1 ;
%poses2(2,2:11 ) - poses2 (2,1:10);
odom2  = poses2(2,2:fl+1 )- poses2 (2,1:fl);
odom2 ( 2 , : ) = poses2(3,2:fl+1 ) - poses2 (3,1:fl);

ransac_transform = eye ( 3 );
ransac_transform ( 1, 3 ) = 1;

prior = struct
prior.x = fl;
prior.y = 0;
prior.theta = 0;
%prior.pose_id = size ( odom1 , 1 ) + 1; % pose at which robot_2 starts
prior.od_id = fl + 1;
odom1 = [ odom1' ; odom1' ]
observations1 = [ observations1' ; observations1' ];
observations1 ( : , 1 ) = 1 : size ( observations1 , 1 )
pause
slam_2D_nonlinear ( odom1 , observations1 , prior )
return
%%%%%%%%

addpath('../util');
odom1         = csvread ( robot1_odom_data     );
observations1 = csvread ( robot1_landmark_data );
odom2         = csvread ( robot2_odom_data     );
observations2 = csvread ( robot2_landmark_data );


%%%%
pose_id = 30
odom1 = odom1 ( 1:pose_id , : );
observations1 = observations1 ( observations1(:,1) <= pose_id , : )
%%%


%1 solve robot 1 pose
%slam_2D_nonlinear ( odom1 , observations1 )
%slam_2D_nonlinear ( odom1 , observations1 )
%2 solve robot 2 
%slam_2D_nonlinear ( odom2, observations2 )
%3 compute robot 2 transform prior
%load ('H21')

%ransac_transform = inv ( ransac_transform )

prior = struct
prior.x = ransac_transform ( 1 , 3 );
prior.y = ransac_transform ( 2 , 3 );
prior.theta = acos ( ransac_transform ( 1 , 1 ) ) ;
%prior.pose_id = size ( odom1 , 1 ) + 1; % pose at which robot_2 starts
prior.pose_id = pose_id+1;

odom2 ( : , 1 ) = odom2 ( : , 1 ) + size ( odom1 , 1 );
observations2 ( : , 1 ) = observations2 ( : , 1 ) + size ( odom1 , 1 );

odom_full = [ odom1 ; odom2];
observations_full = [ observations1 ; observations2 ];

%solve combined map
slam_2D_nonlinear ( odom_full , observations_full , prior )
