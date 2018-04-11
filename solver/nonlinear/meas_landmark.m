% MEAS_LANDMARK
% 16-831 Fall 2016 - *Stub* Provided
% Simple function to predict a nonlinear landmark measurement from a pose
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     h     - odometry measurement prediction = [ theta 
%                                                   d  ]
%
function h = meas_landmark(rx, ry, lx, ly)

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  h = [ ];
  theta = atan2 (   ly - ry     ,   lx - rx );
  d     = sqrt  ( ( lx - rx )^2 + ( ly - ry )^2 );
  h ( 1 , 1 ) = theta;
  h ( 2 , 1 ) = d;
end
  
  
