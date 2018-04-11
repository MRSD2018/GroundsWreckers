% MEAS_LANDMARK
% 16-831 Fall 2016 - *Stub* Provided
% Simple function to predict a nonlinear landmark measurement from a pose
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     rth   - robot's angle in world frame
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     h     - odometry measurement prediction = [ theta 
%                                                   d  ]
%
function h = meas_landmark(rx, ry, rth, lx, ly)

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  h = [ ];
  deltax = cos ( rth ) * ( lx - rx ) + sin ( rth ) * ( ly - ry );
  deltay = sin ( rth ) * ( lx - rx ) + sin ( rth ) * ( ly - ry );
  h ( 1 , 1 ) = deltax;
  h ( 2 , 1 ) = deltay;
end
  
  
