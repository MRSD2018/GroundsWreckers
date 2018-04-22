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
function h = meas_landmark(rx, ry, rt, lx, ly)

  h = [ ];
  deltaX = cos(rt)*(lx-rx) + sin(rt)*(ly-ry);
  deltaY = sin(rt)*(lx-rx) - cos(rt)*(ly-ry);
  h ( 1 , 1 ) = deltaX;
  h ( 2 , 1 ) = deltaY;
end
  
  
