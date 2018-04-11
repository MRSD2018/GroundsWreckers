% MEAS_ODOM
% 16-831 Fall 2016 - *Stub* Provided
% Simple function to predict a linear odometry measurement between two
% poses
%
% Arguments: 
%     rx1   - robot's previous x position
%     ry1   - robot's previous y position
%     rx2   - robot's next x position
%     ry2   - robot's next y position
%
% Returns:
%     h     - odometry measurement prediction 
%
%function h = meas_odom(rx1, ry1, rx2, ry2)
function h = meas_odom( vr , vl , rth )
  
  l = 1 ;  % wheelbase
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  drx = ( vr + vl ) * cos ( rth ) / 2;
  dry = ( vr + vl ) * sin ( rth ) / 2;
  dth = ( vr + vl ) / l              ;
  %h = [ rx2 - rx1 ; ry2 - ry1 ];
  h = [ drx ; dry ; dth ];

end
  
