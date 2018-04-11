% MEAS_LANDMARK_JACOBIAN
% 16-831 Fall 2016 - *Stub* Provided
% Compute the Jacobian of the measurement function
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     rth   - robot's orientation
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     H     - Jacobian of the measurement fuction
%
function H = meas_landmark_jacobian(rx, ry, rth, lx, ly)

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  ddx_drx  = - cos ( rth );
  ddx_dry  = - sin ( rth );
  ddx_drth = - sin ( rth ) * ( lx - rx ) + cos ( rth ) * ( ly - ry );
  ddx_dlx  =   cos ( rth );
  ddx_dly  =   sin ( rth );
  ddy_drx  = - sin ( rth );
  ddy_dry  =   cos ( rth );
  ddy_drth =   cos ( rth ) * ( lx - rx ) + sin ( rth ) * ( ly - ry );
  ddy_dlx  =   sin ( rth );
  ddy_dly  = - cos ( rth );
  H ( 1 , 1 ) = ddx_drx  ;
  H ( 1 , 2 ) = ddx_dry  ;
  H ( 1 , 3 ) = ddx_drth ;
  H ( 1 , 4 ) = ddx_dlx  ;
  H ( 1 , 5 ) = ddx_dly  ;
  H ( 2 , 1 ) = ddy_drx  ;
  H ( 2 , 2 ) = ddy_dry  ;
  H ( 2 , 3 ) = ddy_drth ;
  H ( 2 , 4 ) = ddy_dlx  ;
  H ( 2 , 5 ) = ddy_dly  ;


end
