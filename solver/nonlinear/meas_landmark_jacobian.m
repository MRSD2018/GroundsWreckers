% MEAS_LANDMARK_JACOBIAN
% 16-831 Fall 2016 - *Stub* Provided
% Compute the Jacobian of the measurement function
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     H     - Jacobian of the measurement fuction
%
function H = meas_landmark_jacobian(rx, ry, lx, ly)

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  x =  lx - rx;
  y =  ly - ry;
  dtheta_drx =   ( y ) /       ( x^2 + y^2 );
  dtheta_dlx = - ( y ) /       ( x^2 + y^2 );
  dtheta_dry = - ( x ) /       ( x^2 + y^2 );
  dtheta_dly =   ( x ) /       ( x^2 + y^2 );
  dd_drx     = -   x   / sqrt  ( x^2 + y^2 );
  dd_dlx     =     x   / sqrt  ( x^2 + y^2 );
  dd_dry     = -   y   / sqrt  ( x^2 + y^2 );
  dd_dly     =     y   / sqrt  ( x^2 + y^2 );

  H ( 1 , 1 ) =  dtheta_drx;
  H ( 1 , 2 ) =  dtheta_dlx;
  H ( 1 , 3 ) =  dtheta_dry;
  H ( 1 , 4 ) =  dtheta_dly;
  H ( 2 , 1 ) =  dd_drx    ;
  H ( 2 , 2 ) =  dd_dlx    ;
  H ( 2 , 3 ) =  dd_dry    ;
  H ( 2 , 4 ) =  dd_dly    ;

end
