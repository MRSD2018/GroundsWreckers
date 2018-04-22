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
function H = meas_landmark_jacobian(rx, ry, rt, lx, ly)

  dDX_drx = -cos(rt);
  dDY_drx = -sin(rt);
  dDX_dry = sin(rt);
  dDY_dry = -cos(rt);
  dDX_drt = -(lx-rx)*sin(rt) + (ly-ry)*cos(rt);
  dDY_drt = (lx-rx)*cos(rt) + (ly-ry)*sin(rt);
  dDX_dlx = cos(rt);
  dDY_dly = sin(rt);

  H ( 1 , 1 ) =  dDX_drx;
  H ( 1 , 2 ) =  dDX_dry;
  H ( 1 , 3 ) =  dDX_drt;
  H ( 1 , 4 ) =  dDX_dlx;
  H ( 1 , 5 ) =  dDX_dly;
  H ( 2 , 1 ) =  dDY_drx;
  H ( 2 , 2 ) =  dDY_dry;
  H ( 2 , 3 ) =  dDY_drt;
  H ( 2 , 4 ) =  dDY_dlx;
  H ( 2 , 5 ) =  dDY_dly;

end
