% PROJECT_BR_MEASUREMENT
% 16-831 Fall 2016 - *Stub* Provided
% Simple function project a bearing range measurement in 2D
%
% Arguments: 
%     pose        - x, y pose
%     measurement - bearing, range measurement
%
% Returns:
%     landmark    - 2D landmark location
%
function landmark = project_measurement(pose, measurement)
rx = pose ( 1 );
ry = pose ( 2 );
rt = pose ( 3 );
DX = measurement ( 1 );
DY = measurement ( 2 );
landmark(1) = rx + DX*cos(rt) + DY*sin(rt);
landmark(2) = ry + DX*sin(rt) - DY*cos(rt);