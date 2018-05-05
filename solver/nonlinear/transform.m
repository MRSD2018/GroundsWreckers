%%
%Params
%Error Threshold
threshold = .5;
%Ransac iterations
iterations = 50000;

%%
%Load stuff
load('../../csv/firstfloor_hallway/results.mat', ...
        'landmark_map', ...
        'landmarks',    ...
        'traj',         ...
        'x');
global_lm_map = landmark_map;
global_lm = landmarks;
global_lm(:,3) = 1;
global_traj = traj;
global_x = x;

load('../../csv/firstfloor_loop/results.mat', ...
        'landmark_map', ...
        'landmarks',    ...
        'traj',         ...
        'x');
local_lm_map = landmark_map;
local_lm = landmarks;
local_lm(:, 3) = 1;
local_traj = traj;
local_x = x;

%%
%Find correspondences
[idx, loc] = ismember(global_lm_map, local_lm_map);

%Left col is index for global, right col is index for local
correspondence_idx = find(idx);
correspondence_idx(:, 2) = loc(find(loc));

%%
%Ransac that shit
max_inliers = 0;
for r = 1:iterations
    %Random sample of 2 points
    sample_idx = randperm(size(correspondence_idx, 1), 2);
    
    %Extract coords for corresponding points 
    global_coords = global_lm( correspondence_idx( sample_idx, 1 ), : );
    local_coords  = local_lm(  correspondence_idx( sample_idx, 2 ), : );
    
    %Find rotation between maps
    dv1 = global_coords ( 2 , : ) - global_coords ( 1 , : );
    dv2 = local_coords ( 2 , : ) - local_coords ( 1 , : );

    thetaout = wrapToPi( atan2( dv1(2),dv1(1) ) - atan2( dv2(2), dv2(1) ) ); 
    
    %Find translation between maps
    R21 = [ cos( thetaout ) , -sin( thetaout ) , 0 ; ...
            sin( thetaout ) ,  cos( thetaout ) , 0 ; ...
            0 , 0 , 1 ]; 

    T = global_coords(1,:)' - R21 * local_coords(1,:)';
    
    %Transform local into global
    H = R21;
    H(1:2, 3) = T(1:2);
    transformed = H * local_lm(correspondence_idx(:,2),:)';
    
    %Find inliers
    distance = diag( pdist2( global_lm(correspondence_idx(:,1),:), transformed') );
    inliers = find(abs(distance) < threshold);
    if length(inliers) > max_inliers
        ransac_transform = H;
        max_inliers = length(inliers);
    end
    
end

transformed_lm = (ransac_transform * local_lm')';
transformed_traj = zeros(size(local_traj));
save ('H21','ransac_transform');

for i = 1:size(local_traj, 1)
    x = local_traj(i, 1);
    y = local_traj(i, 2);
    t = local_traj(i, 3);
    
    local_T = [ cos(t) -sin(t) x; ...
                sin(t)  cos(t) y; ...
                0 0 1 ];
    global_T = ransac_transform * local_T;
    
    transformed_traj(i,:) = [global_T(7) global_T(8) acos(global_T(1))];
end

plot(global_traj(:,1), global_traj(:,2), 'r')
hold on
plot(global_lm(:,1), global_lm(:,2), 'ro')
plot(transformed_traj(:,1), transformed_traj(:,2), 'b')
plot(transformed_lm(:,1), transformed_lm(:,2), 'b*')
legend('Global Trajectory', 'Global Landmarks', 'Transformed Trajector', 'Transformed Landmarks');
