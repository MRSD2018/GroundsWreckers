clear;

%%
%Params
%Error Threshold
threshold = .5;
%Ransac iterations
iterations = 50000;

%%
%Load shit
load('../results/outer_loop4/results.mat', ...
        'landmark_map', ...
        'landmarks',    ...
        'traj',         ...
        'x');
global_lm_map = landmark_map;
global_lm = landmarks;
global_lm(:,3) = 1;
global_traj = traj;
global_x = x;

load('../results/extended_loop1/results.mat', ...
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

%     s = cross ( dv1 , dv2 );
%     s = -sign ( s ( 3 ) );
% 
%     thetaout = s * acos ( ( dv1 * dv2' ) / ( norm ( dv1 ) * norm ( dv2 ) ) );

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

ransac_transformed = ransac_transform * local_lm(correspondence_idx(:,2),:)';
ransac_res = sum( diag( pdist2( global_lm(correspondence_idx(:,1),:), ransac_transformed')))

figure
plot(global_lm(correspondence_idx(:,1),1), global_lm(correspondence_idx(:,1),2), 'o');
hold on
% plot(local_lm(correspondence_idx(:,2),1), local_lm(correspondence_idx(:,2),2), 'x');

plot(ransac_transformed(1,:), ransac_transformed(2,:), '*');
hold off





