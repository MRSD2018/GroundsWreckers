% SLAM_2D_NONLINEAR
% 16-831 Fall 2016 - Entire function provided
% Helper function for visualizing the incremental updates in 2D nonlinear
% SLAM
%
function update_plot(name, est_traj, est_landmarks, odom, gt_traj, gt_landmarks)

figure(1); clf; hold on;
%plot(gt_traj(:,1), gt_traj(:,2), 'k')
odom_only = [0 0; cumsum(odom)];
% plot(odom_only(:,1), odom_only(:,2), 'r')
plot(est_traj(1:end,1), est_traj(1:end,2), 'b')

%plot(gt_landmarks(:,1), gt_landmarks(:,2), 'ko')
plot(est_landmarks(:,1), est_landmarks(:,2), 'bx')

% legend('ground truth', 'pure odometry', 'full SLAM', 'Location', 'best');
legend('full SLAM', 'Landmarks', 'Location', 'best');
title(sprintf('Results for %s', name));
daspect([1 1 1])
% xlim([0 10]);
% ylim([0, 4.5]);
