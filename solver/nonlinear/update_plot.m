% SLAM_2D_NONLINEAR
% 16-831 Fall 2016 - Entire function provided
% Helper function for visualizing the incremental updates in 2D nonlinear
% SLAM
%
function update_plot(name, est_traj, est_landmarks, odom, r2_prior )

  figure(1); clf; hold on;
  %plot(gt_traj(:,1), gt_traj(:,2), 'k')
  %odom_only = [0 0; cumsum(odom)];
  %plot(odom_only(:,1), odom_only(:,2), 'r')

  plot(est_landmarks(:,1), est_landmarks(:,2), 'bx')
  if size ( odom , 1 ) >= r2_prior.od_id
    est_traj1 = est_traj ( 1 : r2_prior.od_id - 1 , : );
    est_traj2 = est_traj ( r2_prior.od_id + 1: end   , : );
    plot ( est_traj1 ( : , 1 ) , est_traj1 ( : , 2 ) , 'b' )
    plot ( est_traj2 ( : , 1 ) , est_traj2 ( : , 2 ) , ' r'  )
    legend('Landmarks', 'est. traj. robot 1' , 'est. traj. robot2', 'Location','southwest' );
  else
    plot(est_traj(1:end,1), est_traj(1:end,2), 'b')
    legend('Landmarks', 'est. traj. robot 1' , 'Location','southwest' );
  end

  %legend('ground truth', 'pure odometry', 'full SLAM', 'Location', 'best');
  title(sprintf('Results for %s', name));
  xmin = min ( est_traj ( : , 1 ) ) - 1 ;
  xmax = max ( est_traj ( : , 1 ) ) + 1 ;
  ymin = min ( est_traj ( : , 2 ) ) - 1 ;
  ymax = max ( est_traj ( : , 2 ) ) + 1 ;
  
  xlim([xmin xmax]);
  ylim([ymin ymax]);
  daspect ( [ 1 1 1 ] );
end
