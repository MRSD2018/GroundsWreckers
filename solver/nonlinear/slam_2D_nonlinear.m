% SLAM_2D_NONLINEAR
% 16-831 Fall 2016 - *Stub* Provided
% A 2D SLAM algorithm
%
% Arguments: 
%     none
%
% Returns:
%     none
%
function slam_2D_nonlinear(odom,observations, r2_prior)
  %% Load data
  %TODO: MODIFY THIS TO TAKE IN OUR DATA FORM
  %{
   gt_landmarks          100x2                1600  double              
    gt_traj              1000x2               16000  double              
    landmarks            1089x5               43560  double              
    observations        52566x4             1682112  double              
    odom                  999x2               15984  double              
    sigma_landmark          2x2                  32  double              
    sigma_odom              2x2                  32  double   
  %}
  %close all; clc; 
  if ~exist('r2_prior','var')
    r2_prior = struct;
    r2_prior.od_id = 1000000000; %do not use
  end

  addpath('../util');
  %odom        = csvread('../../csv/odom.csv');
  odom = odom ( : , 2:3 )
  %observations = csvread ('../../csv/landmarks.csv');
  %% Extract useful info
  n_poses = size(odom, 1);
  %n_landmarks = size(gt_landmarks, 1);
  n_landmarks = 210;
  n_odom = size(odom, 1);
  n_obs  = size(observations, 1);

  sigma_odom = [ 0.00025 0 0 ; 0 0.00025 0 ; 0 0 0.000025];
  sigma_landmark = [ 25 0 ; 0 25 ];

  %p_dim = size(gt_traj, 2);
  p_dim = 3;
  %l_dim = size(gt_landmarks, 2);
  l_dim = 2;
  o_dim = size(odom, 2);
  m_dim = size(observations(1, 3:end), 2);

  % A matrix is MxN, b is Nx1
  N = p_dim*n_poses + l_dim*n_landmarks;
  M = o_dim*(n_odom+1) + m_dim*n_obs;     % +1 for prior on the first pose

  %% Solve the SLAM problem at each step

  % Book-keeping
  poses = [0; 0; 0];
  all_landmarks = nan(n_landmarks, l_dim);
  n_seen = 0;
  x=0;
  A = 0;
  b = 0;
  landmark_map = 0;
  for i = 1:n_poses
      tps = (i-1)*p_dim+1;
      tpe = i*p_dim;
      lps = (i-2)*p_dim+1;
      lpe = (i-1)*p_dim;
      if (i > 1)
          % Update pose with odometry
          poses(tps:tpe) = poses(lps:lpe) + meas_odom_z ( odom(i-1, 1) , odom(i-1, 2) , poses (lpe) ); % was transpose...
      end
      %if i == r2_prior.od_id + 1
      %  poses(tps:tpe) = [ r2_prior.x ; r2_prior.y ; r2_prior.theta ] ;
      %end
      
      %%%% Add new landmarks %%%%
      obs = observations(observations(:,1) <= i, :);
      new_obs = obs(obs(:,1) == i, :);
      for j = 1:size(new_obs,1)
          landmark_idx = new_obs(j, 2);
          if isnan(all_landmarks(landmark_idx, 1))
              all_landmarks(landmark_idx, :) = project_measurement(poses(tps:tpe), new_obs(j, 3:end));
              n_seen = n_seen + 1;
          end
      end
      %%%% Re-index landmarks & combine into vector %%%%
      num = 1;
      landmark_map = nan(n_seen, 1);
      landmark_vec = nan(n_seen, 1);
      for j = 1:n_landmarks
          if ~isnan(all_landmarks(j, 1))
              obs(obs(:,2) == j, 2) = num;
              landmark_vec(l_dim*(num-1)+1:l_dim*num,:) = all_landmarks(j,:);
              landmark_map(num) = j;
              num = num + 1;
          end
      end
      

      %if size ( obs , 1 ) > 0 && size ( landmark_vec , 1 ) > 0
        x0 = [poses; landmark_vec];
       
        %%%% Update the solution using Gauss-Newton algorithm %%%%

        if i > 1
            [ x, A , b ]  = gauss_newton(x0, odom(1:i-1,:), obs, sigma_odom, sigma_landmark,r2_prior);
        else
            x = x0;
        end

        [traj, landmarks] = format_solution(x, i, n_seen, p_dim, m_dim);
        update_plot('Nonlinear SLAM', traj, landmarks, odom(1:i-1,:), r2_prior );
        %pause(0.01);
        
        %%%% Update poses and global landmarks %%%%
        for j = 1:i
            poses = x(1:i*p_dim);
        end
        
        updated_landmarks = x(length(poses)+1:end);
        for j = 1:n_seen
            landmark_idx = landmark_map(j);
            all_landmarks(landmark_idx, :) = updated_landmarks(l_dim*(j-1)+1:l_dim*j);
        end
      %end
  end


  %evaluate_method('Nonlinear SLAM', traj, landmarks, odom, gt_traj, gt_landmarks, true);
  save ('results','landmarks','traj','A','b','x','landmark_map');
end
