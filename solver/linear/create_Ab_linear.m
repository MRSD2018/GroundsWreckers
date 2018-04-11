% CREATE_AB_LINEAR
% 16-831 Fall 2016 - *Stub* Provided
% Computes the A and b matrices for the 2D linear SLAM problem
%
% Arguments: 
%     odom    - Matrix that contains the odometry measurements
%               between consecutive poses. Each row corresponds to
%               a measurement. 
%                 odom(:,1) - x-value of odometry measurement
%                 odom(:,2) - y-value of odometry measurement
%     obs     - Matrix that contains the landmark measurements and
%               relevant information. Each row corresponds to a
%               measurement.
%                 obs(:,1) - idx of pose at which measurement was 
%                   made
%                 obs(:,2) - idx of landmark being observed
%                 obs(:,3) - x-value of landmark measurement
%                 obs(:,4) - y-value of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     A       - MxN matrix
%     b       - Mx1 vector
%
function [As, b] = create_Ab_linear(odom, obs, sigma_o, sigma_l)
  % Useful Constants
  n_poses = size(odom, 1) + 1; % +1 for prior on the first pose
  n_landmarks = max(obs(:,2));

  n_odom = size(odom, 1);
  n_obs  = size(obs, 1);

  % Dimensions of state variables and measurements (all 2 in this case)
  p_dim = 2;
  l_dim = 2;
  o_dim = size(odom, 2);
  m_dim = size(obs(1, 3:end), 2);

  % A matrix is MxN, b is Mx1
  N = p_dim*n_poses    + l_dim*n_landmarks;
  M = o_dim*(n_odom+1) + m_dim*n_obs;     % +1 for prior on the first pose

  % Initialize matrices
  A = zeros(M, N);
  b = zeros(M, 1);

  % Add odometry and landmark measurements to A, b - including prior on first
  % pose

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Ai definition : rx1 ry1 rx2 ry2 ... rxN ryN ... %%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%  l1x1 l1y1 l2x1 l2y1 ... lkxN lkxM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%% Where rxi is robot x position at time i %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%% ryi is robot y position at time i %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%lkxi is x position of landmark k at time i %%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%lkyi is y position of landmark k at time i %%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  io  = 1: ( o_dim*n_odom         );
  iox = 1: ( o_dim*n_odom + o_dim );

  sigma_o =  1 / sqrt ( sigma_o ( 1 ) ); %% hack assuming symetric , diagonal matrix 
  sigma_l = 1  / sqrt ( sigma_l ( 1 ) ); %% 


  As = sparse (  io + o_dim , io  ,  -sigma_o * ones ( 1 , length ( io  ) ) , M , N ) + ... %rx/ry at t = -1
       sparse (  iox        , iox ,   sigma_o * ones ( 1 , length ( iox ) ) , M , N ); %     

  b ( o_dim + 1 : o_dim * n_odom + o_dim ) = sigma_o * odom ( 1 : o_dim*n_odom );

  for o = 0 : n_obs -1
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    i = obs ( o + 1 , 1 ); % pose i at observation o
    l = obs ( o + 1 , 2 ); % landmark at obseravation o
    x = obs ( o + 1 , 3 ); 
    y = obs ( o + 1 , 4 );
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%offset from odom and pose init. also offset for each observation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    row_off = o_dim * ( n_odom + 1 ) + l_dim * o  ;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%offset from odom and pose init. also offset for landmark id
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    col_off = o_dim * ( n_odom + 1 ) + l_dim * ( l - 1) ;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Add rxi and ryi %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    As = As + sparse (  (1:2) + row_off, (-1:0) + i*p_dim , sigma_l * [ -1 -1 ] , M , N );
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Add landmark %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    As = As + sparse (  (1:2) + row_off, (1:2) + col_off , sigma_l *  [ 1   1 ] , M , N );
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    b  ( row_off + 1 ) = x * sigma_l ;
    b  ( row_off + 2 ) = y * sigma_l ;
  end

  As = As;
end
