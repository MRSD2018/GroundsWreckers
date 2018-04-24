% CREATE_AB_NONLINEAR
% 16-831 Fall 2016 - *Stub* Provided
% Computes the A and b matrices for the 2D nonlinear SLAM problem
%
% Arguments: 
%     x       - Current estimate of the state vector
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
function [As, b] = create_Ab_nonlinear(x, odom, obs, sigma_o, sigma_l)
  %% Extract useful constants which you may wish to use
  n_poses =     size ( odom, 1 ) + 1; % +1 for prior on the first pose
  n_landmarks = max  ( obs( : , 2 ) );

  n_odom = size(odom, 1);
  n_obs  = size(obs, 1);

  % Dimensions of state variables and measurements (all 2 in this case)
  p_dim = 3;                                  % pose dimension
  l_dim = 2;                                  % landmark dimension
  o_dim = size(odom, 2);                      % odometry dimension
  m_dim = size(obs(1, 3:end), 2);             % landmark measurement dimension

  % A matrix is MxN, b is Mx1
  N = p_dim *   n_poses + l_dim * n_landmarks;
  M = p_dim * ( n_odom  + 1 ) + m_dim * n_obs; % +1 for prior on the first pose

  %% Initialize matrices
  A = zeros(M, N);
  b = zeros(M, 1);

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Ai definition : rx1 ry1 rx2 ry2 ... rxN ryN ... %%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%  l1x1 l1y1 l2x1 l2y1 ... lkxN lkxM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%% Where rxi is robot x position at time i %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%% ryi is robot y position at time i %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%lkxi is x position of landmark k at time i %%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%lkyi is y position of landmark k at time i %%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Formulate odom section of A ( Same as linear example ) %%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  io  = 1: ( o_dim*n_odom         );
  iox = 1: ( o_dim*n_odom + o_dim );

  sigma_o =  1 / sqrt ( sigma_o ( 1 ) ); %% hack assuming symetric , diagonal matrix 
  sigma_l = 1  / sqrt ( sigma_l ( 1 ) ); %% 


  As = sparse (  io + o_dim , io  ,  -sigma_o * ones ( 1 , length ( io  ) ) , M , N ) + ... %rx/ry at t = -1
       sparse (  iox        , iox ,   sigma_o * ones ( 1 , length ( iox ) ) , M , N ); %     

  
  %b ( o_dim + 1 : o_dim * n_odom + o_dim ) = sigma_o * odom ( 1 : o_dim*n_odom );

  %for u = ( 1 : n_odom ) * 2
  for u = ( 1 : n_odom )
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Predict measurement %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ixu = p_dim*u;
    rx1 = x ( ixu - 1 );
    ry1 = x ( ixu + 0 );
    rt1 = x ( ixu + 1 );
    rx2 = x ( ixu + 2 );
    ry2 = x ( ixu + 3 );
    rt2 = x ( ixu + 4 );
    
    h   = meas_odom(rx1, ry1, rt1, rx2, ry2, rt2);
    dxp = h ( 1 );
    dyp = h ( 2 );
    dtp = h ( 3 );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% slice measurement %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %TODO: CHECK IF ODOM INPUT IS ORDERED VL; VR or VR; VL
    vl =  odom ( u  , 1 );
    vr =  odom ( u  , 2 );

    L = .508; %Wheelbase of GroundsBot
    odom_z = meas_odom_z(vl, vr, rt2);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% set b %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    b ( ixu +1 ) = sigma_o * ( odom_z(1) - dxp ) ;
    b ( ixu +2 ) = sigma_o * ( odom_z(2) - dyp ) ;
    b ( ixu +3 ) = sigma_o * ( odom_z(3) - dtp);
  end

  for o = 0 : n_obs -1
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Extract values from observation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    %TODO: CHECK TO SEE IF INDEX TO GET POSE AND OBSERVATION ID IS CORRECT
    i   = obs ( o + 1 , 1 ); % pose i at observation o
    l   = obs ( o + 1 , 2 ); % landmark at obseravation o
    
    DX = obs ( o + 1 , 3 ); % delta x to landmark
    DY  = obs ( o + 1 , 4 ); % delta y to landmark

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Define matrix offsets %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    obs_offset  = o_dim * ( n_odom + 1 ) + l_dim * o        ;
    lm_off      = o_dim * ( n_odom + 1 ) + l_dim * ( l - 1) ;
    pos_off = p_dim * ( i      - 1 )                    ;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Compute jacobian subsection %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rx = x ( pos_off     + 1 );
    ry = x ( pos_off     + 2 );
    rt = x ( pos_off     + 3 );
    lx = x ( lm_off + 1 );
    ly = x ( lm_off + 2 );
    H_sub = meas_landmark_jacobian( rx , ry , rt, lx , ly );


    dDX_drx = H_sub ( 1 , 1 );
    dDX_dry = H_sub ( 1 , 2 );
    dDX_drt = H_sub( 1 , 3 );
    dDX_dlx = H_sub ( 1 , 4 );
    dDX_dly = H_sub ( 1 , 5 );
    dDY_drx = H_sub ( 2 , 1 );
    dDY_dry = H_sub ( 2 , 2 );
    dDY_drt = H_sub ( 2 , 3 );
    dDY_dlx = H_sub ( 2 , 4 );
    dDY_dly = H_sub ( 2 , 5 ); 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Predict measurement based on x %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    h = meas_landmark( rx , ry , rt, lx , ly );
    DX_p = h ( 1 );
    DY_p = h ( 2 );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Add landmark jacobian info to As %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    As = As + sparse (  obs_offset + 1 , pos_off + 1 , sigma_l * dDX_drx, M,N );
    As = As + sparse (  obs_offset + 1 , pos_off + 2 , sigma_l * dDX_dry, M,N );
    As = As + sparse (  obs_offset + 1 , pos_off + 3 , sigma_l * dDX_drt, M,N );
    As = As + sparse (  obs_offset + 1 , lm_off  + 1 , sigma_l * dDX_dlx, M,N );
    As = As + sparse (  obs_offset + 1 , lm_off  + 2 , sigma_l * dDX_dly, M,N );

    As = As + sparse (  obs_offset + 2 , pos_off + 1 , sigma_l * dDY_drx , M,N );
    As = As + sparse (  obs_offset + 2 , pos_off + 2 , sigma_l * dDY_dry , M,N );
    As = As + sparse (  obs_offset + 2 , pos_off + 3 , sigma_l * dDY_drt , M,N );
    As = As + sparse (  obs_offset + 2 , lm_off  + 1 , sigma_l * dDY_dlx , M,N );
    As = As + sparse (  obs_offset + 2 , lm_off  + 2 , sigma_l * dDY_dly , M,N );


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Formulate b ( error vector ) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    b ( obs_offset + 1 ) = sigma_l * ( DX - DX_p );
    b ( obs_offset + 2 ) = sigma_l * ( DY - DY_p );
  end

end


