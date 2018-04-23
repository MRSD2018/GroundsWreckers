% SOLVE_LINEAR_SYSTEM
% 16-831 Fall 2016 - *Stub* Provided
% Solve the linear system with your method of choice
%
% Arguments: 
%     A     - A matrix from linear system that you generate yourself
%     b     - b vector from linear system that you generate yourself
%
% Returns:
%     x     - solution to the linear system, computed using your method of
%             choice
%
function x = solve_linear_system(A, b)

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % SOLVE_CHOL2
  % 16-831 Fall 2016 - *Stub* Provided
  % Solves linear system using second Cholesky method
  %
  % Arguments: 
  %     A     - A matrix from linear system that you generate yourself
  %     b     - b vector from linear system that you generate yourself
  %
  % Returns:
  %     x     - solution to the linear system, computed using the specified
  %             version of the Cholesky decomposition
  %     R     - R factor from the Cholesky decomposition
  %
  %AtA = A' * A;  %one leg short of an at-at
  %p = colamd ( AtA ); 
  %p = symrcm(AtA);
  %R = chol        ( AtA ( p , p ));
  %y = forward_sub ( R' , A ( : , p )'  * b );
  %x = back_sub    ( R  , y      );
  %x ( p ) = x;
  fprintf ( 'inside' )
  size ( A )
  size ( b )
  x = ( ( ( A' * A ) ^ -1 ) * A' )* b
end
