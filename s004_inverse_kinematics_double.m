% differential inverse kinematics
% for double pendulum
%
function kinematyka1

% simulation
[t,theta] = ode45( @robot, 0:0.01:15, [pi/6,-pi/6] )
% animation
for i=1:length(t)
	% calculate points for drawing based on the simulation
	[ p1,p2 ] = kinematics( theta(i,1), theta(i,2) )
	points = [ [0;0;0;1], p1, p2 ]
	plot( points(2,:), points(3,:), 'o-' )  % plot in  (y,z) axis only
	grid on
	axis( [-2 2 -2 2] )
	pause( 0.01 )
end


function dtheta = robot( t, theta )
% the control method
% just the equation   dtheta = J^-1 dp2 = J^-1 v
[p1,p2,J] = kinematics( theta(1), theta(2) )
v = [ 0.1 ; 0 ];                          % uncomment this for just going to the right
% v = [ 0.1*cos(t); -0.1*sin(t) ];          % uncomment this for a small circle
% v = [ 0.1*9*cos(9*t); -0.1*9*sin(9*t) ];  % uncomment this for a fast circle
dtheta = inv( J ) * v;


function [p1, p2, J] = kinematics( theta1 , theta2 )

u1 = [0;0;0];       % around which point you rotate?
p10 = [0;0;1;1];    % where is the initial position of your link?
w1 = [1;0;0];       % around which axis do you rotate?

u2 = [0;0;1];       %
p20 = [0;0;2;1];    %
w2 = [1;0;0];       %

s1 = [ w1 ; -cross(w1,u1) ];  % Lie algebra for describing the whole motion
s2 = [ w2 ; -cross(w2,u2) ];  %

A1 = expm( hat(s1)*theta1 );   % rotation matrix, Lie group, SE(3)
A2 = expm( hat(s2)*theta2 );   %

p1 = A1 * p10 ;        % current position
p2 = A1*A2 * p20;      %

% jacobians, taken from the derivatives of p1 and p2
% (adjoints here would be a tad overkilll...)
J = [ hat(s1)*p2, A1*hat(s2)*inv(A1)*p2 ];
J = J( 2:3, : );


function S = hat(s)

S = [ 0        -s(3)   s(2)    s(4) ; ...
      s(3)     0       -s(1)   s(5) ; ...
      -s(2)    s(1)    0       s(6) ; ...
      0        0       0       0 ];
