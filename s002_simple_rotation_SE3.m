%
%  simple rotation, using functions
%

function s002_simple_rotation_SE3
% make an animation
for theta = 0:0.01:pi/2
    p1 = kinematics( theta )
    plot( [0 p1(2)], [0 p1(3)] )  % plot in x-z axis
    axis( [-2 2 -2 2] )           % proper scale
    pause( 0.01 )
end
 
function [p1] = kinematics( theta1 )
% calculate the kinematics based on angle theta1
u1 = [0;0;0];       % point around which we rotate
p10 = [0;0;1;1];    % point where we start the motion
w1 = [1;0;0];       % direction of rotation

s1 = [ w1 ; -cross(w1,u1) ];  % Lie algebra element describing the movement
A1 = expm( hat(s1)*theta1 );   % rotation-translation matrix, in Lie group
p1 = A1 * p10 ;        % the current position


function S = hat(s)
% this is essentially a matrix version of a vector product
% but since it will have to work in se(3):
% by definition: S = [ hat(w) v ; 0 0 0 0 ]
% where w in so(3) and hat(w) returns a screw-symmetric matrix
S = [ 0        -s(3)   s(2)    s(4) ; ...
      s(3)     0       -s(1)   s(5) ; ...
      -s(2)    s(1)    0       s(6) ; ...
      0        0       0       0 ];

