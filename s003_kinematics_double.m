% symbolic calculation of double pendulum kinematics
%

% declare the angles:
syms theta1 theta2 real

% hat operator for se(3)
hat = @(s) [ 0        -s(3)   s(2)    s(4) ; ...
             s(3)     0       -s(1)   s(5) ; ...
             -s(2)    s(1)    0       s(6) ; ...
             0        0       0       0 ];

u1 = [0;0;0];       % point around which link 1 rotates
p10 = [0;0;1;1];    % point where link 1 starts its movement
w1 = [1;0;0];       % direction of rotation (around which axis?)

u2 = [0;0;1];       % same as above, for link 2
p20 = [0;0;2;1];    %
w2 = [1;0;0];       %

s1 = [ w1 ; -cross(w1,u1) ];  % Lie algebra for link 1
s2 = [ w2 ; -cross(w2,u2) ];  % for link 2

A1 = expm(hat(s1)*theta1);   % rotation-translation matrix, SE(3), for link 1
A2 = expm(hat(s2)*theta2);   % for link 2

p1 = A1 * p10 ;        % current position of point 1
p2 = A1*A2 * p20;      % current position of point 2

p_all = [ [0;0;0;1] p1 p2 ];  % all points we wish to draw, together with (0,0,0)

% we will animate it in 0-90 degrees
for theta = 0:0.01:pi/2
    theta1 = theta;                        % both angles will move the same, just bend
    theta2 = theta;
    p  =  subs(p_all);                     % subs() substitutes theta1 and theta2 in p_all as numbers
    plot3( p(1,:), p(2,:), p(3,:), 'o-' )  % plot x-axis of all points, y-axis ... and so on
    axis( [-2 2 -2 2 -2 2] )               % correct scale
    view(90,0);                            % camera! we are by default in 3D, even if this is a planar manipulator
    pause( 0.01 )
end