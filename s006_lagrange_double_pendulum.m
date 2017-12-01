% Lagrange dynamics for a double pendulum
%
% with SE(3) description of rotations
%

% angles: q1 q2,  angular velocities: dq1 dq2
syms q1 dq1 q2 dq2 real
% vectors will be more useful:
q = [q1;q2];
dq = [dq1;dq2];

u1 = [0;0;0];     % point on the axis of rotation, around which we rotate
p10 = [0;0;1;1];  % initial position of the robot
w1 = [1;0;0];     % axis of rotation around which we rotate

u2 = [0;0;1];
p20 = [0;0;2;1];
w2 = [1;0;0];

s1 = [ w1 ; -cross(w1,u1) ];   % Lie algebra element, describes the movement
s2 = [ w2 ; -cross(w2,u2) ]; 

A1 = simplify(expm( hat(s1)*q1 ));  % rotation matrix
A2 = expm( hat(s2)*q2 );

p1 = simplify(A1 * p10 )   % current position
p2 = simplify(A1*A2 * p20)
dp1 = hat(s1)*dq1 * A1 * p10;   % current velocity (derivative of e^X(t) btw)
dp2 = hat(s1)*dq1 * A1*A2* p20 + A1 * hat(s2)*dq2 * A2 * p20;

m = 1;     % mass
g = 9.81;  % gravity


% the only things we need for energies are:
% for kinetic - velocity v
% for potential - height ... z axis coords of all points?

v1_2 = dp1(1)^2 + dp1(2)^2 + dp1(3)^2  % this is really v1^2
v2_2 = dp2(1)^2 + dp2(2)^2 + dp2(3)^2  % this is really v2^2

T = m*v1_2/2 + m*v2_2/2;   % kinetic energy, by definition
V = m*g*p1(3) + m*g*p2(3); % potential energy, by definition
L = T-V   % Lagrangian

% accelerations
syms ddq1 ddq2 real
ddq = [ddq1;ddq2];

% first part:  partial derivative of L by dq
dLdq = simplify( jacobian(L,dq) )
% and now the time derivative d/dt: (we do it using splitting into partial derivatives, easier)
dL = simplify( jacobian(dLdq,q)*dq + jacobian(dLdq,dq)*ddq )
% the rest:
dLq = simplify(jacobian(L,q) )'

% dynamics equation :   0 = dL-dLq
dyn = dL - dLq

% for simulations generally a robotics style is better:
%  D ddq + C = 0
D = jacobian(dLdq,dq);   % mass-inertia matrix, this is simply a double-jacobian of kinetics energy
C = jacobian(dLdq,q)*dq - jacobian(L,q)   % Coriolis and gravity together

% nobody will substitute symbolic expression, we don't have processor time for that
% dump D and C into files matrixD.m and matrixC.m that we can execute:
matlabFunction( D, 'file', 'matrixD', 'vars', theta ); % mass-inertia is based ONLY on positions q
matlabFunction( C, 'file', 'matrixC', 'vars', [ theta ; dtheta ] );


% you can then start it using ode45 as:
% D = matrixD( theta1, theta2 );
% C = matrixC( theta1, theta2, dtheta1, dtheta2 );
% ddq = -inv(D)*C
