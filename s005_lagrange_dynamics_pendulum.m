% Lagrange dynamics for a simple pendulum
%
% (no time derivative, instead we use lots of jacobians)


% q - angle of the pendulum
% dq - angular velocity
syms q dq real

% so the angular velocity w and angle theta is ...
w = dq;
theta = q;

r = 1;  % radius of the pendulum, length
m = 1;  % mass
g = 9.81; % gravity, guess the planet, you have 8 tries (sorry pluto)

% the only thing you need to do is set the required elements for energies
% we will use kinetic and potential due to gravity so we need:
% kinetic - v
% potential - h
v = w*r;                   % velocity on a circle
h = r*( 1 - cos(theta) );  % height, calculated from the lowermost point in the system (ground)

%-------
% this should be pretty much automatic now:

T = m*v^2 / 2; % kinetic energy, by definition
V = m*g*h;     % potential energy, by definition

L = T-V   % Lagrangian

% we will need acceleration
syms ddq real

% the partial derivative of L with regards to dq
dLdq = jacobian(L,dq)
% and the time derivative of dLdq - can be written as two jacobians
dL =  jacobian(dLdq,q)*dq + jacobian(dLdq,dq)*ddq
% last part
dLq = jacobian(L,q)

% here you go:  0 = dL-dLq    (if you have inputs it will be  Bu = dL-dLq )
dyn = dL - dLq


% note:
% if you want it robotics-style, as
%    D ddq + C = 0
% you can do:
% D = jacobian(dLdq,dq)
% C = jacobian(dLdq,q)*dq - jacobian(L,q)
