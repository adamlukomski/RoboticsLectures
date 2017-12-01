%
% simple rotation
%
% this is essentially a solution of v = w x r
% 

hat = @(s) [ 0     -s(3)  s(2)   ; ...
             s(3)  0      -s(1)  ; ...
             -s(2) s(1)   0      ]; 

p0 = [0;0;1];      % initial position    (where are you at t=0?)
w = [1;0;0];       % direction of motion (around which axis do you rotate?)

% quick animation - go from 0 to 90 degrees
for theta = 0:0.01:pi/2
    p = expm( hat(w) * theta ) * p0   % current position = rotation matrix * initial position
    plot( [0 p(2)], [0 p(3)] )        % the plot is in (y,z) axis, from (0,0) to p
    axis( [-1 1 -1 1] )               % scale it so we can see anything
    pause( 0.01 )                     % pause to show the frame
end
