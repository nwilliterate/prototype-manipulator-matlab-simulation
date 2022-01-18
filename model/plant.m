% Copyright (C) 2022 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:        Jan, 18, 2022
% 
% -------------------------------------------------
% Prototype Robot Plant Function
% -------------------------------------------------
% Equation)
%       ddq = M(q)ddq + c(q, dq)dq + g(q)
%
% Input)
%       x   : Joint Postion and Velocity
%       u   : Joint Torque
% Output)
%       dxdt  : Joint Velocity and Acceleration
%
% the following code has been tested on Matlab 2021a
function dxdt = plant(x,u)
q = x(1:6);
qd = x(7:12);

g = get_GravityVector(q)';
C = get_CoriolisMaxtrix(q,qd);
M = get_MassMaxtrix(q);

% In prototype robot model(M, C, G), Joint 5 is the opposite.
u(5) = -u(5);
temp_dt = inv(M)*(u-C*qd-g);
dxdt = [qd; temp_dt];
end