function dxdt = plant(x,u)
q = x(1:6);
qd = x(7:12);

g = get_GravityVector(q)';
C = get_CoriolisMaxtrix(q,qd);
M = get_MassMaxtrix(q);

u(5) = -u(5);
temp_dt = inv(M)*(u-g-C*qd);
dxdt = [qd; temp_dt];
end