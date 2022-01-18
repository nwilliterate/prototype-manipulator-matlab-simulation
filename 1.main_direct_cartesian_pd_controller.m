% Copyright (C) 2022 All rights reserved.
% Authors:      Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Jan, 17, 2022
% Last Updated: Jan, 18, 2022
%
% -------------------------------------------------
% Prototype Robot 
% driect cartesian pd controller
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%% Data load
clc; clear;
addpath(genpath('.'));

% DoF
n = 6;

% get a initial pose
init_q = [0 -20 30 40 10 10]/180*pi;
[init_p, R]=get_pose(init_q');

% Simulation setup
sim_time = 10;
sim_period = 0.001;
t = 0:sim_period:sim_time;
sample_size = size(t, 2);

% refer trajectory
ref_x(:, 1:3) = [init_p(1)+zeros(sample_size,1), init_p(2)+sin(t)'*0.2, init_p(3)+cos(t)'*0.2-0.2];
x = [init_q'; zeros(6,1)];

% Gain
K = diag([200000 200000 200000]);
D = diag([800 800 800]);

dx = zeros(3,1);

% Simulation
for i=1:sample_size
    q = x(1:6,i);
    dq = x(7:12,i);
    
    [p, R]=get_pose(q);
    J= get_JacobianPos(q);
    cur_p(i,:) = p;
    if( i~= 1)
        dx(:, i) = (cur_p(i,:) - cur_p(i-1,:))/0.001;
    end
    
    g = get_GravityVector(q);
    
    e = ref_x(i,1:3)'-p;
    de = -dx(:, i);
    
    pid = J'*(K * e  + D *de);
    u = pid + g';
    
    if(i ~= sample_size)
        x(:,i+1) = rk(x(:,i), u, sim_period);
    end
end

%
figure(1)
tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
plot3(ref_x(:,1),ref_x(:,2),ref_x(:,3), '-','LineWidth',1,'Color','k');
hold on
plot3(cur_p(:,1), cur_p(:,2), cur_p(:,3), '-','LineWidth',1,'Color','r');
hold off
grid on;
ax = gca;
r = 0.25;
axis([ax.XLim(1)-r ax.XLim(2)+r ax.YLim(1)-r ax.YLim(2)+r ax.ZLim(1)-r ax.ZLim(2)+r])
xlabel('P_x(m)','FontSize', 12);
ylabel('P_y(m)','FontSize', 12);
zlabel('P_z(m)','FontSize', 12);
legend('kin.','CoM', 'Location', 'northwest');
saveas(gcf,'fig\car_pd_con_result1.png');

% figure 2 : Joint Position
fig = figure(2);
tiledlayout(2,3,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
for i=1:6
    ax = nexttile;
    hold off
    plot(t, x(i,:),'-k','LineWidth',1.5')
    hold on
%     plot(t, q_ref(:,i),'--b','LineWidth',1)
    grid on
    xlim([0 sim_time])
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel("q_{"+i+ "}(rad)", 'FontSize', 10);
end
saveas(gcf,'fig\car_pd_con_result2.png');