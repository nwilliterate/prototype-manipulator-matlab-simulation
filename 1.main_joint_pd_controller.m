% Copyright (C) 2022 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Jan, 17, 2022
% Last Updated: Jan, 18, 2022
% 
% -------------------------------------------------
% Prototype Robot
% simple joint pd controller
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%% Data load
clc; clear;
addpath(genpath('.'));

% DoF
n = 6;

% Simulation setup
sim_time = 10;
sim_period = 0.001;
t = 0.001:sim_period:sim_time;
sample_size = size(t, 2);

% LSTB trajectory generation
[s,sd,sdd] = lspb(0, 10, 0:0.001:3);
s = s.*pi/180;
sd = sd.*pi/180;
sdd = sdd.*pi/180;
for i=1:6
    q_ref(:,i) = zeros(sample_size,1);
    dq_ref(:,i) = zeros(sample_size,1);
    ddq_ref(:,i) = zeros(sample_size,1);
    q_ref(1000:4000,i) = s;
    q_ref(4001:sample_size, i) = 10/180*pi;
    dq_ref(1000:4000,i) = sd;
    ddq_ref(1000:4000,i) = sdd; 
end

% Gain
Kp = [1000 16000 8000 1000 200 0.1];
Kd = sqrt(Kp)*0.75;

% simulation
x = [zeros(n,1); zeros(n,1)];
for i=1:sample_size
    q = x(1:6,i);
    dq = x(7:12,i);

    e = q_ref(i,:)'-q;
    de = dq_ref(i,:)'-dq;
    
    pid = Kp'.*(e)+Kd'.*(de);
    u = pid;
    
    if(i ~= sample_size)
        x(:,i+1) = rk(x(:,i), u, sim_period);
    end
end

% Plot
% figure 1 : Joint Position
fig = figure(1);
tiledlayout(2,3,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
for i=1:6
    ax = nexttile;
    hold off
    plot(t, x(i,:),'-k','LineWidth',1.5')
    hold on
    plot(t, q_ref(:,i),'--b','LineWidth',1)
    grid on
    xlim([0 sim_time])
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel("q_{"+i+ "}(rad)", 'FontSize', 10);
end
saveas(gcf,'fig\sim_pd_con_result1.png');

% figure 2 : Joint Velocity
fig = figure(2);
tiledlayout(2,3,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
for i=1:6
    ax = nexttile;
    hold off
    plot(t, x(i+6,:),'-k','LineWidth',1.5')
    hold on
    plot(t, dq_ref(:,i),'--b','LineWidth',1)
    grid on
    xlim([0 sim_time])
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel("dq_{"+i+ "}(rad)", 'FontSize', 10);
end
saveas(gcf,'fig\sim_pd_con_result2.png');