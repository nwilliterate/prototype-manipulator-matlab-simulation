% Copyright (C) 1993-2017, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% LSPB  Linear segment with parabolic blend
% 
% https://github.com/petercorke/robotics-toolbox-matlab/blob/master/lspb.m
%
function [s,sd,sdd] = lspb(q0, q1, t, V)

    t0 = t;
    if isscalar(t)
        t = (0:t-1)';
    else
        t = t(:);
    end
    plotsargs = {'Markersize', 16};

    tf = max(t(:));

    if nargin < 4
        % if velocity not specified, compute it
        V = (q1-q0)/tf * 1.5;
    else
        V = abs(V) * sign(q1-q0);
        if abs(V) < abs(q1-q0)/tf
            error('V too small');
        elseif abs(V) > 2*abs(q1-q0)/tf
            error('V too big');
        end
    end

    if q0 == q1
        s = ones(size(t)) * q0;
        sd = zeros(size(t));
        sdd = zeros(size(t));
        return
    end

    tb = (q0 - q1 + V*tf)/V;
    a = V/tb;

    p = zeros(length(t), 1);
    pd = p;
    pdd = p;
    
    for i = 1:length(t)
        tt = t(i);

        if tt <= tb
            % initial blend
            p(i) = q0 + a/2*tt^2;
            pd(i) = a*tt;
            pdd(i) = a;
        elseif tt <= (tf-tb)
            % linear motion
            p(i) = (q1+q0-V*tf)/2 + V*tt;
            pd(i) = V;
            pdd(i) = 0;
        else
            % final blend
            p(i) = q1 - a/2*tf^2 + a*tf*tt - a/2*tt^2;
            pd(i) = a*tf - a*tt;
            pdd(i) = -a;
        end
    end

    switch nargout
        case 0
            if isscalar(t0)
                % for scalar time steps, axis is labeled 1 .. M
                xt = t+1;
            else
                % for vector time steps, axis is labeled by vector M
                xt = t;
            end

            clf
            subplot(311)
            % highlight the accel, coast, decel phases with different
            % colored markers
            hold on
            %plot(xt, p);
            k = t<= tb;
            plot(xt(k), p(k), 'r.-', plotsargs{:});
            k = (t>=tb) & (t<= (tf-tb));
            plot(xt(k), p(k), 'b.-', plotsargs{:});
            k = t>= (tf-tb);
            plot(xt(k), p(k), 'g.-', plotsargs{:});
            grid; ylabel('$s$', 'FontSize', 16, 'Interpreter','latex');

            hold off

            subplot(312)
            plot(xt, pd, '.-', plotsargs{:});
            grid;
            if isscalar(t0)
                ylabel('$ds/dk$', 'FontSize', 16, 'Interpreter','latex');
            else
                ylabel('$ds/dt$', 'FontSize', 16, 'Interpreter','latex');
            end
            
            subplot(313)
            plot(xt, pdd, '.-', plotsargs{:});
            grid;
            if isscalar(t0)
                ylabel('$ds^2/dk^2$', 'FontSize', 16, 'Interpreter','latex');
            else
                ylabel('$ds^2/dt^2$', 'FontSize', 16, 'Interpreter','latex');
            end
            
            if ~isscalar(t0)
                xlabel('t (seconds)')
            else
                xlabel('k (step)');
                for c=findobj(gcf, 'Type', 'axes')
                    set(c, 'XLim', [1 t0]);
                end
            end
            shg
        case 1
            s = p;
        case 2
            s = p;
            sd = pd;
        case 3
            s = p;
            sd = pd;
            sdd = pdd;
    end