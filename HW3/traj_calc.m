function [t, q, qp, qpp] = traj_calc(q0, qf, tb, tf, tao, vm, am, vref, aref, n, draw, fig_title)
% generate time points and calculate position, speed and acceleration
% q0        : start point
% qf        : finish point
% tb        : rise time
% tf        : overall time
% tao       : constant speed interval
% vm        : maximum speed
% am        : maximum acceleration
% vref      : maximum speed reference for ploting purpose
% aref      : maximum accelertion reference for ploting purpose
% n         : degeree of freedom
% draw      : plot condition (1 or 0)
% fig_title : title of figure window

q   = [];
qp  = [];
qpp = [];
idx = 1;

% solving non vector input
if(length(tb) == 1)
   tb  = tb*ones(1,n);
   tao = tao*ones(1,n);
   tf  = tf*ones(1,n);
end

% apply trajectory eqation for each time span (acc,cont_speed,deacc)
% I add eps because comparison a float point in matlab some times seems
% tricky

for t = linspace(0, max(tf), 100)
    for i = 1:n
        if (t >= 0 && t < tb(i) + eps)
            q(i, idx)   = q0(i) + 0.5*am(i)*t^2;
            qp(i, idx)  = am(i)*t;
            qpp(i, idx) = am(i);
        elseif( t > tb(i) && t < tao(i) + eps)
            q(i, idx)   = q0(i) + 0.5*am(i)*tb(i)^2 + vm(i)*(t - tb(i));
            qp(i, idx)  = vm(i);
            qpp(i, idx) = 0;
        elseif(t > tao(i) + eps && t < tf(i) + eps)
            q(i, idx)   = qf(i) - 0.5*am(i)*(t - tf(i))^2;
            qp(i, idx)  = -am(i)*(t - tf(i));
            qpp(i, idx) = -am(i);
        else
            q(i, idx)   = q(i, idx-1);
            qp(i, idx)  = 0;
            qpp(i, idx) = 0;
        end
    end
    idx = idx + 1;
end

% ploting if draw is not 0 or any false result 
t = linspace(0, max(tf), 100);
if(draw)
    % ploting the results
    figure('Name', fig_title,'units','normalized','outerposition',[0 0 1 1])
    hold all
    for i = 0:n-1
        subplot(6, 3, 3*i + 1)
        plot(t, q(i+1,:), 'k-')
        ylim([(qf(i+1) - 0.2)*(qf(i+1) <= 0), (qf(i+1) + 0.2)*(qf(i+1) > 0)]);
        
        grid on
        axis auto
        refl = refline(0, qf(i+1));
        refl.Color = 'r';
        refl.LineStyle = '--';
        
        legend('q', 'final position', 'Location', 'best')
        xlim('auto');
        
        title(['j', num2str(i+1), ' position curve.'])
        xlabel('time [sec])')
        ylabel('Position [deg]')

        subplot(6, 3, 3*i + 2)
        plot(t, qp(i+1,:), 'b-')
        ylim([(vref(i+1) - 0.5)*(vref(i+1) <= 0), (vref(i+1) + 0.5)*(vref(i+1) > 0)]);
        
        grid on
      
        refl = refline(0, vref(i+1));
        refl.Color = 'r';
        refl.LineStyle = '--';
        
        legend('qp', 'max velocity', 'Location', 'best')
        xlim('auto');
        
        title(['j', num2str(i+1), ' speed curve.'])
        xlabel('time [sec])')
        ylabel('speed [deg.sec]')

        subplot(6, 3, 3*i + 3)
        plot(t, qpp(i+1,:), 'g-')
        ylim([-abs(aref(i+1)) - 1, abs(aref(i+1)) + 1])
        
        grid on
        
        refl = refline(0, aref(i+1));
        refl.Color = 'r';
        refl.LineStyle = '--';
        refl = refline(0, -aref(i+1));
        refl.Color = 'r';
        refl.LineStyle = '--';
        
        legend('qpp', 'max acceleration', 'Location', 'best')
        xlim('auto');
        
        title(['j', num2str(i+1), ' acceleration curve.'])
        xlabel('time [sec])')
        ylabel('acceleration [deg.sec^2]')
    end
end
end