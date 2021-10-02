% Start solution
clc
clear

% variable for plotting
draw = 0;
n = 6; 

% max speed and accelaration for each joint  
vm = [1, 1, 1, 0.5, 0.5, 0.25];                       %rad.s^-1
am = [10, 10, 10, 5, 5, 2];                           %rad.s^-2

% define the start position q0 and the final position qf in joint space 
q1 = [0, 0, 0, 0, 0, 0];                                 % start
%q2 = [0.21, -0.1  , 0.2  , -0.15 , 0.12 , -0.21 ];       % middle
q3 = [0.31, -0.438, 0.238, -0.301, 0.212, -0.223];       % finish
q2 = 2*q3/3;

dq12 = q2 - q1;
dq23 = q3 - q2;

% compensate direction in speed and accelertion
vm12 = vm .* sign(dq12);
am12 = am .* sign(dq12);

vm23 = vm .* sign(dq23);
am23 = am .* sign(dq23);

% calculating tb, tao and tf from q1 -> q2
tb12 = vm./am;
tf12 = abs(dq12)./vm + tb12;
tao12 = tf12 - tb12;

% calculating tb, tao and tf from q2 -> q3
tb23 = vm./am;
tf23 = abs(dq23)./vm + tb23;
tao23 = tf23 - tb23;

% Synchronize motion
% Determine maximum tb and tao then calculate tf q1 -> q2
tb_syn12 = max(tb12);
tao_syn12 = max(tao12);
tf_syn12 =  tao_syn12 + tb_syn12;

disp(['Synchronize tb for q1 -> q2  = ', num2str(tb_syn12)]);
disp(['Synchronize tao for q1 -> q2 = ', num2str(tao_syn12)]);
disp(['Synchronize tf for q1 -> q2  = ', num2str(tf_syn12)]);

% Determine maximum tb and tao then calculate tf q2 -> q3
tb_syn23 = max(tb23);
tao_syn23 = max(tao23);
tf_syn23 =  tao_syn23 + tb_syn23;

disp(" ");
disp(['Synchronize tb for q2 -> q3  = ', num2str(tb_syn23)]);
disp(['Synchronize tao for q2 -> q3 = ', num2str(tao_syn23)]);
disp(['Synchronize tf for q2 -> q3  = ', num2str(tf_syn23)]);

% Recalculating the trajectory using the new time values q1->q2
vm_syn12 = dq12/tao_syn12;                       
am_syn12 = vm_syn12/tb_syn12;
% Recalculating the trajectory using the new time values q2->q3
vm_syn23 = dq23/tao_syn23;                       
am_syn23 = vm_syn23/tb_syn23;

tb  = tb_syn12*ones(1,n);   
tao = tao_syn12*ones(1,n);
tf  = tf_syn12*ones(1,n);

vm = vm_syn12;
am = am_syn12;
q0 = q1;
qf = q2;

q12   = [];
qp12  = [];
qpp12 = [];
idx = 1;
% apply trajectory eqation for each time span (acc,cont_speed,deacc)
% calculate trajectory for q1 but finish at tf - tb/2
for t = linspace(0,max(tf - tb_syn12/2),100)
    for i = 1:n
        if (t >= 0 && t <= tb(i))
            q12(i, idx)   = q0(i) + 0.5*am(i)*t^2;
            qp12(i, idx)  = am(i)*t;
            qpp12(i, idx) = am(i);
        elseif( t > tb(i) && t <= tao(i))
            q12(i, idx)   = q0(i) + 0.5*am(i)*tb(i)^2 + vm(i)*(t - tb(i));
            qp12(i, idx)  = vm(i);
            qpp12(i, idx) = 0;
        elseif(t > tao(i) && t < tf(i))
            q12(i, idx)   = qf(i) - 0.5*am(i)*(t - tf(i))^2;
            qp12(i, idx)  = -am(i)*(t - tf(i));
            qpp12(i, idx) = -am(i);
        end
    end
    idx = idx + 1;
end

t12 = linspace(0,max(tf - tb_syn12/2),100);
% delete the last point to merge intersection
t12(end) = [];

% calculate shift amount
shift = max(tf - tb_syn12/2);

% recalculate for q2
tb  = tb_syn23*ones(1,n);   
tao = tao_syn23*ones(1,n);
tf  = tf_syn23*ones(1,n);
vm = vm_syn23;
am = am_syn23;
q0 = q2;
qf = q3;


q23   = [];
qp23  = [];
qpp23 = [];
idx = 1;
% apply trajectory eqation for each time span (acc,cont_speed,deacc)
% I add 0.1 to make plot more clear it will only effect visiual

for t = linspace(tb_syn12/2,max(tf),100)
    for i = 1:n
        if (t >= 0 && t <= tb(i))
            q23(i, idx)   = q0(i) + 0.5*am(i)*t^2;
            qp23(i, idx)  = am(i)*t;
            qpp23(i, idx) = am(i);
        elseif( t > tb(i) && t <= tao(i))
            q23(i, idx)   = q0(i) + 0.5*am(i)*tb(i)^2 + vm(i)*(t - tb(i));
            qp23(i, idx)  = vm(i);
            qpp23(i, idx) = 0;
        elseif(t > tao(i) && t < tf(i))
            q23(i, idx)   = qf(i) - 0.5*am(i)*(t - tf(i))^2;
            qp23(i, idx)  = -am(i)*(t - tf(i));
            qpp23(i, idx) = -am(i);
        end
    end
    idx = idx + 1;
end

% merge the result
q = [q12, q23];
qp = [qp12, qp23];
qpp = [qpp12, qpp23];
% ploting if draw is not 0 or any false result 
t23 = linspace(tb_syn12/2,max(tf),100);
t23 = t23 + shift - tb_syn12/2;
t = [t12, t23];
size(q)
size(t)

% ploting the results
figure('units','normalized','outerposition',[0 0 1 1])
hold all
for i = 0:n-1
    subplot(6, 3, 3*i + 1)
    plot(t, q(i+1,:), 'k-')
    ylim([(qf(i+1) - 0.2)*(qf(i+1) <= 0), (qf(i+1) + 0.2)*(qf(i+1) > 0)]);

    grid on

    legend('q', 'Location', 'best')

    title(['j', num2str(i+1), ' position curve.'])
    xlabel('time [sec])')
    ylabel('Position [deg]')

    subplot(6, 3, 3*i + 2)
    plot(t, qp(i+1,:), 'b-')

    grid on


    legend('qp', 'Location', 'best')

    title(['j', num2str(i+1), ' speed curve.'])
    xlabel('time [sec])')
    ylabel('speed [deg.sec]')

    subplot(6, 3, 3*i + 3)
    plot(t, qpp(i+1,:), 'g-')

    grid on


    legend('qpp', 'Location', 'best')

    title(['j', num2str(i+1), ' acceleration curve.'])
    xlabel('time [sec])')
    ylabel('acceleration [deg.sec^2]')
end