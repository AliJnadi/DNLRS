% Start solution
clc

% variable for plotting
draw = 0;

% DOF
n = 6;

% define the start position q0 and the final position qf in joint space 
q0 = [0, 0, 0, 0, 0, 0];                              % zero configuration position -> start
qf = [0.31, -0.438, 0.238, -0.301, 0.212, -0.223];    % final position
deltaq = qf - q0; 

% max speed and accelaration for each joint  
vm = [1, 1, 1, 0.5, 0.5, 0.25];                       %rad.s^-1
am = [10, 10, 10, 5, 5, 2];                           %rad.s^-2

% calculating tb, tao and tf for each joint
tb = vm./am;
tf = abs(deltaq)./vm + tb;
tao = tf - tb;

% collect everything in one matrix and display result
res = array2table([tb', tao', tf'], 'VariableNames',{'tb','tao','tf'});
fprintf('Trajectory time points:\n\n');
disp(res)

% I put all the trajectory calculation in one function if the draw variable
% is 1 the function will plot trajectory and I am compensating the
% directsion of moving by taking the sign of the delataq
vm = vm .* sign(deltaq);
am = am .* sign(deltaq);
traj_calc(q0, qf, tb, tf, tao, vm, am, vm, am, n, draw, 'Normal control');

% Synchronize motion
% Determine maximum tb and tao then calculate tf
tb_syn = max(tb);
tao_syn = max(tao);
tf_syn =  tao_syn + tb_syn;

disp(['Synchronize tb  = ', num2str(tb_syn)]);
disp(['Synchronize tao = ', num2str(tao_syn)]);
disp(['Synchronize tf  = ', num2str(tf_syn)]);

% Recalculating the trajectory using the new time values
vm_syn = deltaq/tao_syn;                       
am_syn = vm_syn/tb_syn;

% Display results for comparison
res = array2table([vm_syn', am_syn', vm', am']...
                  , 'VariableNames',{'synchronise maximum speed'...
                  , 'synchronise maximum acceleration'...
                  , 'maximum speed' , 'maximum acceleration'});
fprintf('\nSynchronize speed and acceleration:\n\n');
disp(res)

% Recalculate trajectories
traj_calc(q0, qf, tb_syn, tf_syn, tao_syn, vm_syn, am_syn, vm, am, n, draw, 'Sychronise control');

% numerical control solution
dt = 0.1;

% flooring dt to the nearist tenth 
num = 0;
while (floor(dt*10^num)~=dt*10^num)
    num=num+1;
end
E = 1*10^-num;

% calculating numerical time points to be identical to sample multiplication  
if( rem(tb_syn, dt) ~= 0)
    tb_num = round(tb_syn, num) + E;
else
    tb_num = round(tb_syn, num);
end

if( rem(tao_syn, dt) ~= 0)
    tao_num = round(tao_syn, num) + E;
else
    tao_num = round(tao_syn, num);
end

tf_num = tao_num + tb_num;


% Display results for comparison
res = array2table([tb_num, tb_syn; tao_num, tao_syn; tf_num, tf_syn]...
                  , 'VariableNames',{'numerical values'...
                  , 'normal values'}...
                  , 'RowNames',{'tb', 'tao', 'tf'});
fprintf('\nNumerical and normal control time values comparison:\n\n');
disp(res);

% Recalculating the trajectory using the new time values
vm_num = deltaq/tao_num;                     
am_num = vm_num/tb_num;

% Display results for comparison
res = array2table([vm_num', am_num', vm_syn', am_syn']...
                  , 'VariableNames',{'numerical maximum speed' , 'numerical maximum acceleration'...
                  , 'synchronise maximum speed'...
                  , 'synchronise maximum acceleration'});
fprintf('\nSynchronize speed and acceleration:\n\n');
disp(res);

% ploting trajectories
traj_calc(q0, qf, tb_num, tf_num, tao_num, vm_num, am_num, vm, am, n, draw, 'Numirical Synchronise control');

% Calculate propagated error in end-effector position
% I will make a new for loop with change rate equal to dt then I will 
% calculate qi with respect to numerical control and without
% after that using the FDM I will calculate the EF position and caculate
% error
q_num = [];
q_nor = [];
p_num = [];
p_nor = [];
idx = 1;
for t = 0 : dt : tf_num
    for i = 1:n
        if (t >= 0 && t < tb_num + eps)
            q_num(i, idx)   = q0(i) + 0.5*am_num(i)*t^2;
        elseif( t > tb_num && t < tao_num  + eps)
            q_num(i, idx)   = q0(i) + 0.5*am_num(i)*tb_num^2 + vm_num(i)*(t - tb_num);
        elseif(t > tao_num && t < tf_num  + eps)
            q_num(i, idx)   = qf(i) - 0.5*am_num(i)*(t - tf_num)^2;
        else
            q_num(i, idx)   = q_num(i, idx-1);
        end
    end
    [P, ~] = DKM(q_num(1,idx), q_num(2,idx), q_num(3,idx), q_num(4,idx), q_num(5,idx), q_num(6,idx));
    p_num(idx,:) = P;
    
    for i = 1:n
        if (t >= 0 && t < tb_syn  + eps)
            q_nor(i, idx)   = q0(i) + 0.5*am_syn(i)*t^2;
        elseif( t > tb_syn && t < tao_syn  + eps)
            q_nor(i, idx)   = q0(i) + 0.5*am_syn(i)*tb_syn^2 + vm_syn(i)*(t - tb_syn);
        elseif(t > tao_syn && t < tf_syn  + eps)
            q_nor(i, idx)   = qf(i) - 0.5*am_syn(i)*(t - tf_syn)^2;
        else
            q_nor(i, idx)   = q_num(i, idx-1);
        end
    end
    [P, ~] = DKM(q_nor(1,idx), q_nor(2,idx), q_nor(3,idx), q_nor(4,idx), q_nor(5,idx), q_nor(6,idx));
    p_nor(idx,:) = P;
    idx = idx + 1;
end
t = 0 : dt : tf_num;
plot3(p_num(:,1),p_num(:,2),p_num(:,3), 'g-', p_nor(:,1),p_nor(:,2),p_nor(:,3), 'r--')
legend('Numerical control','Normal Control with digital processor')

[pf, ~] = DKM(qf(1), qf(2), qf(3), qf(4), qf(5), qf(6));

% Display results for comparison
res = array2table([p_num(end, :)', p_nor(end, :)', pf, abs(pf - p_num(end, :)')]...
                  , 'VariableNames',{'Position with numerical control'...
                  , 'Position without taking sampling into account'...
                  , 'Final position', 'Error'}...
                  , 'RowNames',{'EFx', 'EFy', 'EFz'});
fprintf('\nNumiric vs Normal comparison:\n\n');
disp(res);