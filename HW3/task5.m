% solving the polynomial
% Start solution
clc
clear

% defining the three points and merge them in one matrix
q1 = [0   ,  0    , 0    ,  0    , 0    ,  0    ];       % start
q2 = [0.21, -0.1  , 0.2  , -0.15 , 0.12 , -0.21 ];       % middle
q3 = [0.31, -0.438, 0.238, -0.301, 0.212, -0.223];       % finish
q = [q1', q2', q3'];


% constants
n = 6;

% max speed and accelaration for each joint  
vm = [1, 1, 1, 0.5, 0.5, 0.25]';                       %rad.s^-1
am = [10, 10, 10, 5, 5, 2]';                           %rad.s^-2

% calculating deltaq for each pair 
deltaq = zeros(n, size(q,2) - 1);
for i = 1 : size(q,2) - 1
    deltaq(:, i) = q(:, i + 1) - q(:, i);
end

% calculating tf for each pair
tf = abs(deltaq)./vm + vm./am;

% calcute the maximum tf for each pair I want to synchronise
tf_syn = [0, max(tf)];

% calculate coeffecints be solving Ax = B
pc = zeros(n,6);

qt = [];
qp = [];
qpp = [];

for j = 1 : n
    idx = 1;
    for i = 1 : length(tf_syn) - 1
        t0 = tf_syn(i);
        tf = tf_syn(i + 1) + t0;
        q0 = q(j, i);
        qf = q(j, i+1);
   
        A = [t0^5   , t0^4   , t0^3  , t0^2, t0, 1;
             tf^5   , tf^4   , tf^3  , tf^2, tf, 1;
             5*t0^4 , 4*t0^3 , 3*t0^2, 2*t0, 1 , 0;
             5*tf^4 , 4*tf^3 , 3*tf^2, 2*tf, 1 , 0;
             20*t0^3, 12*t0^2, 6*t0  , 2   , 0 , 0;
             20*tf^3, 12*tf^2, 6*tf  , 2   , 0 , 0];
         
        B = [q0, qf, 0, 0, 0, 0]';
        b = A\B;
        
        a0 = b(6); a1 = b(5); a2 = b(4); 
        a3 = b(3); a4 = b(2); a5 = b(1);
        for t = linspace(t0, tf, 100)
            qt(j,idx) = a0+a1.*t+a2.*t.^2+a3.*t.^3+a4.*t.^4+a5.*t.^5;
            qp(j,idx) = a1+2*a2.*t+3*a3.*t.^2+4*a4.*t.^3+5*a5.*t.^4;
            qpp(j,idx) = 2*a2+6*a3.*t+12*a4.*t.^2+20*a5.*t.^3;
            idx = idx + 1;
        end
    end
end

% plotting the result 
tt = sum(sum(tf_syn));
t = linspace(0, tt, idx - 1);
    % ploting the results
    figure('Name', 'Polynomial solving','units','normalized','outerposition',[0 0 1 1])
    hold all
    for i = 0:n-1
        subplot(6, 3, 3*i + 1)
        plot(t, qt(i+1,:), 'k-')
        
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