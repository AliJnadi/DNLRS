function [res] = finalIKM(T)
%IDM Summary of this function goes here
%   Detailed explanation goes here

%constants
l1 = 0.183; l2 = 0.210; d = 0.030; l3 = 0.2215; l4 = 0.0237; l5 = 0.0055; eps = 1*10^(-5);
%

%posistion

%retract to wrist center in matlab inv(A) = 1/A
Tw= T / (Tz(-l5) * Tx(l4));
%

%calculate q1 from robot top view.
Xw = Tw(1,4);
Yw = Tw(2,4);
Zw = Tw(3,4);

if(Xw == 0 && Zw == 0)
    disp('Singularity!!');
    res = NaN;
    return
end

q1(1) = atan2(Yw,  Xw);
q1(2) = atan2(-Yw, -Xw); 

q1(abs(q1) < eps) = 0;
%

%calculate q3
s = Zw - l1;
r = sqrt(Xw^2 + Yw^2);
cosq3_ = (r^2 + s^2 - l3^2 - d^2 - l2^2)/(2*l2*sqrt(l3^2 + d^2)); 

% if cq3_ is greater than 1 then we have a complex solution, the point is
% unreachable

if(abs(cosq3_) > 1)
    disp('Point is unreachable');
    res = NaN;
    return
end

q3_1 = atan2(sqrt(1 - cosq3_^2) , cosq3_);
q3_2 = atan2(-sqrt(1 - cosq3_^2) , cosq3_);

q3(1) = q3_1 - atan2(l3,d);
q3(2) = q3_2 - atan2(l3,d);


q3(abs(q3) < eps) = 0;
%

%calculate q2
alpha1_1 = pi - q3_1;
alpha1_2 = pi - q3_2;

beta_1 = atan2(s,r);
beta_2 = atan2(s,-r);

sina2_1 = sqrt(l3^2 + d^2)*sin(alpha1_1)/(sqrt(r^2 + s^2));
sina2_2 = sqrt(l3^2 + d^2)*sin(alpha1_2)/(sqrt(r^2 + s^2));

alpha2_1 = atan2(sina2_1, sqrt(1 - sina2_1^2));
alpha2_2 = atan2(sina2_1, -sqrt(1 - sina2_1^2));
alpha2_3 = atan2(sina2_2, sqrt(1 - sina2_2^2));
alpha2_4 = atan2(sina2_2, -sqrt(1 - sina2_2^2));

ab1 = beta_1 + alpha2_1;
ab2 = beta_1 + alpha2_2;
ab3 = beta_1 + alpha2_3;
ab4 = beta_1 + alpha2_4;

ab5 = beta_2 + alpha2_1;
ab6 = beta_2 + alpha2_2;
ab7 = beta_2 + alpha2_3;
ab8 = beta_2 + alpha2_4;


q2(1) = pi/2 - ab1;
q2(2) = pi/2 - ab2;
q2(3) = pi/2 - ab3;
q2(4) = pi/2 - ab4;

q2(5) = pi/2 - ab5;
q2(6) = pi/2 - ab6;
q2(7) = pi/2 - ab7;
q2(8) = pi/2 - ab8;


q2(abs(q2) < eps) = 0;
%because the arm can't bend over this angle [-pi, pi]
q2(abs(q2) > pi/2) = [];
%
%end position


%orientation
res = [];
l = 1;
for i = 1:length(q1)
    theta1 = q1(i);
    for j = 1: length(q2)
        theta2 = q2(j);
        for k = 1:length(q3)
            theta3 = q3(k);
            T0w = Tz(l1)*Rz(theta1)*Ry(theta2)*Tz(l2)*Ry(theta3)*Tz(d)*Tx(l3);
            T456 = T0w\T;
            
            if (abs(T456(1,1)) == 1)
                theta41 = 0;
                theta51 = 0;
                theta61 = atan2(T456(2,3), T456(3,3));
            
                res = [res; 
                theta1, theta2, theta3, theta41, theta51, theta61];
            else
               theta41 = atan2(T456(2,1), -T456(3,1));
               theta61 = atan2(T456(1,2), T456(1,3));
               theta51 = atan2(sqrt(T456(1,3)^2 + T456(1,2)^2),T456(1,1));
               
               theta42 = atan2(-T456(2,1), T456(3,1));
               theta62 = atan2(-T456(1,2), -T456(1,3));
               theta52 = atan2(-sqrt(T456(1,3)^2 + T456(1,2)^2),T456(1,1));
            
               res = [res; 
               theta1, theta2, theta3, theta41, theta51, theta61; 
               theta1, theta2, theta3, theta42, theta52, theta62];
            end
        end
    end
end
%end orientation

end

