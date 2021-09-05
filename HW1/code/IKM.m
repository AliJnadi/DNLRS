function [res] = IKM(T)
%IDM Summary of this function goes here
%   Detailed explanation goes here

%constants
%l1 = 0.183; l2 = 0.210; d = 0.030; l3 = 0.2215; l4 = 0.0237; l5 = 0.0055; eps = 10^(-3);
l1 = 183; l2 = 210; d = 30; l3 = 221.5; l4 = 23.7; l5 = 5.5; eps = 10^(-3);
%

Tw= T / (Tx(l4) * Tz(-l5));

Xw = Tw(1,4);
Yw = Tw(2,4);
Zw = Tw(3,4);

if(Xw == 0 && Zw == 0)
    disp('Singularity!!');
    res = NaN;
    return
end

%calculate q1 from robot top view
q1 = [atan2(Yw,  Xw), atan2(-Yw, -Xw)]; 
%

%calculate q2

%calculate q3
s = Zw - l1;
r = sqrt(Xw^2 + Yw^2);
cosq3_ = (r^2 + s^2 - l3^2 - d^2 - l2^2)/(2*l2*sqrt(l3^2 + d^2)); 

if(abs(cosq3_) > 1)
    disp('Point is unreachable');
    q = NaN;
    return
end

q3_1 = atan2(sqrt(1 - cosq3_^2) , cosq3_);
q3_2 = atan2(-sqrt(1 - cosq3_^2) , cosq3_);

q3 = q3_1 - atan2(l3,d);
q3 = [q3, q3_1 - atan2(-l3,-d)];
q3 = [q3, q3_2 - atan2(l3,d)];
q3 = [q3, q3_2 - atan2(-l3,-d)];

q3(abs(q3) < eps) = 0;
q3(q3 >= pi) = [];

%calculate q2
alpha1_1 = pi - q3_1;
alpha1_2 = pi - q3_2;

beta_1 = atan2(s,r);
beta_2 = atan2(-s,-r);

sina2_1 = sqrt(l3^2 + d^2)*sin(alpha1_1)/(sqrt(r^2 + s^2));
sina2_2 = sqrt(l3^2 + d^2)*sin(alpha1_2)/(sqrt(r^2 + s^2));


alpha2_1 = atan2(sina2_1, sqrt(1 - sina2_1^2));
alpha2_2 = atan2(sina2_1, -sqrt(1 - sina2_1^2));
alpha2_3 = atan2(sina2_2, sqrt(1 - sina2_2^2));
alpha2_4 = atan2(sina2_2, -sqrt(1 - sina2_2^2));

q2 = pi/2 - beta_1 - alpha2_1;
q2 = [q2, pi/2 - beta_1 - alpha2_2];
q2 = [q2, pi/2 - beta_1 - alpha2_3];
q2 = [q2, pi/2 - beta_1 - alpha2_4];
q2 = [q2, pi/2 - beta_2 - alpha2_1];
q2 = [q2, pi/2 - beta_2 - alpha2_2];
q2 = [q2, pi/2 - beta_2 - alpha2_3];
q2 = [q2, pi/2 - beta_2 - alpha2_4];

q2(q2 >= pi) = [];

display(q1);
display(q2);
display(q3);


end