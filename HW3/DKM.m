function [P, T] = DKM(theta1, theta2, theta3, theta4, theta5, theta6)
%DKM Summary of this function goes here
%   Detailed explanation goes here

%constants
l1 = 0.103; l2 = 0.08; l3 = 0.21; d = 0.03; l4 = 0.0415; l5 = 0.180; l6 = 0.0237; l7 = 0.0055; eps = 10^(-3);
%

%calculating the Transformation matrix for arm, wrist and end effector
T0w = Tz(l1 + l2)*Rz(theta1)*Ry(theta2)*Tz(l3)*Ry(theta3)*Tz(d)*Tx(l4 + l5);
Tw6 = Rx(theta4)*Ry(theta5)*Rx(theta6);
T6e = Tx(l6)*Tz(-l7);
T = T0w*Tw6*T6e;

P = T(1:3,4);
end