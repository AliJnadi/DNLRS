function [J] = Jn(theta1, theta2, theta3, theta4, theta5, theta6)
%JG Summary of this function goes here
%   Detailed explanation goes here

%constants
l1 = 0.103; l2 = 0.08; l3 = 0.21; d = 0.03; l4 = 0.0415; l5 = 0.180; l6 = 0.0237; l7 = 0.0055; eps = 10^(-3);
%

%Calculate Transformation Matrix
T6 =  Tz(l1+l2)*Rz(theta1)*Ry(theta2)*Tz(l3)*Ry(theta3)*Tz(d)*Tx(l4+l5)*Rx(theta4)*Ry(theta5)*Rx(theta6)*Tx(l6)*Tz(-l7);

T0 = eye(4,4);
T0(1:3,1:3) = inv(T6(1:3,1:3));

%The First Joint Rz
Jtemp = Tz(l1 + l2)*DRz(theta1)*Ry(theta2)*Tz(l3)*Ry(theta3)*Tz(d)*Tx(l4+l5)*Rx(theta4)*Ry(theta5)*Rx(theta6)*Tx(l6)*Tz(-l7)*T0;
J1 = [Jtemp(1,4), Jtemp(2,4), Jtemp(3,4), Jtemp(3,2), Jtemp(1,3), Jtemp(2,1)]';
%

%The Second Joint Ry
Jtemp = Tz(l1 + l2)*Rz(theta1)*DRy(theta2)*Tz(l3)*Ry(theta3)*Tz(d)*Tx(l4+l5)*Rx(theta4)*Ry(theta5)*Rx(theta6)*Tx(l6)*Tz(-l7)*T0;
J2 = [Jtemp(1,4), Jtemp(2,4), Jtemp(3,4), Jtemp(3,2), Jtemp(1,3), Jtemp(2,1)]';
%

%The Third Joint Ry
Jtemp = Tz(l1 + l2)*Rz(theta1)*Ry(theta2)*Tz(l3)*DRy(theta3)*Tz(d)*Tx(l4+l5)*Rx(theta4)*Ry(theta5)*Rx(theta6)*Tx(l6)*Tz(-l7)*T0;
J3 = [Jtemp(1,4), Jtemp(2,4), Jtemp(3,4), Jtemp(3,2), Jtemp(1,3), Jtemp(2,1)]';
%

%The Fourth Joint Rx
Jtemp = Tz(l1 + l2)*Rz(theta1)*Ry(theta2)*Tz(l3)*Ry(theta3)*Tz(d)*Tx(l4+l5)*DRx(theta4)*Ry(theta5)*Rx(theta6)*Tx(l6)*Tz(-l7)*T0;
J4 = [Jtemp(1,4), Jtemp(2,4), Jtemp(3,4), Jtemp(3,2), Jtemp(1,3), Jtemp(2,1)]';
%

%The FiFth Joint Ry
Jtemp = Tz(l1 + l2)*Rz(theta1)*Ry(theta2)*Tz(l3)*Ry(theta3)*Tz(d)*Tx(l4+l5)*Rx(theta4)*DRy(theta5)*Rx(theta6)*Tx(l6)*Tz(-l7)*T0;
J5 = [Jtemp(1,4), Jtemp(2,4), Jtemp(3,4), Jtemp(3,2), Jtemp(1,3), Jtemp(2,1)]';
%

%The Sixth Joint Rx
Jtemp = Tz(l1 + l2)*Rz(theta1)*Ry(theta2)*Tz(l3)*Ry(theta3)*Tz(d)*Tx(l4+l5)*Rx(theta4)*Ry(theta5)*DRx(theta6)*Tx(l6)*Tz(-l7)*T0;
J6 = [Jtemp(1,4), Jtemp(2,4), Jtemp(3,4), Jtemp(3,2), Jtemp(1,3), Jtemp(2,1)]';
%

J = [J1, J2, J3, J4, J5, J6];
end
