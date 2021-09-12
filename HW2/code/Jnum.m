function [J] = Jnum(theta1, theta2, theta3, theta4, theta5, theta6)
%JG Summary of this function goes here
%   Detailed explanation goes here

eps = 10^(-8);

%The First Joint theta1
Jtemp = (DKM(theta1 + eps, theta2, theta3, theta4, theta5, theta6)-DKM(theta1, theta2, theta3, theta4, theta5, theta6))/(eps);
J1 = Jtemp(1:3,4);
%

%The Second Joint theta2
Jtemp = (DKM(theta1, theta2 + eps, theta3, theta4, theta5, theta6)-DKM(theta1, theta2, theta3, theta4, theta5, theta6))/(eps);
J2 = Jtemp(1:3,4);
%

%The Third Joint theta3
Jtemp = (DKM(theta1, theta2, theta3 + eps, theta4, theta5, theta6)-DKM(theta1, theta2, theta3, theta4, theta5, theta6))/(eps);
J3 = Jtemp(1:3,4);
%

%The Fourth Joint theta4
Jtemp = (DKM(theta1, theta2, theta3, theta4 + eps, theta5, theta6)-DKM(theta1, theta2, theta3, theta4, theta5, theta6))/(eps);
J4 = Jtemp(1:3,4);
%

%The FiFth Joint theta5
Jtemp = (DKM(theta1, theta2, theta3, theta4, theta5 + eps, theta6)-DKM(theta1, theta2, theta3, theta4, theta5, theta6))/(eps);
J5 = Jtemp(1:3,4);
%

%The Sixth Joint Rx
Jtemp = (DKM(theta1, theta2, theta3, theta4, theta5, theta6 + eps)-DKM(theta1, theta2, theta3, theta4, theta5, theta6))/(eps);
J6 = Jtemp(1:3,4);
%

J = [J1, J2, J3, J4, J5, J6];
end
