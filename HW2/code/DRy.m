function res = DRy(theta)
%RZ Summary of this function goes here
%   Detailed explanation goes here
res = [-sin(theta), 0, cos(theta), 0;
       0          , 0, 0         , 0;
       -cos(theta), 0 -sin(theta), 0;
       0          , 0, 0         , 0];
end