function res = DRz(theta)
%RZ Summary of this function goes here
%   Detailed explanation goes here
res = [-sin(theta), -cos(theta), 0, 0 ;
        cos(theta), -sin(theta), 0, 0 ;
        0         , 0          , 0, 0 ;
        0         , 0          , 0, 0 ];
end