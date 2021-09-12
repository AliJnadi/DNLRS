function res = DRx(theta)
%RZ Summary of this function goes here
%   Detailed explanation goes here
res = [0,  0          , 0           , 0 ;
       0, -sin(theta) , -cos(theta) , 0 ;
       0,  cos(theta) , -sin(theta) , 0 ;
       0,  0          ,  0          , 0 ];
end