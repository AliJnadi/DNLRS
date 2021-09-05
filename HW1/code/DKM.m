function [T] = DKM(theta1, theta2, theta3, theta4, theta5, theta6, draw_robot, show_symb)
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


if(show_symb == true)
    %using symbolic variables this help me to check if the my hand results is correct 
    syms q1 q2 q3 q4 q5 q6
    
    T0w = Tz(l1 + l2)*Rz(q1)*Ry(q2)*Tz(l3)*Ry(q3)*Tz(d)*Tx(l4 + l5);
    Tw6 = Rx(q4)*Ry(q5)*Rx(q6);
    T6e = Tx(l6)*Tz(-l7);

    sT   = simplify(T0w*Tw6*T6e);              %symbolicT,
    disp(sT);
end

if(draw_robot == false)
    return;
else
    
%for ploting I have to gather all points from transformation matrics
    T01  = Tz(l1)*Rz(theta1);
    T12  = Tz(l2)*Ry(theta2);
    T23  = Tz(l3)*Ry(theta3);
    T3d  = Tz(d);
    Tdw1  = Tx(l4)*Rx(theta4);
    Tw1w = Tx(l5)*Ry(theta5);
    Tw6p = Rx(theta6);
    T6e1 = Tx(l6);    
    Te1e = Tz(-l7);

    P1  = T01;
    P2  = P1*T12;
    P3  = P2*T23;
    P4  = P3*T3d;
    P5  = P4*Tdw1;
    P6  = P5*Tw1w;
    P7  = P6*Tw6p;
    P8  = P7*T6e1;
    P9  = P8*Te1e;

    pos = [[0, 0, 0];P1(1:3,4)'; P2(1:3,4)'; P3(1:3,4)'; P4(1:3,4)'; P5(1:3,4)'; 
           P6(1:3,4)'; P7(1:3,4)'; P8(1:3,4)'; P9(1:3,4)'];
    figure;
    hold on
    grid on
    view(25, 25);
    
    plot3(pos(:,1), pos(:,2), pos(:,3), '-o', 'LineWidth', 1);
end

end

