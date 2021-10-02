% Start solution
clear
clc

% %constants
% l1 = 0.103; l2 = 0.08; l3 = 0.21; d = 0.03; l4 = 0.0415; l5 = 0.180; 
% l6 = 0.0237; l7 = 0.0055; eps = 10^(-3);
n = 6;
%

%Symbolics (real) is syms parameter that all syms are real numbers
syms q1 q2 q3 q4 q5 q6 real %% robot thetas
syms d1 d2 d3 d4 d5 d6 real %% COM shifted
syms l1 l2 l3 d l4 l5 l6 l7 real % robot links dimenstions
syms m1 m2 m3 m4 m5 m6 % robot links masses
syms I1 I2 I3 I4 I5 I6 % robot links Inertias
syms g % 9.81
%

q = [q1, q2, q3, q4, q5, q6];
m = [m1, m2, m3, m4, m5, m6];
I = [I1, I2, I3, I4, I5, I6];

% Jacobians


% The first link
R0c1  = Rz(q1)*Tz(d1);
u0 = [0 0 0]';               %first link rotate around z 
% Center of mass cordinates
oc1  = simplify(R0c1(1:3,4));
Jv1 = [diff(oc1, q(1)) zeros(3,n-1)];
Jw1 = [u0 zeros(3,5)];
%

% The second link
R0c2  = Rz(q1)*Tz(l1+l2)*Ry(q2)*Tz(d2);
u1 = R0c1(1:3,2);               %second link rotate around y
% Center of mass cordinates
oc2 = simplify(R0c2(1:3,4));

Jv2 = [diff(oc2, q(1)) diff(oc2, q(2)) zeros(3,n-2)];
Jv2 = simplify(Jv2);

Jw2 = [u0 u1 zeros(3,n-2)];
Jw2 = simplify(Jw2);
%

% The third link
R0c3  = Rz(q1)*Tz(l1+l2)*Ry(q2)*Tz(l3)*Ry(q3)*Tz(d)*Tx(d3);
u2 = R0c2(1:3,2);             %third link rotate around y
% Center of mass cordinates
oc3 = simplify(R0c3(1:3,4));

Jv3 = [diff(oc3, q(1)) diff(oc3, q(2)) diff(oc3, q(3)) zeros(3,n-3)];
Jv3 = simplify(Jv3);

Jw3 = [u0 u1 u2 zeros(3,n-3)];
Jw3 = simplify(Jw3);
%

% The fourth link
R0c4  = Rz(q1)*Tz(l1+l2)*Ry(q2)*Tz(l3)*Ry(q3)*Tz(d)*Tx(l4)*Rx(q4)*Tx(d4);
u3 = R0c3(1:3,1);               %fourth link rotate around x
% Center of mass cordinates
oc4 = simplify(R0c4(1:3,4));

Jv4 = [diff(oc4, q(1)) diff(oc4, q(2)) diff(oc4, q(3)) diff(oc4, q(4)) zeros(3,n-4)];
Jv4 = simplify(Jv4);

Jw4 = [u0 u1 u2 u3 zeros(3,n-4)];
Jw4 = simplify(Jw4);
%

% The fifth link
R0c5  = Rz(q1)*Tz(l1+l2)*Ry(q2)*Tz(l3)*Ry(q3)*Tz(d)*Tx(l4)*Rx(q4)*Tx(l5)*Ry(q5)*Ty(d5);
u4 = R0c4(1:3,2);                %fifth link rotate around y
% Center of mass cordinates
oc5 = simplify(R0c5(1:3,4));

Jv5 = [diff(oc5, q(1)) diff(oc5, q(2)) diff(oc5, q(3)) diff(oc5, q(4)) diff(oc5, q(5)) zeros(3,n-5)];
Jv5 = simplify(Jv5);

Jw5 = [u0 u1 u2 u3 u4 zeros(3,n-5)];
Jw5 = simplify(Jw5);
%

% The sixth link
R0c6  = Rz(q1)*Tz(l1+l2)*Ry(q2)*Tz(l3)*Ry(q3)*Tz(d)*Tx(l4)*Rx(q4)*Tx(l5)*Ry(q5)*Rx(q6)*Tx(d6);
u5 = R0c5(1:3,1);                %sixth link rotate around y
% Center of mass cordinates
oc6 = simplify(R0c6(1:3,4));

Jv6 = [diff(oc6, q(1)) diff(oc6, q(2)) diff(oc6, q(3)) diff(oc6, q(4)) diff(oc6, q(5)) diff(oc6, q(6))];
Jv6 = simplify(Jv6);

Jw6 = [u0 u1 u2 u3 u4 u5];
Jw6 = simplify(Jw6);
%

Jv = [Jv1, Jv2, Jv3, Jv4, Jv5, Jv6];
Jw = [Jw1, Jw2, Jw3, Jw4, Jw5, Jw6];
%

%Calculating M
R1 = R0c1(1:3,1:3); R2 = R0c2(1:3,1:3); R3 = R0c3(1:3,1:3);
R4 = R0c4(1:3,1:3); R5 = R0c5(1:3,1:3); R6 = R0c6(1:3,1:3);

R = [R1, R2, R3, R4, R5, R6];

syms M real
M1 =  m1 * (Jv1)' * Jv1 + (Jw1)' * R1 * I1 * R1' * Jw1;
M2 =  m2 * (Jv2)' * Jv2 + (Jw2)' * R2 * I2 * R2' * Jw2;
M3 =  m3 * (Jv3)' * Jv3 + (Jw3)' * R3 * I3 * R3' * Jw3;
M4 =  m4 * (Jv4)' * Jv4 + (Jw4)' * R4 * I4 * R4' * Jw4;
M5 =  m5 * (Jv5)' * Jv5 + (Jw5)' * R5 * I5 * R5' * Jw5;
M6 =  m6 * (Jv6)' * Jv6 + (Jw6)' * R6 * I6 * R6' * Jw6; 
M = M1 + M2 + M3 + M4 + M5 + M6;
%


% Coriolis
C = coriolis(M,q);
%

%Calculating G
g0 = g*[0 0 -1]';

G1 = -1 * Jv1(:,1)' * m1 * g0 -1 * Jv2(:,1)' * m2 * g0 -1 * Jv3(:,1)' * m3 * g0 -1 * Jv4(:,1)' * m4 * g0 -1 * Jv5(:,1)' * m5 * g0 -1 * Jv6(:,1)' * m6 * g0;
G2 = -1 * Jv1(:,2)' * m1 * g0 -1 * Jv2(:,2)' * m2 * g0 -1 * Jv3(:,2)' * m3 * g0 -1 * Jv4(:,2)' * m4 * g0 -1 * Jv5(:,2)' * m5 * g0 -1 * Jv6(:,2)' * m6 * g0;
G3 = -1 * Jv1(:,3)' * m1 * g0 -1 * Jv2(:,3)' * m2 * g0 -1 * Jv3(:,3)' * m3 * g0 -1 * Jv4(:,3)' * m4 * g0 -1 * Jv5(:,3)' * m5 * g0 -1 * Jv6(:,3)' * m6 * g0;
G4 = -1 * Jv1(:,4)' * m1 * g0 -1 * Jv2(:,4)' * m2 * g0 -1 * Jv3(:,4)' * m3 * g0 -1 * Jv4(:,4)' * m4 * g0 -1 * Jv5(:,4)' * m5 * g0 -1 * Jv6(:,4)' * m6 * g0;
G5 = -1 * Jv1(:,5)' * m1 * g0 -1 * Jv2(:,5)' * m2 * g0 -1 * Jv3(:,5)' * m3 * g0 -1 * Jv4(:,5)' * m4 * g0 -1 * Jv5(:,5)' * m5 * g0 -1 * Jv6(:,5)' * m6 * g0;
G6 = -1 * Jv1(:,6)' * m1 * g0 -1 * Jv2(:,6)' * m2 * g0 -1 * Jv3(:,6)' * m3 * g0 -1 * Jv4(:,6)' * m4 * g0 -1 * Jv5(:,6)' * m5 * g0 -1 * Jv6(:,6)' * m6 * g0;

G = [G1; G2; G3; G4; G5; G6];
G = simplify(G);
%

if(show_result == 1)
   display(Jv1);
   display(Jw1);
   display(Jv2);
   display(Jw2);
   display(Jv3);
   display(Jw3);
   display(Jv4);
   display(Jw4);
   display(Jv5);
   display(Jw5);
   display(Jv6);
   display(Jw6);
   
   display(M);
   display(C);
   display(G);
end

%