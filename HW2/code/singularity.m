clc

disp('Singularity check')
disp('-----------------')

disp('Shoulder Singularity check')
disp('(3.1416, -1.2269 ,1.05 ,3.1416, 1.8575, 0)')
J = Jn(3.1416, -1.2269 ,1.05 ,3.1416, 1.8575, 0);
fprintf('Jacobian rank %d.\n', rank(J));
fprintf('Jacobian det %d.\n', det(J));
[~,s,~] = svd(J);
fprintf('SVD rank %d.\n', rank(s));
disp('Determinant is very small but not reach zero due to I am near singularity')
disp('-----------------')

disp('Elbow Singularity check')
disp('(0, pi/2, -pi/2, 0, 0, 0)')
J = Jn(0, pi/2 ,-pi/2 ,0, 0, 0);
fprintf('Jacobian rank %d.\n', rank(J));
fprintf('Jacobian det %d.\n', det(J));
[~,s,~] = svd(J);
fprintf('SVD rank %d.\n', rank(s));
disp('-----------------')


disp('Wrist Singularity check')
disp('(0, 0, 0, 0, 0, 0)')
J = Jn(0, 0, 0, 0, 0, 0);
fprintf('Jacobian rank %d.\n', rank(J));
fprintf('Jacobian det %d.\n', det(J));
[~,s,~] = svd(J);
fprintf('SVD rank %d.\n', rank(s));
disp('-----------------')