%generates random values for testing
clc
disp("Jacobian Validation Check");
n = 100;

fprintf('number of tests %i \n' , n);
p = 0;
for i = 1 : n
   b1 = [-pi -pi/2 -pi/2 -pi -pi/2 -pi];
   b2 = [ pi  pi/2  pi/4  pi  pi/2  pi];
   test = (b2-b1).*rand(1,6) + b1;
   
   Tn = Jn(test(1),test(2),test(3),test(4),test(5),test(6));
   Tnum = Jnum(test(1),test(2),test(3),test(4),test(5),test(6));
   T = Tnum - Tn(1:3,:);
   if max(max(T)) < 10^-5
     p = p + 1;  
   end
end
fprintf('success %d \n' , p);
fprintf('failed %d \n' , n - p);