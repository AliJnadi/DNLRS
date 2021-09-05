%generates random values for testing
disp("start");
n = 5;

fprintf('number of tests %i \n' , n);
match = [];
for i = 1 : n
   b1 = [-pi -pi/2 -pi/2 -pi -pi/2 -pi];
   b2 = [ pi  pi/2  pi/4  pi  pi/2  pi];
   test = (b2-b1).*rand(1,6) + b1;
   m = 0;
   T = DKM(test(1),test(2),test(3),test(4),test(5),test(6),0,0);
   q = finalIKM(T);
   for j = 1 : length(q)
      Ts = DKM(q(j,1),q(j,2),q(j,3),q(j,4),q(j,5),q(j,6),0,0);
      if(norm(T - Ts) < 1*10^(-5))
          m = m + 1;
      end
   end
   match = [match, m];
end
fprintf('number of solution %d \n' , length(q));
disp('number of match');
disp(match);