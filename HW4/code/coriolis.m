function [C] = coriolis(M, q)
%CORIOLIS Summary of this function goes here
%   Detailed explanation goes here
    n = length(q);
    % Coriolis
    syms C
    C = C * zeros(n, n);
    for i = 1: n
        for j = 1: n
            temp = 0;
            for k = 1:n
                temp = temp + 0.5 * (diff(M(i,j),q(k)) + diff(M(i,k),q(j)) - diff(M(j,k),q(i)));
            end
            if isa(temp, 'sym')
                C(i,j) = temp;
            else
                C(i,j) = 0;
            end
        end
    end
    if isa(C,'sym')
        C = simplify(C);
    end
end

