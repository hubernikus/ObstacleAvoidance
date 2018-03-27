function xd = parallelFlow_DS(x, U_inf, dir)
% stableLimitcycle_DS
xd = zeros(size(x));

if nargin<2
    U_inf = 1;
end


xd(1,:) = U_inf;
xd(2,:) = 0;

end