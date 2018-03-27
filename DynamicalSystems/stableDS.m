function xd = stableDS(x, x0)
% STABLEDS_3D - Stable 3D dynamical system
% A = - I ; 
[N,M] = size(x);
if nargin < 2
    x0 = zeros(N,1);
end

xd = - (x(:,:)-repmat(x0,1,M));
end

