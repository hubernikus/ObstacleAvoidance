function xd = linearStableDS_tanh(x,x0, vel_max, k)
% A = k(x) * I ; 

if nargin<2
    x0 = [0;0];
end

if nargin<3
    vel_max = 1;
end

if nargin<4
    % Defines the smoothnes of the tangent hyperbolicus, (k-> infinity = sign function)
    k = 1;
end
normX = norm(x-x0,2);

xd = - vel_max*tanh(normX*k)./normX.*(x-x0);

end

