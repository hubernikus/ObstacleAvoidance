function xd = linearStableDS_erf(x,x0, vel_max, k)
% A = k(x) * I ; 
% Using error function

if nargin<2
    x0 = [0;0];
end

if nargin<3
    vel_max = 1;
end

if nargin<4
    % Defines the smoothnes of the tangent hyperbolicus, (k-> infinity = sign function)
    % with k=1, decrease starts at around 5.926
    k = 1;
end

normX = sqrt(sum((x-repmat(x0,1,size(x,2))).^2,1));

xd = - vel_max*erf(normX*k)./normX.*(x-x0);

end

