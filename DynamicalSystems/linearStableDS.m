function xd = linearStableDS(x,x0)
% A = - I ; 
if nargin<2
    x0 = [0;0];
end

xd = - (x-x0);

end

