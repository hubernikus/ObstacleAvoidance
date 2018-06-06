function xd = linearStableDS_const(x,x0, velConst)
% A = - I ; 
if nargin<2
    x0 = [0;0];
end

if nargin<3
    velConst = 6;
end

dim = size(x,1);

% Uniform DS
xd = - (x-x0);

% Get zero values
normXd = sqrt(sum(xd.^2,1));
ind0 = normXd ~= 0;

xd(:,ind0) = xd(:,ind0).*repmat(min(1./normXd(ind0),1)*velConst, dim,1);
xd(:,~ind0) = zeros(size(xd,1),size(x0,2)-length(ind0));

end

