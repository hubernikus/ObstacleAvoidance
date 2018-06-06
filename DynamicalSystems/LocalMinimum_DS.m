function xd = LocalMinimum_DS(x, pow, velMax)
if nargin<2
    pow = 3;
end
if nargin<3
    velMax = 1;
end

N = size(x,2);

% stableLimitcycle_DS
xd(1,:) = -x(1,:); %- min(ones(1,N), x(1,:) );
xd(2,:) = -sign(x(2,:)).*abs(x(2,:)).^pow;

% Norm of DS
normXD = norm(xd);
xd = min(velMax, normXD)/normXD*xd;
end