function xd = stableLimitcycle_DS(x)
% stableLimitcycle_DS
%xd = x(2,:);
%xd(2,:) = -x(1,:) + 0.9*(1 - x(1,:).^2).*x(2,:);
xd =     -x(2,:) + x(1,:) .* (1 - (x(1,:).^2 + x(2,:).^2));
xd(2,:) = x(1,:) + x(2,:) .* (1 - (x(1,:).^2 + x(2,:).^2));
%xd(3,:) = -x(3,:)*0.3;


end