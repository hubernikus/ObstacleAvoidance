function xd= ellipseLimit_cycle(x)
% stableLimitcycle_DS of a form of an ellipse

x0 = [1;1]; % Center of ellipse
x = x-x0; 

A = diag([4,1]); % A = diag([1/a, 1/b]) with a & b the axis of the ellipse
phi = 30/180*pi; % Orienation of ellipse

cosPhi = cos(phi); sinPhi = sin(phi); 
R = [cosPhi, -sinPhi;
     sinPhi, cosPhi]; 
 
%x = pinv(A)*R'*x; 

xd      = -x(2,:) + x(1,:) .* (1 - (x(1,:).^2 + x(2,:).^2));
xd(2,:) = x(1,:) + x(2,:) .* (1 - (x(1,:).^2 + x(2,:).^2));
%xd(3,:) = -x(3,:)*0.3;

%xd = R*A*xd;


end