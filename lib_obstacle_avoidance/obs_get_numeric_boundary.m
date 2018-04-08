function [x_obs_sf, x_obs] = obs_get_numeric_boundary(obs,ns)

if nargin < 2
    ns=40;
end

d = size(obs{1}.x0,1);
if d > 3 || d < 2
    disp('Error: obs_draw_ellipsoid only supports 2D or 3D obstacle')
    x_obs = [];
    x_obs_sf = [];
    return
end

if d == 2
    theta = linspace(-pi,pi,ns(1));
else
    [theta phi] = meshgrid(linspace(-pi,pi,ns(1)),linspace(-pi/2,pi/2,ns(2))); %
    theta = theta(:)';
    phi = phi(:)';
end

np = prod(ns);

N = length(obs); %number of obstacles
x_obs = zeros(d,np,N);

if nargout > 1;
    x_obs_sf = zeros(d,np,N);
end
    
% rotating the query point into the obstacle frame of reference
if isfield(obs{n},'th_r')
    if size(obs{n}.th_r(:),1) == d && size(obs{n}.th_r(:),2) == d
        R = obs{n}.th_r;ob
    else
        R = compute_R(d,obs{n}.th_r);
    end
else
    R = eye(d);
end

% For an arbitrary shap, the next two lines are used to find the shape segment
if isfield(obs{n},'partition')
    for i=1:size(obs{n}.partition,1)
        ind(i,:) = theta>=(obs{n}.partition(i,1)) & theta<=(obs{n}.partition(i,2));
    end
    [i, ind]=max(ind);
else
    ind = 1;
end
a = obs{n}.a(:,ind);
p = obs{n}.p(:,ind);



if nargout ==1
    if isfield(obs{n},'sf')
        if d == 2
        x_obs_sf(1,:,n) = obs{n}.sf*a(1,:).*cos(theta);
        x_obs_sf(2,:,n) = obs{n}.sf*a(2,:).*sign(theta).*(1 - cos(theta).^(2.*p(1,:))).^(1./(2.*p(2,:)));
    else
        x_obs_sf(1,:,n) = obs{n}.sf*a(1,:).*cos(phi).*cos(theta);
        x_obs_sf(2,:,n) = obs{n}.sf*a(2,:).*sign(theta).*cos(phi).*(1 - 0.^(2.*p(3,:)) - cos(theta).^(2.*p(1,:))).^(1./(2.*p(2,:)));
        x_obs_sf(3,:,n) = obs{n}.sf*a(3,:).*sign(phi).*(1 - (sign(theta).*cos(phi).*(1 - 0.^(2.*p(3,:)) - cos(theta).^(2.*p(1,:))).^(1./(2.*p(2,:)))).^(2.*p(2,:)) - (cos(phi).*cos(theta)).^(2.*p(1,:))).^(1./(2.*p(3,:)));
        end
    end
end
%         if length(obs{n}.sf) == 1
%             x_obs_sf(:,:,n) = R*(x_obs(:,:,n).*obs{n}.sf) + repmat(obs{n}.x0,1,np);
%         else
%             x_obs_sf(:,:,n) = R*(x_obs(:,:,n).*repmat(obs{n}.sf,1,np)) + repmat(obs{n}.x0,1,np);
%         end
%     else
%         x_obs_sf(:,:,n) = R*x_obs(:,:,n) + repmat(obs{n}.x0,1,np);
%     end
% end
% 
% x_obs(:,:,n) = R*x_obs(:,:,n) + repmat(obs{n}.x0,1,np);    

end
