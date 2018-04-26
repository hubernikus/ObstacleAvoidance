 gfunction [ div, u, v,Gamma,t1,t2] = divergence_velocityField(x, obs )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Function defined for two dimensions
d=2; 

R = compute_R(d,obs{1}.th_r);

x1 = x(1,:); x2 = x(2,:);
x_transp = R'*(x-obs{1}.x0);

% Calculate Eigenvalues
% Change in case of Change in Gamma
Gamma = sum((x_transp./obs{1}.a).^(2*obs{1}.p));

l_n = 1-1/(Gamma);
l_t = 1+1/(Gamma);

% Distance to obstalce -- only one obstacle at center
d1 = obs{1}.x0(1);
d2 = 0;

% Tangent to obstacle --- Adapt for more coplex obstacle than ellipse
t = [obs{1}.a(1)^2*x_transp(2);
    -obs{1}.a(2)^2*x_transp(1)];

t = t/norm(t);
t = R*t;
t1 = t(1);
t2 = t(2);

% 'Pseudo' Normal vector
n1 = (x1-d1);
n2 = (x2-d2);


% E = [n1, t1;
%      n2, t2];
% D = diag([l_n, l_t]);
% 
% M = E*D*pinv(E);
% xd = M*[-x1;-x2];

% Calculate divergence (formula from python script)
div = (-d1*l_n*t2 - d1*l_t*t2 - 2*l_n*t1*x2 + 2*l_n*t2*x1)/(d1*t2 + t1*x2 - t2*x1);

coll = obs_check_collision(obs, [x1],[x2]);
if coll
    u = 0;
    v = 0;
    return;
end

u = -(-l_n*t1*(-d1 + x1)./(-t1*x2 + t2*(-d1 + x1)) + l_t*t1*(-d1 + x1)./(-t1*x2 + t2*(-d1 + x1)))*x2 - (l_n*t2*(-d1 + x1)/(-t1*x2 + t2*(-d1 + x1)) - l_t*t1*x2/(-t1*x2 + t2*(-d1 + x1)))*x1;
v = -(-l_n*t1*x2/(-t1*x2 + t2*(-d1 + x1)) + l_t*t2*(-d1 + x1)/(-t1*x2 + t2*(-d1 + x1)))*x2 - (l_n*t2*x2/(-t1*x2 + t2*(-d1 + x1)) - l_t*t2*x2/(-t1*x2 + t2*(-d1 + x1)))*x1;

%fprintf('x=[%2.3f,%2.3f], u=[%2.3f,%2.3f], dx=[%2.3f,%2.3f] \n',x1,x2,u,v, (u-xd(1)),(v-xd(2)) )

end

function R = compute_R(d,th_r)
% rotating the query point into the obstacle frame of reference

if d == 2 
    R = [cos(th_r(1)) -sin(th_r(1));sin(th_r(1)) cos(th_r(1))];
elseif d == 3
    R_x = [ 1, 0, 0; 0, cos(th_r(1)), sin(th_r(1)); 0, -sin(th_r(1)), cos(th_r(1))];
    R_y = [cos(th_r(2)), 0, -sin(th_r(2)); 0, 1, 0; sin(th_r(2)), 0, cos(th_r(2))];
    R_z = [cos(th_r(3)), sin(th_r(3)), 0; -sin(th_r(3)), cos(th_r(3)), 0; 0, 0, 1];
    R = R_x*R_y*R_z;
else %rotation is not yet supported for d > 3
    R = eye(d);
end

end
