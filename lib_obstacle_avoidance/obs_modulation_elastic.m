function [xd, M, compTime] = obs_modulation_elastic(x,xd,obs, ds_handle, varargin)
%
% Obstacle avoidance module: Version 1.2, issued on July 30, 2015
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%   Copyright (c) 2011 S. Mohammad Khansari-Zadeh, LASA Lab, EPFL,    %%%
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This function computes the sufficient modulation
% due to the presence of obstacle(s) so that the generated trajectories
% do not penetrate into the obstacles. 
%
% To avoid struggling with the MATLAB symbolic toolbox, and for the sake of
% simplicity, the provided source code can only be used to avoid obstacles
% in the form of:
%           \Gamma(xt):   \sum_{i=1}^d (xt_i/a_i)^(2p_i) = 1
% For other forms of obstacle shape, it is necessary to modify this file to
% properly compute the matrix of eigenvalues and eigenvectors of the
% dynamic modulation matrix. 
%
% The function is called using:
%       [xd M] = obs_modulation_ellipsoid(x,xd,obs,xd_obs)
%
%
% Inputs -----------------------------------------------------------------
%
%   o x:         d x 1 column vector corresponding to the current robot state
%                probabilities of the K GMM components.
%
%   o xd:        d x 1 column vector corresponding to the derivative of the robot state
%
%   o obs:       A cell array of length N, containing the definition of the N
%                presented obstacles. For example, obs{i} provides the
%                properties of the i-th obstacle. Each obs{i} is a
%                structure variable, and should contains at least the
%                following properties: 
%           - .a:    The ellipsoid length scale (e.g. [1;1] for a 2D circle.
%           - .p:    The ellipsoid power terms (e.g. [1;1] for a 2D circle.
%           - .x0:   The ellipsoid reference point (e.g. [0;0]).
%           - .sf:   The safety factor (optional, default = 1).
%           - .tailEffect (optional, default = true): If it is set to true,
%                    the obstacle modifies the motion even when the it is
%                    moving away from the obstacle. 
%           - .rho (optional, default = 1.0): Sets the reactivity
%                    parameter. The larger the rho, the earlier the robot
%                    responds to the presence of the obstacle. 
%           - .th_r: The obstacle rotation with respect to the global
%                    frame of reference (optional, default = 0.0 rad). 
%           - .partition: Defines the partition of the ellipsoid (optional,
%                    default = [-pi pi]) 
%           Please run 'Tutorial_Obstacle_Avoidance.m' for further information
%           on how to use this obstacle avoidance module.
%
%   o xd_obs:    d x 1 column vector defining the obstacle velocity
%
% Outputs ----------------------------------------------------------------
%
%   o xd:        d x 1 column vector corresponding to the modulated
%                robot velocity.
%
%   o M:         d x d matrix representing the dynamic modulation matrix.
% 
% 
% This code is writen based on the following paper:
% 
%     S.M. Khansari Zadeh and A. Billard, "A Dynamical System Approach to
%     Realtime Obstacle Avoidance", Autonomous Robots, 2012
%
%%
tic;

N_obs = length(obs); %number of obstacles
dim = size(x,1);
Gamma = zeros(1,N_obs);

xd_dx_obs = zeros(dim,N_obs);
xd_w_obs = zeros(dim,N_obs); %velocity due to the rotation of the obstacle

% List of all velocities with respecto to each obstacle
xd_list = zeros(dim,N_obs);

% Initialize variable in case of fixeable in case of fixeable in case of fixeable in case of fixed points
Gamma_fixedPoints = [];
fixed_points = [];

% Weird behavior of varargin when creating function handle, this can be
% handled by adding this line.
% TODO: explore weird behavior and remove next lines..
switch(class(varargin{1}))
     case 'cell'
         varargin = varargin{1};
end    
switch(class(varargin{2})) % weird behavior line added... 
     case 'cell'
         varargin{2} = varargin{2}{1};
end    

for i=1:length(varargin)
    if ~isempty(varargin)
        switch i
            case 1
                xd_dx_obs = varargin{1};
            case 2
                w_obs = varargin{2};
                if dim==2 && size(w_obs,1)==1 && size(w_obs,2)==N_obs
                    for n=1:N_obs
                        x_tmp = x-obs{n}.x0;
                        xd_w_obs(:,n) = [-x_tmp(2);x_tmp(1)]*w_obs(n); %cross(w,x_tmp)
                    end
                end
                if dim==3 && length(w_obs)==dim %supporting situation
                    xd_w_obs = cross(w_obs,x_tmp);
                end
            case 3
                fixed_points = varargin{3}; 
                if size(fixed_points,1)
                    % Gamma of fixed points is 
                    Gamma_fixedPoints = sqrt(sum((x-fixed_points).^2,1));
                    xd_list = [xd_list,repmat(xd,1,size(fixed_points,2))];
                end
        end
    end
end

if dim==3
    E = zeros(dim,dim+1,N_obs);
else
    E = zeros(dim,dim,N_obs);
end

rotMat = zeros(dim,dim,N_obs);

R = zeros(dim,dim,N_obs);
M = eye(dim);

x_hat = zeros(dim, N_obs);

for n=1:N_obs
    % rotating the query point into the obstacle frame of reference
    if isfield(obs{n},'th_r')
        R(:,:,n) = compute_R(dim,obs{n}.th_r);
    else
        R(:,:,n) = eye(dim);
    end
    % x_alligned is centered at obs center and alligned with 
    x_alligned = R(:,:,n)'*(x-obs{n}.x0);
    %[E(:,:,n) Gamma(n)] = compute_basis_matrix(d,x_t,obs{n},R(:,:,n));
    
    % Caclulate ds at center of obstacle
    obs{n}.dx_center = ds_handle(obs{n}.x0);
    
    % Rotation Matrix
    [rotMat(:,:,n), Gamma(n)] = compute_rotation_matrix(dim,x_alligned,obs{n},R(:,:,n));
    [E(:,:,n), Gamma(n)] = compute_basis_matrix(dim,x_alligned,obs{n},R(:,:,n));
%     if Gamma(n)<0.99
%         disp(Gamma(n))
%     end

    % Caclulate DS hat
    x_hat(:,n) = compute_replacement_position(dim,x_alligned,obs{n},R(:,:,n), Gamma(n)); % TODO -- maybe include in previous function

end


%xd = xd_hat; % REMOVE

w = compute_weights([Gamma,Gamma_fixedPoints],N_obs);

%adding the influence of the rotational and cartesian velocity of the
%obstacle to the velocity of the robot
% xd_obs = 0;
% for n=1:N_obs
%     if ~isfield(obs{n},'sigma')
%         obs{n}.sigma = 1;
%     end
%     %the exponential term is very helpful as it help to avoid the crazy rotation of the robot due to the rotation of the object
%     xd_obs = xd_obs + w(n) * exp(-1/obs{n}.sigma*(max([Gamma(n) 1])-1))* ... 
%                                        (xd_dx_obs(:,n) + xd_w_obs(:,n)); 
% end
% xd = xd-xd_obs; %computing the relative velocity with respect to the obstacle

%ordering the obstacle number so as the closest one will be considered at
%last (i.e. higher priority)
[~,obs_order] = sort(Gamma,'descend');
for n = obs_order
    
    % Replacement DS
    xd = ds_handle(x_hat(:,n));
    
    if ~isfield(obs{n},'sigma')
        obs{n}.sigma = 1;
    end
    %the exponential term is very helpful as it help to avoid the crazy rotation of the robot due to the rotation of the object
    xd_obs = exp(-1/obs{n}.sigma*(max([Gamma(n) 1])-1))* ... 
                                       (xd_dx_obs(:,n) + xd_w_obs(:,n)); 
    xd = xd-xd_obs; %computing the relative velocity with respect to the obstacle

    if isfield(obs{n},'rho')
        rho = obs{n}.rho;
    else
        rho = 1;
    end
%     if isfield(obs{n},'eigenvalue')
%         d0 = obs{n}.eigenvalue;
%     else
         d0 = ones(size(E,2)-1,1);
%     end
            
    D = w(n)*([-1;d0]/abs(Gamma(n))^(1/rho));
    if isfield(obs{n},'tailEffect') && ~obs{n}.tailEffect && xd'*R(:,:,n)*E(:,1,n)>=0 %the obstacle is already passed, no need to do anything
        D(1) = 0.0;
    end
    
    if D(1) < -1.0
        D(2:end) = d0;
        if xd'*R(:,:,n)*E(:,1,n) < 0
            D(1) = -1.0;
        end
    end
    
    %M = (R(:,:,n)*E(:,:,n)*diag(D+[1;d0])/E(:,:,n)*R(:,:,n)')*M;
    M = (R(:,:,n)*E(:,:,n)*diag(D+[1;d0])/E(:,:,n)*R(:,:,n)');
    xd = M*xd; %velocity modulation
    % kappa = 1; % TODO -- is correct?
    % TODO: is correct?
    %M = kappa*rotMat(:,:,n)*M;
    xd_list(:,n) = xd + xd_obs; %transforming back the velocity into the global coordinate system
end

xd = sum(xd_list.*repmat(w,size(xd_list,1),1),2);

%E(:,:,n) = R(:,:,n)*E(:,:,n); %transforming the basis vector into the global coordinate system

% fig1 = figure(2);
% plot(x(1),x(2),'ko'); hold on;
% [~,x_obs_sf]= obs_draw_ellipsoid(obs,50);
% axis equal;
% patch(x_obs_sf(1,:),x_obs_sf(2,:),[0.2,0.4 0.2],'FaceAlpha',0.3)
% quiver(x(1),x(2), xd(1),xd(2),'b'); hold on;

%xd = M*xd; %velocity modulation
% quiver(x(1),x(2), xd(1),xd(2),'r'); hold on;

%     if norm(M*xd)>0.05
%         xd = norm(xd)/norm(M*xd)*M*xd; %velocity modulation
%     end

%xd = xd + xd_obs; %transforming back the velocity into the global coordinate system

compTime = toc;
end


function [E, Gamma] = compute_basis_matrix(d,x_t,obs, R)
% For an arbitrary shap, the next two lines are used to find the shape segment
th = atan2(x_t(2),x_t(1));
if isfield(obs,'partition')
    ind = find(th>=(obs.partition(:,1)) & th<=(obs.partition(:,2)),1);
else
    ind = 1;
end
if isfield(obs,'sf')
    a = obs.a(:,ind).*obs.sf;
else
    a = obs.a(:,ind);
end
p = obs.p(:,ind);
Gamma = sum((x_t./a).^(2*p));

nv = (2*p./a.*(x_t./a).^(2*p - 1)); %normal vector of the tangential hyper-plane

%generating E, for a 2D model it simply is: E = [dx [-dx(2);dx(1)]];
E = zeros(d,d);

if isfield(obs, 'x_center_dyn') % automatic adaptation of center 
    %R= compute_R(d, obs.th_r);
    E(:,1) = - (x_t - R'*(obs.x_center_dyn - obs.x0));
    
    %E(:,1) = - (x_t - (obs.x_center.*obs.a))
    %fprintf('remove')
elseif isfield(obs, 'x_center') % For relative center
    E(:,1) = - (x_t - (obs.x_center.*obs.a));
else
    E(:,1) = - x_t;
end

E(1,2:d) = nv(2:d)';
E(2:d,2:d) = -eye(d-1)*nv(1);

if d == 3
    E(:,end+1) = [0;-nv(3);nv(2)];
end

end

function [rotMat, Gamma] = compute_rotation_matrix(dim, x_t, obs, R)

% For an arbitrary shap, the next two lines are used to find the shape segment
th = atan2(x_t(2),x_t(1));
if isfield(obs,'partition')
    ind = find(th>=(obs.partition(:,1)) & th<=(obs.partition(:,2)),1);
else
    ind = 1;
end
if isfield(obs,'sf')
    a = obs.a(:,ind).*obs.sf;
else
    a = obs.a(:,ind);
end
p = obs.p(:,ind);
Gamma = sum((x_t./a).^(2*p));

nv = (2*p./a.*(x_t./a).^(2*p - 1)); %normal vector of the tangential hyper-plane

%generating E, for a 2D model it simply is: E = [dx [-dx(2);dx(1)]];
E = zeros(dim,dim);

if isfield(obs, 'x_center_dyn') % automatic adaptation of center 
    %R= compute_R(d, obs.th_r);
    n_center = - (x_t - R'*(obs.x_center_dyn - obs.x0));
    
    %E(:,1) = - (x_t - (obs.x_center.*obs.a))
    %fprintf('remove')
elseif isfield(obs, 'x_center') % For relative center  -- TODO REMOVE
    n_center = - (x_t - (obs.x_center.*obs.a));
else
    n_center = - x_t;
end

E(:,1) = n_center;
E(1,2:dim) = nv(2:dim)';
E(2:dim,2:dim) = -eye(dim-1)*nv(1);

if dim == 3
    E(:,end+1) = [0;-nv(3);nv(2)];
end

% Importance influence of rotation: 1 on surface, 0 far away
p_h = 1;
h_weight = (1/Gamma)^p_h;

if dim ==2
    tang = E(:,2);
    
    % Get direction of rotation -- true???
    clockWise = cross([n_center;0],[obs.dx_center;0]);
    clockWise = clockWise(3) > 0;
    
    % Maximum Rotation Angle to have ds at center align with 
    if ~ clockWise
        tang = -tang;
    end
    crossP = cross([obs.dx_center;0],[tang;0]);
    rotAng_max = acos(sum(tang'/norm(tang)*obs.dx_center/norm(obs.dx_center)))...
                        *sign(crossP(3));
    maxAng_deg = rotAng_max*180/pi;
    
    % Rototation Matrix
    rotMat = compute_R(dim, rotAng_max*h_weight);
end

end

function [x_hat] = compute_replacement_position(dim, x_t, obs, R, Gamma)




% Distance to center of obstacle
r_x = norm(x_t);

if dim ==2
    phi_x = atan2(x_t(1),x_t(2));

    % Calculate obstacle's radius r_obs
    x_obs(1) = obs.a(1,:).*cos(phi_x);
    x_obs(2) = obs.a(2,:).*sign(phi_x).*(1 - cos(phi_x).^(2.*obs.p(1,:))).^(1./(2.*obs.p(2,:)));
end

r_obs = norm(x_obs);

%delta_r = r_obs*(1-1/Gamma);
p =1;
delta_r = r_obs*(1/Gamma)^p;

% Calculate now center
x_hat = (r_x-delta_r)/r_x*x_t;

% Move x_hat to original coordinate system
x_hat = R'*x_hat + obs.x0;

end

% function w = compute_weights(Gamma,N)
% w = zeros(1,N);
% Gamma(Gamma<1) = 1;
% Gamma = Gamma-1;
% for i=1:N
%     ind = 1:N;
%     ind(i) = [];
%     w(i) = prod(Gamma(ind)./(Gamma(i)+Gamma(ind)));
% end

