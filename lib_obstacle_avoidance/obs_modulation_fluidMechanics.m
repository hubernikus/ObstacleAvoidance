function [xd b_contour M] = obs_modulation_fluidMechanics(x,xd,obs,b_contour,varargin)
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
%       [xd b_contour M] = obs_modulation_ellipsoid(x,xd,obs,b_contour,xd_obs)
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
%   o b_contour: A boolean indicating whether the algorithm is in the
%                contouring stage or not.
%
%   o xd_obs:    d x 1 column vector defining the obstacle velocity
%
% Outputs ----------------------------------------------------------------
%
%   o xd:        d x 1 column vector corresponding to the modulated
%                robot velocity.
%
%   o b_contour: A boolean indicating whether the algorithm is in the
%                contouring stage or not.
%
%   o M:         d x d matrix representing the dynamic modulation matrix.
% 
% 
% This code is writen based on the following paper:
%     S.M. Khansari Zadeh and A. Billard, "A Dynamical System Approach to
%     Realtime Obstacle Avoidance", Autonomous Robots, 2012
%
%%
% To try:
% Two objects -- litterature
% Multibple objects -- litterature
% intersection objects -- literature
% Convex objects -- literature
% Convex objects -- space transformation... Airfoil.
%
%

N = length(obs); %number of obstacles
d = size(x,1);

xd_obs = zeros(2,N);
w_obs = zeros(2,N);

for i=1:length(varargin)
    if ~isempty(varargin)
        switch i
            case 1
                xd_obs = varargin{1};
            case 2
                w_obs = varargin{2};
        end
    end
end

% Find sort object descending distance
dist2= zeros(1,N);

for ii = 1:N
    dist2(ii) = sum((x-obs{ii}.x0).^2);
end
[dist,indDist] = sort(dist2,'descend');



for ind = indDist
    
    % TODO; object and rotation opposit... what doo..?!?!?
    % Only include relative veloctiy if its moving towards object
    if(and(dot(xd_obs(:,ind), (x-obs{ind}.x0)) > 0 ,... % obstacle moving towards
            norm(xd_obs(:,ind)) > norm(xd))) % not moving faster than object -- moving into it 
        xd_obsFrame = xd_obs(:,ind);
    else
        xd_obsFrame = zeros(2,1);
    end
    
    %xd_obsFrame = xd_obs(:,ind);
    xd = xd  - xd_obsFrame; % transformation into velocity frame
    
    xd = xd + randn(2,1)*1e-10; % Adding little noise, for instablities
    
    if(isfield(obs{ind},'concaveAngle'))
        xd = spaceTrafo_concaveCirce(obs{ind}, x, xd, w_obs(ind));
    else
        xd = spaceTrafo_ellipseCircle(obs{ind}, x, xd, w_obs(ind));
    end
    
    xd = xd + xd_obsFrame; % transforming from velocity to inertial frame
    %xd_fin = xd
end


end


function [xd] = spaceTrafo_concaveCirce(obs, x, xd, w_obs)

% Recenter 
x = x - obs.x0;

% Rotation Matrix
dim = 2;
if(isfield(obs, 'th_r'))
    R = rotMatrix(obs.th_r, dim);
else
    R = eye(dim);
end

% Rotatational rate to modify velocity
if(w_obs)
        % Rotation from rTheta -> xy
        R_pos = rotMatrix(x,2);

        dist = norm(x);
        
        I = [0;1]; 
        
        % xd_w = R_pos * I*(w_obs*dist*safety) -- expected correct!
        xd_w = R_pos * I*(w_obs*dist);
        
            % Only include relative veloctiy if its moving towards object
        if( and( dot(xd_w, x) > 0, ... % obstacle moving towards robo
                norm(xd_w) > norm(xd) ) )% not moving faster than object -- moving into it 
             xd = xd - xd_w;
        else 
            xd_w = zeros(2,1);
        end
else
    xd_w = 0;
end

% Transformation of space (Ellipse to unit circle)
A = obs.sf(1)*diag([obs.a(1),obs.a(2)]);
invA = pinv(A);

% Rotation of everything
x = R' * x;
%x = ellipsUnfold(x,[0;0],obs.concaveAngle);
x = invA*x;

xd = R' * xd;
xd = ellipsUnfold(xd,[0;0],obs.concaveAngle)
xd = invA*xd;

% Applyt deflction on Unit Circle 
xd = deflectionUnitCircle(x, xd);

% Move back to original plane [Stretch - Rotate]
xd = A *xd;
xd = ellipsFold(xd,[0;0],obs.concaveAngle);
xd = R *xd;


xd = xd + xd_w; % Remove velocity due tro rotation

end


function [xd] = spaceTrafo_ellipseCircle(obs, x, xd, w_obs)

% Recenter 
%<<<<<<< HEAD
% x = x- obs.x0;
% 
% % Rotate 
% R = [cos(th_r),-sin(th_r);
%      sin(th_r),cos(th_r)];
%  
% % Stretch  
% =======
x = x - obs.x0;

% Rotation Matrix
dim = 2;
if(isfield(obs, 'th_r'))
    R = rotMatrix(obs.th_r, dim);
else
    R = eye(dim);
end

% Rotatational rate to modify velocity
if(w_obs)
        % Rotation from rTheta -> xy
        R_pos = rotMatrix(x,2);

        dist = norm(x);
        
        I = [0;1];
        % xd_w = R_pos * I*(w_obs*dist*safety) -- expected correct!
        xd_w = R_pos * I*(w_obs*dist);
        xd = xd - xd_w;
else
    xd_w = 0;
end

% Transformation of space (Ellipse to unit circle)
%>>>>>>> 8e49294e1a0778f6697a16446a6834dcfc80b31a
A = obs.sf(1)*diag([obs.a(1),obs.a(2)]);
trafoMat = pinv(A)*R';

% Rotation of everything
x = trafoMat*x;
xd = trafoMat*xd;

% Applyt deflction on Unit Circle 
xd = deflectionUnitCircle(x, xd);

% Move back to original plane [Stretch - Rotate]
xd = R*A* xd;

xd = xd + xd_w; % Remove velocity due tro rotation

end

function [xd] = deflectionUnitCircle(x, xd)
% Unit radius (R = 1), Centerd x0 =[0,0]

r = sqrt(sum(x.^2));
R_theta = rotMatrix(x,2);

u_rTheta = R_theta' *xd;
% M = diag([(1 - 1/r^2),(1 + 1/r^2)]); % initial fluid dynamics
M = diag([(1 - 1/r^2),(1)]);

u_rTheta = M*u_rTheta;
if(r<1)
    u_rTheta = [1;0];
end

xd = R_theta*u_rTheta;

end

function [xd] = source(Q, x0, x)
% Sink for Q < 0 
x_bar = x-x0;
r = sqrt(sum(x_bar.^2));
cosTheta = x_bar(1)/r;
sinTheta = x_bar(2)/r;

u_r = Q/(2*pi*r);
if(r<R) % remove arrows in circle
    u_r = 0;
end
u_phi = 0;

xd = [sinTheta*(u_r+u_phi);
      cosTheta*(u_r-u_phi)];

end

function [xd] = vortex(Gamma, x0, x)
% Gamma>0 - right hand turn
% Cricital: 4*pi*a*U

x_bar = x-x0;

r = sqrt(sum(x_bar.^2));
% Trigonometric relations
cosTheta = x_bar(1)/r;
sinTheta = x_bar(2)/r;

% Angle
Theta = asin(sinTheta);

u_r = -Gamma/(2*pi)*log(r);
u_phi = Gamma/(2*pi)*Theta;

xd = [sinTheta*(u_r+u_phi);
      cosTheta*(u_r-u_phi)];
end

function [xd] = rankineOval(h,a,x0,x)
xd = [0;0];
end

function [R] = rotMatrix(angleInp, dim)
if(nargin<2) 
    dim = 2; % Default dimension
end


if(length(angleInp) == 1)
    theta = angleInp; % angle is given
    % Trigonemetric function evaluation
    cosTheta = cos(theta);
    sinTheta = sin(theta);
elseif(length(angleInp) == 2) % Angle input is 2D vector
    x = angleInp;
    dist = norm(x);
    if (dist) % bigger than 0
        cosSinTheta = x/dist;
        cosTheta = cosSinTheta(1);
        sinTheta = cosSinTheta(2);
    else
        % Unit matrix
        cosTheta = 1;
        sinTheta = 0;
    end
else % Define 3D .. - quaternions?
    fprintf('not defined \n')
end

% Create Roation matrix
if (dim == 2)
    R = [cosTheta, -sinTheta;
         sinTheta, cosTheta];
else
    fprintf('rotMat of dimension=%d no implemented \n', dim);
    R = eye(dim);
end
end


function x = ellipsUnfold(x, x0, concaveAng)
x = x-x0; % center

normX = sqrt(sum(x.^2,1));

phi = atan2(x(2,:), x(1,:))
%phi = phi - (phi>pi)*2*pi

phi_prime = unfoldFunction_lin(phi, concaveAng); % apply fold

x = [cos(phi_prime);sin(phi_prime)].*normX;

x = x +x0; % move initial posiiton 

end


function phi = unfoldFunction_lin(phi, concaveAng)
absPhi = abs(phi);

for i = 1:length(absPhi)
    if absPhi(i) < concaveAng/2
        m = (absPhi(i)*concaveAng/pi);
        phi(i) = sign(phi(i))./m;
    else
        m = (2-concaveAng/pi);
        q = (concaveAng-pi);
        phi(i) = sign(phi(i)).*(1/m*(absPhi(i)-q));
    end
end
end

function x = ellipsFold(x, x0, concaveAng)
x = x-x0; % center

normX = sqrt(sum(x.^2,1));

phi = atan2(x(2,:), x(1,:));
%phi = phi - (phi>pi)*2*pi

phi_prime = foldFunction_lin(phi, concaveAng); % apply fold

x = [cos(phi_prime);sin(phi_prime)].*normX;

x = x +x0; % move initial posiiton 

end


function phi = foldFunction_lin(phi, concaveAng)
absPhi = abs(phi);

for i = 1:length(absPhi)
    if absPhi(i) < pi/2
        m = (absPhi(i)*concaveAng/pi);
        phi(i) = sign(phi(i)).*m;
    else
        m = (2-concaveAng/pi);
        q = (concaveAng-pi);
        phi(i) = sign(phi(i)).*(m*absPhi(i)+q);
    end
end

end