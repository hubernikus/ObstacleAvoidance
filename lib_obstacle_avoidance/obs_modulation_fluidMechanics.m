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
% 
%     S.M. Khansari Zadeh and A. Billard, "A Dynamical System Approach to
%     Realtime Obstacle Avoidance", Autonomous Robots, 2012
%
%%
N = length(obs); %number of obstacles
d = size(x,1);
Gamma = zeros(1,N);

% xd_dx_obs = zeros(d,N);
% xd_w_obs = zeros(d,N); %velocity due to the rotation of the obstacle
% 
% %adding the influence of the rotational and cartesian velocity of the
% %obstacle to the velocity of the robot
% xd_obs = 0;
% for n=1:N
%     if ~isfield(obs{n},'sigma')
%         obs{n}.sigma = 1;
%     end
%     xd_obs = xd_obs + w(n) * exp(-1/obs{n}.sigma*(max([Gamma(n) 1])-1))* ... %the exponential term is very helpful as it help to avoid the crazy rotation of the robot due to the rotation of the object
%                                        (xd_dx_obs(:,n) + xd_w_obs(:,n)); 
% end
i=1;
if(isfield(obs{i},'perturbation'))
    xd_obs = obs{i}.perturbation.dx;
else
    xd_obs = 0;
end

xd = xd  - xd_obs; %computing the relative velocity with respect to the obstacle
%xd = xd + randn(2,1)*1e-10 - xd_obs; %computing the relative velocity with respect to the obstacle

modulation = 'ellipse';
switch modulation
    case 'ellipse'
        i = 1; 
        xd = deflectionEllipse(obs{i}, x, xd);
    case 'circle'
        i = 1;
        R = obs{i}.a(1)*obs{i}.sf(1);
        x0 = obs{i}.x0;
        
        xd = deflectionOfInitSys(R, x0, x, xd);
    case 'random'
        fprintf('not done')
    otherwise
        fprinft('case not defined \n');
end



xd = xd + xd_obs; %transforming back the velocity into the global coordinate system

end

function [xd] = deflectionEllipse(obs, x, xd)
th_r = obs.th_r;

% Recenter 
x = x- obs.x0;

% Rotate 
R = [cos(th_r),-sin(th_r);
     sin(th_r),cos(th_r)];
 
% Stretch  
A = obs.sf(1)*diag([obs.a(1),obs.a(2)]);
trafoMat = pinv(A)*R';

% Rotation of everything
x = trafoMat*x;
xd = trafoMat*xd;

xd = deflectionOfInitSys(1, [0;0], x, xd);

% Stretch 
% Rotate

xd = R*A* xd;

end


function [xd] = deflectionOfInitSys(R, x0, x, xd)
%x_bar = x-x0;
x_bar = x;
R=1;
r = sqrt(sum(x_bar.^2));
cosTheta = x_bar(1)/r;
sinTheta = x_bar(2)/r;

R_theta = [cosTheta, -sinTheta;
           sinTheta, cosTheta];
       
u_rTheta = R_theta' *xd;

u_rTheta = [(1-R^2/r^2);(1+R^2/r^2)].*u_rTheta;
if(r<R)
    u_rTheta = [R*0.1;0];
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