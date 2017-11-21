function [xd, phi_c, h_x, kappa, velTowardsBody] = obs_modulation_rotation(x,xd,obs,xd_obs, plotFigure)
%
% Obstacle avoidance module: Version 1.1, issued on March 26, 2012
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
if  nargin < 5
    plotFigure = false; 
end

N = length(obs); %number of obstacles
d = size(x,1);
Gamma = zeros(1,N);

if(N>1)
    warning('Module needs to be updated for more than 1 Object!!!')
end

if(d~=2)
    warning('Module only for 2D')
end

xd = xd-xd_obs; %computing the relative velocity with respect to ALL obstacle

n = 1; 
virtObs{1} = obs{1};
[x_obs, x_obs_sf] = obs_draw_ellipsoid(virtObs,50); 

% COMMENT: smallest deviation not good idea, as it does not converge
% already for simple system ( smallest rotation mode..)
% 2nd try: stay on CM of object
[phi_max, phi_mid, deltaPhi, phi_CM] = findMaxAngle(virtObs{1}.x0, x_obs_sf, x);

[min_dist, phi_minDist] = findMinDistance(x_obs_sf, x);   

phi_xd = atan2(xd(2),xd(1)); % direction of the TI DS

if plotFigure
    % Illustaration remove
    figure(11);
    patch(x_obs(1,:,n),x_obs(2,:,n),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6]);
    hold on;
    plot(x_obs_sf(1,:,n),x_obs_sf(2,:,n),'k--','linewidth',0.5);

    lineLength = 10;
    plot(x(1),x(2), 'rx')
    plotLine(x, phi_max(1), lineLength)
    plotLine(x, phi_max(2), lineLength)
    plotLine(x, phi_mid, lineLength)
    plotLine(x, phi_CM, lineLength,'y')
    plotLine(x, phi_minDist, min_dist,'g')
    plotLine(x, phi_xd, min_dist,'b')
    
end


% Find direction of minimal rotation -> least rotation of the velocity
%[~,indMinDir] = min(abs(dirAngleDiff(phi_xd,phi_max)));
%dirRotation = dirAngleDiff(phi_max(indMinDir),phi_xd); 
dirRotation = (dirAngleDiff(phi_xd, phi_CM) > 0 );


% Angle the velocity is already rotated in the direction of closest exit
%rotationVelocity = dirAngleDiff(phi_xd,phi_mid); 
rotationVelocity = dirAngleDiff(phi_xd,phi_CM); 

% Maximum rotation angle (limits over rotation > zick-zack)
phi_c = (2*dirRotation-1)*pi - rotationVelocity;

%
% Relative Magnitude of Rotation --- h_r
epsilon_v = 0.001; % velocity safety margin
n_minDist = [cos(phi_minDist);sin(phi_minDist)]; % normal vector in direction of the minimal distance
dist0 = 6;
h_x0 = 4; % > 1 --- Inverse default gain
d_phiVel = 0.1; % Angle margin, with which the object is avoided

% COMMENT: Velocity rotation does not work, only based on the realtive
% velocity towards the body. This is to concervative and additionally
% velocities behind the bodies are affected to much. 
% velGain = max(epsilon_v + dot(n_minDist, xd),realmin);

% % Check eter point is moving towards object: 1 if attacking body, 0 if
% going away
velTowardsBody = and((dirAngleDiff(phi_xd,phi_max(1))>0), ...
                        (dirAngleDiff(phi_max(2),phi_xd)>0));

% angle needed to exit in the direction of rotatoin

% Local importance of Rotation
% if(velGain > 10*realmin)
%     h_x = n0*n_gain^(- 1/velGain*min_dist/dist0);
% else
%     h_x = 0;
% end
h_x = h_x0^(-1*min_dist/dist0)*velTowardsBody* ...
                  abs(angleSubtraction(phi_max(2-dirRotation),phi_xd));
% h_x = h_x % diplay h_x

%
% Rotation Matrix --- R
R = compute_R(2,phi_c*h_x);

%
% Change in speed --- kappa = (1+kappa_old)
kappa0 = 2; % default gain
phi0 = pi; % no bais if =pi, otherwise towards one side
d0 = 1; % reference distance, the bigger the stronge the distance effect
nd = 1; % distance exponent, the bigger, the stronger the distance effect

kappa = kappa0 ^ ((2*pi-deltaPhi)/phi0 * (d0/d)^nd);
%kappa = 1;
M = kappa*R;

if plotFigure; plot([x(1),x(1)+xd(1)],[x(2),x(2)+xd(2)],'r'); end

xd = M*xd; % velocity modulation

if plotFigure; plot([x(1),x(1)+xd(1)],[x(2),x(2)+xd(2)],'m'); end

xd = xd + xd_obs ; % transforming back the velocity into the global coordinate system

end

function plotLine(x0, phi, d,col)
    if nargin>3
        plot([x0(1),x0(1)+d*cos(phi)],[x0(2), x0(2)+d*sin(phi)], '--','Color', col)
    else
        plot([x0(1),x0(1)+d*cos(phi)],[x0(2), x0(2)+d*sin(phi)], 'k--')
    end
    
end


function dPhi = dirAngleDiff(alpha, beta)
% Caluclates the minimal difference between the angles including sign!
% negative -> beta in direction counterclock of alpha
% positive -> alpha in direction counterclock of beta
    diff = alpha-beta;
    dPhi = diff +  2*pi*(diff<-pi) - 2*pi*(diff > pi);
%     if(diff < 0)
%         if(diff > -pi)
%             dPhi = diff;
%         else
%             dPhi = diff+2*pi;
%         end
%     else
%         if(diff < pi)
%             dPhi = diff;
%         else
%             dPhi = diff-2*pi;
%         end
%     end
%     if(diff-diff2) % if no warning during testing, can be removed
%         warning('Error in calculation of diff2')
%     end
end


function dPhi = angleDiff(alpha, beta)
% Caluclates the minimal difference between the angles

% Enable scalar & vector compability
Na = legnth(alpha);
Nb = length(beta);
if(and(Na>1,Nb>1, Na~=Nb))
    error('Dimension missmatch')
end

% loop of angle difference
for i=1:max(Na,Nb)
    d1 = alpha(min(i,Na))-beta(min(i,Nb));
    if(d1 < 0)
        dPhi = min(d1+2*pi,-d1); 
    else
        dPhi = min(-d1+2*pi, d1);
    end
end
end

function dPhi = angleSubtraction(alpha, beta)
% subtracts to angles intor range -pi to pi
    dPhi = alpha-beta;
    dPhi = dPhi+(dPhi<-pi)*2*pi-(dPhi>pi)*2*pi;
end

function dPhi = angleSubtraction360(alpha, beta)
% subtracts two angles, range into range between 0 to 2pi
    dPhi = alpha-beta;
    dPhi = dPhi+(dPhi<0)*2*pi;
end

function dPhi = addAngle(alpha, beta)
% subtracts to angles and 
    if nargin<2 beta=0; end
   
    dPhi = alpha+beta;
    while(or(dPhi > pi, dPhi < - pi))
        dPhi = dPhi+(dPhi<-pi)*2*pi-(dPhi>pi)*2*pi;
    end
end

function [phi_max, phi_mean, delta_phi, phi_CM] = findMaxAngle(x_obs_ref, x_obs_sf, refPoint)
% find the most extreme angles
% if phi_mid < 0 -> concave
    phi_CM = atan2(x_obs_ref(2)-refPoint(2),x_obs_ref(1)-refPoint(1));
    
    phi = atan2(x_obs_sf(2,:)-refPoint(2),x_obs_sf(1,:)-refPoint(1));

    % Find largest gap -> there is the opening of concave/convex object
    % sort points according to phi
    phi_sort = zeros(1,length(phi));
    phi_rest = phi; % remaining items
    
    [phi_sort(1),ind_min] = min(phi_rest);
    phi_rest(ind_min) = [];
    
    for n = 2:length(phi)-1
        for i = 2:length(phi_rest)
            % Normal subtraction, cause minimal angle was startet with
            nextAngle = phi_rest(1)-phi_sort(n); 
            ind_next = 1;
            if(nextAngle > phi_rest(i)-phi_sort(n))
                nextAngle(i) = phi_rest(i)-phi_sort(n);
                ind_next = i;
            end
        end
        phi_sort(n) = phi_rest(ind_next);
        phi_rest(ind_next) = [];
    end
    phi_sort(end) = phi_rest; 
    lastAngle = angleSubtraction360(phi_sort(1),phi_sort(end));
    deltaPhi = [phi_sort(2:end)-phi_sort(1:end-1), lastAngle];
                        
    [valMax,indMax] = max(deltaPhi);
    
    % vis-a-vis of the largest opening is the center of the object
    phi_mean = addAngle(phi_sort(indMax)+0.5*valMax+pi); 
        
    phiCentered = angleSubtraction(phi,phi_mean);
    phi_max = [min(phiCentered), max(phiCentered)];
    delta_phi = angleSubtraction(phi_max(2), phi_max(1));
    
    phi_max = addAngle(phi_max, phi_mean); % return in original reference frame
end


function [min_dist,phi_minDist] = findMinDistance(x_obs_sf, refPoint)
% find the minimum distance and correpsonding angle
    [min_dist, indMin] = min(sum((x_obs_sf-refPoint).^2,1),[],2);
    
    % Eucledian distance
    min_dist = sqrt(min_dist);
    
    phi_minDist = atan2(x_obs_sf(2,indMin)-refPoint(2),x_obs_sf(1,indMin)-refPoint(1));
end

function [max_dist,phi_maxDist] = findMaxDistance(x_obs_sf, refPoint)
% find the minimum distance and correpsonding angle
    [max_dist, indMin] = max(sum((x_obs_sf-refPoint).^2,1),[],2);
    
    % Eucledian distance
    max_dist = sqrt(max_dist);
    
    phi_maxDist = atan2(x_obs_sf(2,indMin)-refPoint(2),x_obs_sf(1,indMin)-refPoint(1));
end

function R = compute_R(d,th_r)
% rotating the query point into the obstacle frame of reference

if d == 2 
    R = [cos(th_r(1)) -sin(th_r(1));sin(th_r(1)) cos(th_r(1))]; % why niegtive...
elseif d == 3
    warning('Rotation is around x-axis... no better option?')
    R_x = [ 1, 0, 0; 0, cos(th_r(1)), sin(th_r(1)); 0, -sin(th_r(1)), cos(th_r(1))];
    R_y = [cos(th_r(2)), 0, -sin(th_r(2)); 0, 1, 0; sin(th_r(2)), 0, cos(th_r(2))];
    R_z = [cos(th_r(3)), sin(th_r(3)), 0; -sin(th_r(3)), cos(th_r(3)), 0; 0, 0, 1];
    R = R_x*R_y*R_z;
else %rotation is not yet supported for d > 3
    warning('NO rotation matrix implemented for higher dimensions...')
    R = eye(d);
end

end