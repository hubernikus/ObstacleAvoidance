function DS_fluidDynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%          Semester Project - HUBER Lukas, LASA Lab, EPFL  
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

clc; close all; clear variables;
rng(1) % Set seeed for repeatability of simulation

%% preparing the obstacle avoidance module

%adding the obstacle avoidance folder to the MATLAB path directories
if isempty(regexp(path,['lib_obstacle_avoidance' pathsep], 'once'))
    addpath([pwd, '/lib_obstacle_avoidance']);
end
%adding the example folder to the MATLAB path directories
if isempty(regexp(path,['DynamicalSystems' pathsep], 'once'))
    addpath([pwd, '/DynamicalSystems']);
end
if isempty(regexp(path,['lib_simulation_tools' pathsep], 'once'))
    addpath([pwd, '/lib_simulation_tools']);
end

% Set default simulation parameters
opt_sim.dt = 0.01; %integration time steps
opt_sim.i_max = 300; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
aoarpt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined


%%
close all

% obstacle 1
obs1{1}.a = [0.8;3];
%obs1{1}.a = [3; .9];
%obs1{1}.a = [1;1];
obs1{1}.p = [1;1];
obs1{1}.x0 = [0;0];
obs1{1}.sf = [1.;1.];
%obs1{1}.th_r = 0*pi/180;
%obs1{1}.concaveAngle = pi*0.4; % in [0,pi]
obs1{1}.concaveAngle = pi*0.4; % in [0,pi]

xd_obs = [0;0];
xd_max = 2;
w_obs = 0;
x0 = obs1{1}.x0;

N_init = 10
xInit = [repmat([3],1,N_init);linspace(-3,3,N_init)];
xInit = [2;1];


[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs1,101);

fig_elli = figure('Position',[1250,100,600,600]);
% Draw object
patch(x_obs(1,:),x_obs(2,:),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6],'FaceAlpha', 0.4); hold on;
plot(x_obs_sf(1,:),x_obs_sf(2,:),'k-x','linewidth',0.5);
axis equal; grid on;
plot(obs1{1}.x0(1),obs1{1}.x0(2), 'ro'); hold on;

x0 = [-2;0];
% 


% %xd_fluid= obs_modulation_fluidMechanics(x0,xd_hat, obs1, 0, xd_obs,w_obs);
% 
% plot(x0(1), x0(2), 'kx')
% xd_hat = xd_hat*0.5;
% plot([x0(1),x0(1)+xd_hat(1)],[x0(2),x0(2)+xd_hat(2)],'k--')
% %xd_fluid = xd_fluid*0.5;
% plot([x0(1),x0(1)+xd_fluid(1)],[x0(2),x0(2)+xd_fluid(2)],'r')


% Unit circle
N_circ = 50;
phi = linspace(0,2*pi,N_circ);
xCic = cos(phi);
yCirc = sin(phi);

fig_circ = figure('Position',[50,100,600,600]);
patch(xCic,yCirc,[0.6 1 0.6], 'FaceAlpha', 0.3); hold on;
axis equal;
%xd_fluid= obs_modulation_fluidMechanics(x0,xd_hat, obs1, 0, xd_obs,w_obs);
grid on;

%fig_elliUnfold = figure('Position',[50,100,600,600]);

fig_elliUnfold = figure('Position',[650,100,600,600]);
a = obs1{1}.a
patch(xCic*a(1),yCirc*a(2),[0.6 1 0.6], 'FaceAlpha', 0.3); hold on;
grid on; axis equal;

%% %%%%% Loooop %%%%%
dt = 0.05;
iter = true;

%xInit = [0.85;0.1];
%for ii = 1:3
    
xInit = [4;1];
for ii = 1:50
    
    x = xInit(:,ii);
    xd_hat = stableDS(x,x0);
    
    figure(fig_elli);
    %plotArrow(x,xd_hat*0.3,'k');
    plot(x(1),x(2),'x')
    %plotArrow(x,xd_hat*0.3,'k')
    
    figure(fig_circ);  % circle plot
    obs = obs1{1};
    xd = xd_hat;

    % Recenter
    x = x - obs.x0;
    

    % Rotation Matrix
    dim = 2;
    if(isfield(obs, 'th_r'))
        R = rotMatrix(obs.th_r, dim);
    else
        R = eye(dim);
    end
    
    % Transformation of space (Ellipse to unit circle)
    A = obs.sf(1)*diag(obs.a);
    invA = pinv(A);

    % Rotation of everything
    x = R' * x;
    %x = ellipsUnfold(x,[0;0],obs.concaveAngle);
    % Applyt deflction on Unit Circle
    
    
    xd = R' * xd;
    %plotArrow(x,xd*0.3,'g')
    [xd,xFold] = unfoldVector(xd,xInit(:,ii),obs.concaveAngle);
    xd_temp = foldVector(xd,xFold,obs.concaveAngle);
    %xFold = x

    
    figure(fig_elliUnfold);
    plot(xFold(1), xFold(2),'x')
    %plotArrow(xFold, xd)
    %xdEllipse = xd

    x = xFold;
    x = invA*x;
    
    phi = atan2(x(2),x(1));
    

    figure(fig_circ);  % circle plot
    xd = invA*xd;
    plotArrow(x,xd*0.3,'b')
    
    % Normal
    normal = [cos(phi);sin(phi)];
    %plot([x(1),x(1)+normal(1)],[x(2),x(2)+normal(2)],'b')
    dim = 2;

    % Normal vector due to rotation
    phi_prime = unfoldFunction_lin(phi, obs.concaveAngle); % apply fold
    %phi_primeDeg = phi_prime*90;
    rotMat = rotMatrix(phi_prime-phi, dim);
    normal = rotMat*normal;
    plot([x(1),x(1)+normal(1)],[x(2),x(2)+normal(2)],'b--')

    % Tangent
    tangent = [-sin(phi);cos(phi)];
    plot([x(1),x(1)+tangent(1)],[x(2),x(2)+tangent(2)],'r--')

    D = [normal, tangent];

%     plotArrow(x,xd*0.3,'k--')
    
    xd = D\xd;
    %xd = xd*
    
    [xd,xd_hat, distCircle] = deflectionUnitCircle(x, xd);
    
    %plotArrow(x(1),x(2),'kx')

    xd = D*xd;
    plotArrow(x,xd*0.3,'r');
    %plotArrow(x,xd*0.3,'k')
    %plotArrow(x,xd_hat*0.3,'k--')
    
    %xd2 = deflectionUnitCircle(x, xd);

    % Move back to original plane [Stretch - Rotate]
    xd = A *xd;
    
    figure(fig_elliUnfold)
    plotArrow(xFold,xd*0.3,'r')
    
    %fprintf('Now wrong angle \n')
    xd = foldVector(xd,xFold,obs.concaveAngle);

    %xd = ellipsFold(xd,[0;0],obs.concaveAngle);
    xd = R *xd;
    
    
    if xd_max<norm(xd)
        xd = xd*xd_max/norm(xd);
    end
    %xdFinal = xd
    %xd = xd + xd_w; % Remove velocity due tro rotation
    %M = diag(1-1/distCircle^2, 1);
    
    %D =
    %xd = xd 

    figure(fig_elli);
    plotArrow(xInit(:,ii),xd_hat*0.3,'k'); hold on;
%     plotArrow(xInit(:,ii)+[0.1;0],xd*0.3,'r')
    plotArrow(xInit(:,ii),xd*0.3,'r');
    
    if iter
        xInit(:,ii+1) = xInit(:,ii) + dt*xd;
    end
end
figure(fig_elli);
axis equal;
    

end

function x = ellipsUnfold(x, x0, concaveAng)
x = x-x0; % center

normX = sqrt(sum(x.^2,1));

phi = atan2(x(2,:), x(1,:));
%phi = phi - (phi>pi)*2*pi

phi_prime = unfoldFunction_lin(phi, concaveAng); % apply fold

x = [cos(phi_prime);sin(phi_prime)].*normX;

x = x +x0; % move initial posiiton

end

function [xd,xFold] = unfoldVector(xd, x, concavAng)
phi = atan2(x(2,:), x(1,:)); % Caclulate position

phi_prime = unfoldFunction_lin(phi, concavAng); % apply fold

dim = 2; % Implemented only for 2d....
rotMat = rotMatrix(phi_prime-phi, dim);
%dPhiDeg_unfold = (phi_prime-phi)*180/pi
rotMat = rotMat;
% Apply unfold-rotation
xd = rotMat*xd;

xFold = rotMat*x;

end

function xd = foldVector(xd, xFold, concavAng)
phi = atan2(xFold(2,:), xFold(1,:)); % Caclulate position

phi_prime = foldFunction_lin(phi, concavAng); % apply unfold

dim=2; % only for 2d so far...
rotMat = rotMatrix(phi_prime-phi, dim);
%PhiDeg_fold = (phi_prime-phi)*180/pi;
rotMat = rotMat;
% Apply fold-rotation
xd = rotMat*xd;
end

function phi = unfoldFunction_lin(phi, concaveAng)
% Concave angle between pi and 0
halfConcave = concaveAng/2;
%halfConcaveDeg = halfConcave*180/pi

absPhi = abs(phi);
%phiDegI = absPhi*180/pi

for i = 1:length(absPhi)
    if absPhi(i) < concaveAng/2
        %m = (absPhi(i)*concaveAng/pi);
        %phi(i) = sign(phi(i))./m;
        %phi(i) = sign(phi(i))*absPhi/halfConcave*pi/2;
        phi(i) = sign(phi(i))*absPhi/concaveAng*pi;
    else
        %m = (2-concaveAng/pi);
        %q = (concaveAng-pi);
        %phi(i) = sign(phi(i)).*(1/m*(absPhi(i)-q));
        phi(i) = sign(phi(i)).*((absPhi-halfConcave)./(pi-halfConcave)*pi/2+pi/2);
    end
end
%phiDegO = phi(i)*180/pi
end

function x = ellipsFold(x, x0, concaveAng)
x = x-x0; % center

normX = sqrt(sum(x.^2,1));

phi = atan2(x(2,:), x(1,:));
%phi = phi - (phi>pi)*2*pi

phi_prime = foldFunction_lin(phi, concaveAng); % apply fold

x = [cos(phi_prime);-sin(phi_prime)].*normX;

x = x +x0; % move initial posiitonq

end


function phi = foldFunction_lin(phi, concaveAng)
absPhi = abs(phi);
%phiDeg = absPhi*180/pi

for i = 1:length(absPhi)
    if absPhi(i) < pi/2
        %m = (absPhi(i)*concaveAng/pi);
        phi(i) = sign(phi(i)).*absPhi*concaveAng/(pi);
    else
        %m = (2-concaveAng/pi);
        %q = (concaveAng-pi);
        %phi(i) = sign(phi(i)).*(m*absPhi(i)+q);
        phi(i) = sign(phi(i)).*((absPhi-pi/2)./(pi/2)*(pi-concaveAng/2)+concaveAng/2);
    end
end

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



function [xd,xd_hat,r] = deflectionUnitCircle(x, xd)
% Unit radius (R = 1), Centerd x0 =[0,0]
xd_hat = xd;
r = sqrt(sum(x.^2));
%R_theta = rotMatrix(x,2);
R_theta = eye(2);

u_rTheta = R_theta' *xd;
%M = diag([(1 - 1/r^2),(1 + 1/r^2)]); % initial fluid dynamics
M = diag([(1 - 1/r^2),(1)]);

u_rTheta = M*u_rTheta;
if(r<1)
    u_rTheta = [1;0];
end

xd = R_theta*u_rTheta;

end

function plotArrow(x, dx, col)
if nargin<3
    col = 'k';
end

if nargin<4
    lw = 2;
end

%plot([x(1), x(1)+dx(1)],[x(2), x(2)+dx(2)],col, 'LineWidth',lw)
%plot(x(1),x(2),'LineWidth',lw)
quiver(x(1),x(2), dx(1),dx(2), col)

end