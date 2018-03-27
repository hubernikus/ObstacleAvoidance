%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%       Safety region Drawing - 
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; close all; clear all;

rotCol = [0.6 0.0 0];

lw = 3;
saveFig = false;
N_circResol = 50;

%% preparing the obstacle avoidance module

%adding the obstacle avoidance folder to the MATLAB path directories
if isempty(regexp(path,['lib_obstacle_avoidance' pathsep], 'once'))
    addpath([pwd, '/lib_obstacle_avoidance']);
end
%adding the example folder to the MATLAB path directories
if isempty(regexp(path,['DynamicalSystems' pathsep], 'once'))
    addpath([pwd, '/DynamicalSystems']);
end
% adding simulation tool-folder to path
if isempty(regexp(path,['lib_simulation_tools' pathsep], 'once'))
    addpath([pwd, '/lib_simulation_tools']);
end

%% Transition rotating ellipse
w_transReg = 0.4;

margin = 1

it_obs=1;
obs{it_obs}.a = [1.8;0.8];
obs{it_obs}.p = [1;1];
obs{it_obs}.x0 = [0;0];
obs{it_obs}.sf = [1.2];
obs{it_obs}.th_r = -60*pi/180;
w_obs = [1];

[x_obs_boundary, x_obs_sf] = obs_draw_ellipsoid(obs,50);

r1 = obs{it_obs}.sf(1)*max(obs{it_obs}.a);
r2 = r1+w_transReg;

phi = linspace(0,2*pi,N_circResol);

boundary_1 = [r1*cos(phi);r1*sin(phi)]; 
boundary_2 =  [r2*cos(phi);r2*sin(phi)];


xplotLim = [min(boundary_2(1,:)), max(boundary_2(1,:))];
yplotLim = [min(boundary_2(2,:)), max(boundary_2(2,:))];
xplotLim = xplotLim + [-margin,margin];
yplotLim = yplotLim + [-margin,margin];


% Vusualization
figure('Position',[100,100,300,350]);
set(groot,'DefaultAxesFontSize',12)
set(groot,'DefaultLineLineWidth',0.8)
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')

patch_b2 = patch([boundary_2(1,:),boundary_1(1,:)],[boundary_2(2,:),boundary_1(2,:)],[1 1 0], 'LineStyle', 'none','FaceAlpha',0.2); hold on;
plot(boundary_2(1,:),boundary_2(2,:),'k:')
patch_b1 = patch([boundary_1(1,:)],[boundary_1(2,:)],[.8 .3 0], 'LineStyle', ':', 'FaceAlpha',0.2); hold on;
plot(x_obs_sf(1,:),x_obs_sf(2,:),'k--')
patch_obs = patch(x_obs_boundary(1,:),x_obs_boundary(2,:),[0.6 1 0.6], 'FaceAlpha',1); hold on;
mainAxis = get(patch_obs, 'Parent');
% Draw Arrow Angular Rate
r_angRate = max(obs{it_obs}.a)*0.7;
arc_angRate = w_obs(it_obs)/2*pi+pi/5;
arc_angRate = min(abs(arc_angRate), pi*3/4)*sign(arc_angRate);
arc_angRate = max(abs(arc_angRate),pi/5)*sign(arc_angRate);

samp_it = 0:5;
x_angRate = obs{it_obs}.x0(1)+ -r_angRate*sin(-arc_angRate/2+arc_angRate/samp_it(end)*samp_it);
y_angRate = obs{it_obs}.x0(2) + r_angRate*cos(-arc_angRate/2+arc_angRate/samp_it(end)*samp_it);
plot(x_angRate(1:end-1), y_angRate(1:end-1), 'color',rotCol,'LineWidth',lw); hold on;
dx = (x_angRate(end)-x_angRate(end-1));
dy = (y_angRate(end)-y_angRate(end-1));

arrowAngRate = drawArrow(x_angRate(end-1:end),y_angRate(end-1:end),{'color', rotCol, 'LineWidth',lw},mainAxis); hold on;

               
grid on;
axis equal;
set(gca,'xtick',[],'ytick',[]); %
box on;
grid on;

xlim(xplotLim); ylim(yplotLim);

if saveFig
    print(strcat('fig_vector/','transition_rotatingEllipse'),'-depsc')
end


%% Transition several ellipse
N_circResol = 50;
w_transReg = 0.4;

margin_1 = 0.2;
margin_2 = 0.4;

it_obs=1;
obs{it_obs}.a = [2;0.6];
obs{it_obs}.p = [1;1];
obs{it_obs}.x0 = [-1;3];
obs{it_obs}.sf = [1.2];
obs{it_obs}.th_r = 30*pi/180;

it_obs=2;
obs{it_obs}.a = [0.9;0.9];
obs{it_obs}.p = [1;1];
obs{it_obs}.x0 = [3;1];
obs{it_obs}.sf = [1.2];

it_obs=3;
obs{it_obs}.a = [0.6;0.6];
obs{it_obs}.p = [1;1];
obs{it_obs}.x0 = [2;-1];
obs{it_obs}.sf = [1.2];

N_obs = length(obs);

[x_obs_boundary, x_obs_sf] = obs_draw_ellipsoid(obs,N_circResol);

r1 = obs{it_obs}.sf(1)*max(obs{it_obs}.a);
r2 = r1+w_transReg;

boundary_1 = zeros(2,N_circResol,N_obs);
boundary_2 = zeros(2,N_circResol,N_obs);

for it_obs = 1:length(obs)
    % Boundary ellipse
    pseudObs{1} = obs{it_obs};
    pseudObs{1}.a = obs{it_obs}.a*obs{it_obs}.sf+margin_1;
    [boundary, ~] = obs_draw_ellipsoid(pseudObs,N_circResol);
    boundary_1(:,:,it_obs) = boundary;

    pseudObs{1}.a = obs{it_obs}.a*obs{it_obs}.sf+margin_1+margin_2;
    [boundary, ~] = obs_draw_ellipsoid(pseudObs,N_circResol);
    boundary_2(:,:,it_obs) = boundary;   
end



% 
% xplotLim = [min(boundary_2(1,:)), max(boundary_2(1,:))];
% yplotLim = [min(boundary_2(2,:)), max(boundary_2(2,:))];
% xplotLim = xplotLim + [-margin,margin];
% yplotLim = yplotLim + [-margin,margin];




%% Find circle 
i1 = 2;
i2 = 3;

r1 = mean(obs{i1}.a)+margin_1+margin_2;
r2 = mean(obs{i2}.a)+margin_1+margin_2;

dx = obs{i1}.x0-obs{i2}.x0; % Difference x & y between circles

d  = norm(dx,2); % Distance between circles

phi1 = atan2(dx(2),dx(1)); % direction 1 - 2
phi2 = phi1 +pi; % direction 2 - 1

% Solve quadratic equation
a = 2;
b = 2*d;
c = r1^2+r2^2+d^2;

D = (4*a*c);
if D>=0
    d1 = (-b + sqrt(D))/(2*b);
    d_12 = (-b - sqrt(D))/(2*b);
    
    d2 = d-d1;
else
    error('Negative Diskirimant \n')
end

dPhi1 = acos(d1/r1);
dPhi2 = acos(d2/r2);


ang_intersect =[addAngle(phi1,dPhi1), addAngle(phi1,-dPhi1)];

% % Boundary Circle 1
% for ii = 1:size(boundary_1,2)
%     
%     if(angInAng23())
%         
%     end
% end

% TODO flatten at intersection

    
% Vusualization
figure('Position',[100,100,300,350]);
set(groot,'DefaultAxesFontSize',12)
set(groot,'DefaultLineLineWidth',0.8)
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')

for it_obs = 1:N_obs
    patch_b2 = patch([boundary_2(1,:,it_obs),boundary_1(1,:,it_obs)],[boundary_2(2,:,it_obs),boundary_1(2,:,it_obs)],[1 1 0], 'LineStyle', 'none','FaceAlpha',0.2); hold on;
    plot(boundary_2(1,:,it_obs),boundary_2(2,:,it_obs),'k:')
    patch_b1 = patch([boundary_1(1,:,it_obs)],[boundary_1(2,:,it_obs)],[.8 .3 0], 'LineStyle', ':', 'FaceAlpha',0.2); hold on;
    plot(x_obs_sf(1,:,it_obs),x_obs_sf(2,:,it_obs),'k--')
    patch_obs = patch(x_obs_boundary(1,:,it_obs),x_obs_boundary(2,:,it_obs),[0.6 1 0.6], 'FaceAlpha',1); hold on;
end

               
grid on;
axis equal;
set(gca,'xtick',[],'ytick',[]); %
box on;

% xlim(xplotLim); ylim(yplotLim);

if saveFig
    print(strcat('fig_vector/','transition_severalEllipse'),'-depsc')
end


%%
function inbetweenAngs = angInAng23(ang1,ang23)
    
    if(ang23(1) > ang23(2))
        dPhi = 2*pi - ang23(1);
        ang23(2) = ang23(2)+dPhi;
        ang1 = ang1 + dPhi;
        ang23 = -pi;
    end
    
    if(and(ang1>ang23(1), ang1<ang23(2)))
        inbetweenAngs = true;
    else
        inbetweenAngs = false;
    end

end

function dPhi = addAngle(alpha, beta)
% subtracts to angles and 
    if nargin<2 beta=0; end
   
    dPhi = alpha+beta;
    while(or(dPhi > pi, dPhi < - pi))
        dPhi = dPhi+(dPhi<-pi)*2*pi-(dPhi>pi)*2*pi;
    end
end


