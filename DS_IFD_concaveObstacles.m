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
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined

%% Demo: concave
%close all; clc;

fprintf('Start 2D-Simulation \n');

x_attr=[-2;0];
fn_handle = @(x) stableDS(x,x_attr);

N =10;
x0 = [ones(1,N)*10 ; linspace(-10,10,N)];
x0 = x0(:,5)
x0 = [10;0.9];

% Place obstacles
obs1 = [];

% obstacle 1
obs1{1}.a = [1;4];
obs1{1}.p = [1;1];
obs1{1}.x0 = [0;0.0];
obs1{1}.sf = [1.2;1.2];
obs1{1}.th_r = 0*pi/180;
obs1{1}.concaveAngle = pi*0.3;

% obs{1}.perturbation.t0 = 0;
% obs{1}.perturbation.tf = 5;
% obs{1}.perturbation.dx = [3;0];  
% Start simulation
opt_sim.obstacle = obs1;

opt_sim.obstacleAvoidanceFunction = @(x,xd,obs,b_contour,varargin) obs_modulation_fluidMechanics(x,xd,obs,b_contour, varargin);


fig(1) = figure('name','fluidDynamics_model_movingObj_posx','position',[100 100 800 600]);

opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC
axis equal;
plot(obs1{1}.x0(1),obs1{1}.x0(2), 'ro')
fprintf('End 2D-Simulation \n');

%% Mehsgrids
close all; % clear variables
clc; 

% obs = [];
% obs1{1}.a = [1;4];
% obs1{1}.p = [1;1];
% obs1{1}.x0 = [-2;3];
% obs1{1}.sf = [1.2;1.2];
% obs1{1}.th_r = -30*pi/180;
% n= 1;
% obs1{n}.extra.ind = 2;
% obs1{n}.extra.C_Amp = 0.01;
% obs1{n}.extra.R_Amp = 0.0;

% obstacle 1
obs1 = [];
obs1{1}.a = [1;4];
obs1{1}.p = [1;1];
obs1{1}.x0 = [4;0.5];
obs1{1}.sf = [1.2;1.2];
obs1{1}.th_r = 0*pi/180;
obs1{1}.concaveAngle = pi*0.1;

dim = 2; 

N_x = 21; % Number of samples
N_y = N_x; X = [];
N_tot = N_x*N_y;

[X(:,:),Y(:,:)] = meshgrid(linspace(-0,10,N_x), linspace(-6,4,N_y));

xd_hat = zeros(dim, N_x, N_y);
xd_fluid = zeros(dim, N_x, N_y);

xd_obs = [0;0]; % assumed to not moving
w_obs = [0]; % assumed to not moving

b_contour = 0; % Not contouring object

for ix = 1:N_x
    for iy = 1:N_y
%        coll = obs_check_collision(obs1,[X(ix,iy);Y(ix,iy)]);
%        if(coll)
%            xd_hat(:,ix,iy)=0;
%            xd_fluid(:,ix,iy)=0;
%        else
            xd_hat(:,ix,iy) = stableDS([X(ix,iy);Y(ix,iy)]);
            xd_fluid(:,ix,iy) = obs_modulation_fluidMechanics([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs1, b_contour, xd_obs,w_obs);
%        end
    end
end

[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs1,50);

figure('Position',[0 0 800 600]);
patch(x_obs(1,:),x_obs(2,:),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6]); hold on;
plot(x_obs_sf(1,:),x_obs_sf(2,:),'k--','linewidth',0.5);

% Initialxd_hat(:,ix,iy) = stableDS([X(ix,iy);Y(ix,iy)]);
            xd_fluid(:,ix,iy) = obs_modulation_fluidMechanics([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs1, b_contour, xd_obs,w_obs); system
quiver(X(:,:), Y(:,:), squeeze(xd_hat(1,:,:)), squeeze(xd_hat(2,:,:)), 'k'), hold on;
%figure
quiver(X(:,:), Y(:,:), squeeze(xd_fluid(1,:,:)), squeeze(xd_fluid(2,:,:)), 'g')
xlim([X(1,1),X(end,end)]);ylim([Y(1,1),Y(end,end)])
axis equal;
legend('','','Original',  'Fluid')
%print('fig/quiverPlot_DS_LS_circle_fluidMechanics','-depsc')


%%
close all

% obstacle 1
obs1{1}.a = [1;8];
obs1{1}.p = [1;1];
obs1{1}.x0 = [0;0];
obs1{1}.sf = [1.2;1.2];
obs1{1}.th_r = 0*pi/180;
obs1{1}.concaveAngle = pi*0.2; % in [0,pi]

xd_obs = [0;0];
w_obs = 0;

[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs1,101);
    

figure('Position',[0 0 800 800]);
% Draw object
patch(x_obs(1,:),x_obs(2,:),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6],'FaceAlpha', 0.4); hold on;
plot(x_obs_sf(1,:),x_obs_sf(2,:),'k-x','linewidth',0.5);
axis equal;
plot(obs1{1}.x0(1),obs1{1}.x0(2), 'ro'); hold on;

x0 = [6;1];

xd_hat = stableDS(x0,[-2;0]);
xd_fluid= obs_modulation_fluidMechanics(x0,xd_hat, obs1, 0, xd_obs,w_obs);

plot(x0(1), x0(2), 'kx')
xd_hat = xd_hat*0.5;
plot([x0(1),x0(1)+xd_hat(1)],[x0(2),x0(2)+xd_hat(2)],'k--')
xd_fluid = xd_fluid*0.5;
plot([x0(1),x0(1)+xd_fluid(1)],[x0(2),x0(2)+xd_fluid(2)],'r')


% Unit circle
N_circ = 50;
phi = linspace(0,2*pi,N_circ);
x = cos(phi);
y = sin(phi);

figure;
patch(x,y,[0.6 1 0.6], 'FaceAlpha', 0.3); hold on;
axis equal;
xd_fluid= obs_modulation_fluidMechanics(x0,xd_hat, obs1, 0, xd_obs,w_obs);
grid on;

end
