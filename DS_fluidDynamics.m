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

% Set default simulation parameters
opt_sim.dt = 0.02; %integration time steps
opt_sim.i_max = 300; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined


%% Mehsgrids
close all; % clear variables

obs = [];
obs1{1}.a = [5;5];
obs1{1}.p = [1;1];
obs1{1}.x0 = [-2;1];
obs1{1}.sf = [1.2;1.2];
obs1{1}.th_r = 90*pi/180;
n= 1;
obs1{n}.extra.ind = 2;
obs1{n}.extra.C_Amp = 0.01;
obs1{n}.extra.R_Amp = 0.0;

dim = 2; 

N_x = 21; % Number of samples
N_y = N_x; X = [];

[X(:,:),Y(:,:)] = meshgrid(linspace(-10,10,N_x), linspace(-15,15,N_y));

xd_hat = zeros(dim, N_x, N_y);
xd_fluid = zeros(dim, N_x, N_y);
xd_ellips = zeros(dim, N_x, N_y);

xd_obs = [0;0]; % assumed to not moving

for ix = 1:N_x
    for iy = 1:N_y
        xd_hat(:,ix,iy) = parallelFlow_DS([X(ix,iy);Y(ix,iy)]);
        b_contour = 0;
        xd_fluid(:,ix,iy) = obs_modulation_fluidMechanics([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs1, b_contour, xd_obs);
        xd_ellips(:,ix,iy) = obs_modulation_ellipsoid([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs1, b_contour, xd_obs);
    end
end
[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs1,50);

for ii = 1:size(X,1)
    for jj = 1:size(X,2)
        coll = obs_check_collision(obs1,[X(ii,jj);Y(ii,jj)]);
        if(coll)
            xd_hat(:,ii,jj)=0;
            xd_fluid(:,ii,jj)=0;
            xd_ellips(:,ii,jj)=0;
        end
        
    end
end




figure('Position',[0 0 1000 1000]);
% Draw object
patch(x_obs(1,:),x_obs(2,:),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6]); hold on;
plot(x_obs_sf(1,:),x_obs_sf(2,:),'k--','linewidth',0.5);

% Initial system
quiver(X(:,:), Y(:,:), squeeze(xd_hat(1,:,:)), squeeze(xd_hat(2,:,:)), 'k'), hold on;
% Modulated system
%figure
quiver(X(:,:), Y(:,:), squeeze(xd_ellips(1,:,:)), squeeze(xd_ellips(2,:,:)), 'b')
quiver(X(:,:), Y(:,:), squeeze(xd_fluid(1,:,:)), squeeze(xd_fluid(2,:,:)), 'g')
xlim([X(1,1),X(end,end)]);ylim([Y(1,1),Y(end,end)])
%legend('','','Original System', 'Ellipse Modulation', 'Ellipse Modulation Adaptded')
%drawArrow([-4,-4],[-5,-8],'linewidth',3,'color','r')

print('fig/quiverPlot_DS_LS_circle_fluidMechanics','-depsc')

%
%% Demo: 2D - multiple objects overlapping
%close all; clc;

fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

%x0 = randn(2,15)*10; % randomly distributed points
%N_samples = 20;
%phi0 = (1:N_samples)/N_samples*2*pi;
%r0 = 10;
% x0 = [r0* cos(phi0); r0*sin(phi0)];
%x0 = [-10;-4];
x0 = [ones(1,27)*-10 ; -13:1:13];

% Place obstacles
obs = [];

% obstacle 1
obs{1}.a = [1;2];
obs{1}.p = [1;1];
obs{1}.x0 = [-5;0];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 45*pi/180;

% Start simulation
opt_sim.obstacle = obs;

fprintf('Simulating normal object modul \n')
opt_sim.obsFunc_handle = @(x, xd, obs, xd_obs) obs_modulation_fluidMechanics(x,xd,obs,xd_obs);
fig(1) = figure('name','fluidDynamics_model','position',[100 350 750 600]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

fprintf('Simulating fluid object modul \n')
opt_sim.obsFunc_handle = @(x, xd, obs, xd_obs) obs_modulation_ellipsoid(x,xd,obs,xd_obs);
fig(2) = figure('name','general_model','position',[100 350 750 600]);
opt_sim.figure = fig(2);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC
%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');


%% Demo: 2D - moving objects
%close all; clc;

fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

%x0 = randn(2,15)*10; % randomly distributed points
%N_samples = 20;
%phi0 = (1:N_samples)/N_samples*2*pi;
%r0 = 10;
% x0 = [r0* cos(phi0); r0*sin(phi0)];
%x0 = [-10;-4];
x0 = [ones(1,27)*-10 ; -13:1:13];

% Place obstacles
obs = [];

% obstacle 1
obs{1}.a = [1;3];
obs{1}.p = [1;1];
obs{1}.x0 = [-2;-3];
obs{1}.sf = [1.2];
obs{1}.th_r = 45*pi/180;

obs{1}.perturbation.t0 = 0;
obs{1}.perturbation.tf = 4;
obs{1}.perturbation.dx = [-2;2];  
% Start simulation
opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj','position',[100 550 560 420]);


opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');

%% Demo: 2D - moving objects
%close all; clc;

fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

%x0 = randn(2,15)*10; % randomly distributed points
%N_samples = 20;
%phi0 = (1:N_samples)/N_samples*2*pi;
%r0 = 10;
% x0 = [r0* cos(phi0); r0*sin(phi0)];
%x0 = [-10;-4];
x0 = [ones(1,27)*-10 ; -13:1:13];

% Place obstacles
obs = [];

% obstacle 1
obs{1}.a = [1;3];
obs{1}.p = [1;1];
obs{1}.x0 = [3;1];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = -30*pi/180;

obs{1}.perturbation.t0 = 0;
obs{1}.perturbation.tf = 3;
obs{1}.perturbation.dx = [-4;0];  
% Start simulation
opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj_negx','position',[100 550 560 420]);


opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');

%% Demo: 2D - moving objects
%close all; clc;

fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

%x0 = randn(2,15)*10; % randomly distributed points
%N_samples = 20;
%phi0 = (1:N_samples)/N_samples*2*pi;
%r0 = 10;
% x0 = [r0* cos(phi0); r0*sin(phi0)];
%x0 = [-10;-4];
x0 = [ones(1,27)*-10 ; -13:1:13];

% Place obstacles
obs = [];

% obstacle 1
obs{1}.a = [1;2];
obs{1}.p = [1;1];
obs{1}.x0 = [-8;1];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 30*pi/180;

obs{1}.perturbation.t0 = 0;
obs{1}.perturbation.tf = 5;
obs{1}.perturbation.dx = [3;0];  
% Start simulation
opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj_posx','position',[100 550 560 420]);


opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');
