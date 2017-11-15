%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%          Semester Project - HUBER Lukas, LASA Lab, EPFL
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%clear all;
%%
clc; close all; clear all;

rng(1) % Set seeed for repeatability of simulation

%% preparing the obstacle avoidance module

%adding the obstacle avoidance folder to the MATLAB path directories
if isempty(regexp(path,['lib_obstacle_avoidance' pathsep], 'once'))
    addpath([pwd, '/lib_obstacle_avoidance']);
end
%adding the example folder to the MATLAB path directories
if isempty(regexp(path,['Example_DSs' pathsep], 'once'))
    addpath([pwd, '/Example_DSs']);
end

% Set default simulation parameters
opt_sim.dt = 0.1; %integration time steps
opt_sim.i_max = 100; %maximum number of iterations
opt_sim.tol = 0.1; % convergence tolerance
opt_sim.plot = true; % enabling the animationqq
opt_sim.model = 1; % first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined

%adding the obstacle avoidance folder to the MATLAB path directories
if isempty(regexp(path,['lib_obstacle_avoidance' pathsep], 'once'))
    addpath([pwd, '/lib_obstacle_avoidance']);
end
%adding the example folder to the MATLAB path directories
if isempty(regexp(path,['Example_DSs' pathsep], 'once'))
    addpath([pwd, '/Example_DSs']);
end



%%
figure('Position',[0 0 550 400])
set(groot,'DefaultLineLineWidth',1.2)            
set(groot,'DefaultAxesFontSize',14)

    
N = 20; % number of divisions 
xLim = [-10,10]; yLim = [-10,10];
[X,Y] = meshgrid(xLim(1) : (xLim(2)-xLim(1))/N : xLim(2), ...
                yLim(1) : (yLim(2)-yLim(1))/N : yLim(2));

[Nx,Ny] = size(X);
u = zeros(Nx,Ny); v = zeros(Nx,Ny);
for ix = 1:Nx
    for iy = 1:Ny
        dx = stableDS([X(ix,iy),; Y(ix,iy)]);
        u(ix,iy) = dx(1); v(ix,iy) =dx(2);
    end
end

quiver(X,Y,u,v,'k')
xlim(xLim); ylim(yLim);
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')

print('fig/stable_DS','-dpng')


%% 
figure('Position',[0 0 550 400])
set(groot,'DefaultLineLineWidth',1.2)            
set(groot,'DefaultAxesFontSize',14)

    
N = 20; % number of divisions 
xLim = [-1.3,1.3]; yLim = [-1.3,1.3];
[X,Y] = meshgrid(xLim(1) : (xLim(2)-xLim(1))/N : xLim(2), ...
                yLim(1) : (yLim(2)-yLim(1))/N : yLim(2));

[Nx,Ny] = size(X);
u = zeros(Nx,Ny); v = zeros(Nx,Ny);
for ix = 1:Nx
    for iy = 1:Ny
        dx = stableLimitcycle_DS([X(ix,iy),; Y(ix,iy)]);
        u(ix,iy) = dx(1); v(ix,iy) =dx(2);
    end
end


quiver(X,Y,u,v,1.5,'k')
xlim(xLim); ylim(yLim);
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')

print('fig/limitCycle_DS','-dpng')


%%

opt_sim.dt = 0.1; %integration time steps
opt_sim.tol = 0.1; % convergence tolerance
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
x0 = (rand(2,15)-0.5)*20; % randomly distributed points


obs = [];
opt_sim.obstacle = obs;


fig(1) = figure('name','linear_DynSys_2D','Position',[100 100 700 500]);
set(groot,'DefaultLineLineWidth',1.2)            
set(groot,'DefaultAxesFontSize',14)

opt_sim.figure = fig(1);
Simulation_improved(x0,[],fn_handle,opt_sim);


fprintf('End Simulation')

%%
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) stableLimitcycle_DS(x);
x0 = (rand(2,15)-0.5)*2.2; % randomly distributed points
opt_sim.i_max = 200; %maximum number of iterations


obs = [];
opt_sim.obstacle = obs;


fig(1) = figure('name','limitCycle_2D','Position',[100 100 700 500]);
xlim([-1,1]*1.3); ylim([-1,1]*1.3)
set(groot,'DefaultLineLineWidth',1.2)            
set(groot,'DefaultAxesFontSize',14)

opt_sim.figure = fig(1);
Simulation_improved(x0,[],fn_handle,opt_sim);


fprintf('End Simulation')

%%

fprintf('Start 3D-Simulation \n');

fn_handle = @(x) stableDS_3D(x);
x0 = (rand(3,30)-0.5)*20; % randomly distributed points


obs = [];
opt_sim.obstacle = obs;


fig(1) = figure('name','linear_DynSys_3D','Position',[100 100 700 500]);
set(groot,'DefaultLineLineWidth',1.2)            
set(groot,'DefaultAxesFontSize',14)
opt_sim.figure = fig(1);
Simulation_improved(x0,[],fn_handle,opt_sim);


fprintf('End Simulation')


%%

fprintf('Start 3D-Simulation \n');

fn_handle = @(x) stableDS_3D(x);
x0 = (rand(3,30)-0.5)*20; % randomly distributed points


obs = [];
opt_sim.obstacle = obs;


fig(1) = figure('name','linear_DynSys_3D','Position',[100 100 700 500]);
set(groot,'DefaultLineLineWidth',1.2)            
set(groot,'DefaultAxesFontSize',14)
opt_sim.figure = fig(1);
Simulation_improved(x0,[],fn_handle,opt_sim);


fprintf('End Simulation')
%%
opt_sim.dt = 0.01; %integration time steps
opt_sim.i_max = 400; %maximum number of iterations

%% Demo: 2D - multiple objects overlapping -> moving into object
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 10;
phi0 = (1:N_samples)/N_samples*2*pi;
r0 = 10;
%x0 = [r0* cos(phi0); r0*sin(phi0)];
x0 = [-10*ones(1,10); (1:N_samples)/N_samples*20-8];

% Place obstacles
obs = [];
% obstacle 1
i = 2;
obs{i}.a = [1 1;2 2];
obs{i}.p = [1 1;1 1];
obs{i}.partition = [-pi 0;0 pi];
obs{i}.x0 = [-6;1];
obs{i}.sf = [1.2;1.2];
obs{i}.rho = 1.5;
obs{i}.th_r = 0*pi/180;
% obstacle 2
i = 1;
obs{i}.a = [0.5 0.5; 2 2];
obs{i}.p = [1 1;1 1];
obs{i}.partition = [-pi 0;0 pi];
obs{i}.x0 = [-8;-5];
obs{i}.sf = [1.2;1.2];
obs{i}.rho = 1.5;  % Reactivity
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 10;
obs{i}.perturbation.dx = [0;1];


opt_sim.obstacle = obs;

% Start simulation
close all;
fig(1) = figure('name','twoMovingObjects_DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
%Simulation(x0,[],fn_handle,opt_sim);
Simulation(x0,[],fn_handle,opt_sim);
fprintf('End 2D-Simulation \n');

%% Demo: 2D - multiple objects overlapping -> moving into object
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 10;
phi0 = (1:N_samples)/N_samples*2*pi;
r0 = 10;
%x0 = [r0* cos(phi0); r0*sin(phi0)];
x0 = [-10*ones(1,10); (1:N_samples)/N_samples*20-8];

% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [1 1;2 2];
obs{1}.p = [1 1;1 1];
obs{1}.partition = [-pi 0;0 pi];
obs{1}.x0 = [-6;1];
obs{1}.sf = [1.2;1.2];
obs{1}.rho = 1.5;
obs{1}.th_r = 0*pi/180;
% obstacle 2
obs{2}.a = [0.5 0.5; 2 2];
obs{2}.p = [1 1;1 1];
obs{2}.partition = [-pi 0;0 pi];
obs{2}.x0 = [-6;-3];
obs{2}.sf = [1.2;1.2];
obs{2}.rho = 1.5;  % Reactivity
obs{2}.th_r = 0*pi/180;
%obs{2}.perturbation.t0 = 0;
%obs{2}.perturbation.tf = 10;
%obs{2}.perturbation.dx = [0;1];


opt_sim.obstacle = obs;

% Start simulation
close all;
fig(1) = figure('name','concavePositioning','position',[100 550 560 420]);
opt_sim.figure = fig(1);
%Simulation(x0,[],fn_handle,opt_sim);
Simulation(x0,[],fn_handle,opt_sim);
fprintf('End 2D-Simulation \n');
