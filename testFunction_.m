function convexObstacle_timeEvolution
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
opt_sim.dt = 0.005; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary dLLifferential equation
opt_sim.obstacle = []; %no obstacle is defined

%% Demo: 2D - moving objects
close all; clc;
% 1 movign object.
fprintf('Start 2D-Simulation \n');

x_attr = [0;0];
fn_handle = @(x) linearStableDS(x);
fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
                          obs_modulation_convergence(x,xd,obs, varargin);
% fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
%                           obs_modulation_convergence_attractor(x,xd,obs, fn_handle, varargin);
N = 3;
x0 = [ones(1,N)*8 ; linspace(-4,4,N)];

% Place obstacles
obs = [];
i=1;
obs{i}.a = [1;3];
obs{i}.p = [1;1];
obs{i}.x0 = [4;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 30*pi/180;

obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 0;
obs{i}.perturbation.dx = [0;0];   
obs{i}.perturbation.w = 0;  

% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;

%--Simulation(x0,[],fn_handle,opt_sim); % NOT good IC
x = [0;3];
xd_init = linearStableDS(x);
xd_obs = obs{i}.perturbation.dx;
w_obs = 0;
xd = obs_modulation_convergence(x,xd_init,obs, xd_obs, w_obs)
%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');

