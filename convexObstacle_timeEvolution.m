function convexObstacle_timeEvolution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%          Semester Project - HUBER Lukas, LASA Lab, EPFL  
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


clc; close all; clear variables;
rng(1) % Set seeed for repeatability of simulation

%% preparing the obstacle avoidance module

%adding the obstacle avoidance folder to the MATLABWW path directories
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
ds_handle = @(x) linearStableDS(x);
% fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
%                           obs_modulation_convergence(x,xd,obs, varargin);
fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
                          obs_modulation_convergence_attractor(x,xd,obs, ds_handle, varargin);
N = 3;
x0 = [ones(1,N)*8 ; linspace(-4,4,N)];

% Place obstacles
obs = [];
i=1;
obs{i}.a = [2.6;2.6];
obs{i}.p = [1;1];
obs{i}.x0 = [3.5;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 0;
obs{i}.perturbation.dx = [0;0];   
obs{i}.perturbation.w = 0;  

% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');

%% Demo: 2D - moving objects
close all; clc;
% 1 movign object.
fprintf('Start 2D-Simulation \n');

x_attr = [0;0];
ds_handle = @(x) linearStableDS(x);
% fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
%                           obs_modulation_convergence(x,xd,obs, varargin);
fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
                          obs_modulation_convergence_attractor(x,xd,obs, ds_handle, varargin);
N = 10;
x0 = [ones(1,N)*10 ; linspace(-8,8,N)];

% Place obstacles
obs = [];
i=1;
obs{i}.a = [2.6;2.6];
obs{i}.p = [1;1];
obs{i}.x0 = [3.5;-3];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 10;
obs{i}.perturbation.dx = [0;1];   
obs{i}.perturbation.w = 0;  

% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');

%% Demo: 2D - moving objects
opt_sim.dt = 0.003; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
% close all; clc;
% 1 movign object.
fprintf('Start 2D-Simulation \n');

x_attr = [0;0];
ds_handle = @(x) linearStableDS(x);
fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
                          obs_modulation_convergence(x,xd,obs, varargin);
% fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
%                           obs_modulation_convergence_attractor(x,xd,obs, ds_handle, varargin);
N = 10;
x0 = [ones(1,N)*10 ; linspace(-8,8,N)];

% Place obstacles
obs = [];
i=1;
obs{i}.a = [0.8;2];
obs{i}.p = [1;1];
obs{i}.x0 = [5;-2];
obs{i}.sf = [1.0];
obs{i}.th_r = 60*pi/180;


i=2;
obs{i}.a = [0.8;1];
obs{i}.p = [1;1];
obs{i}.x0 = [0;2];
obs{i}.sf = [1.0];
obs{i}.th_r = 60*pi/180;

% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');

%% Demo: 2D - moving objects
opt_sim.dt = 0.003; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
% close all; clc;
% 1 movign object.
fprintf('Start 2D-Simulation \n');

x_attr = [0;0];
ds_handle = @(x) linearStableDS(x);
% fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
%                           obs_modulation_convergence(x,xd,obs, varargin);
fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
                          obs_modulation_convergence_attractor(x,xd,obs, ds_handle, varargin);
N = 15;
x0 = [ones(1,N)*10 ; linspace(-8,8,N)];

% Place obstacles
obs = [];
i=1;
obs{i}.a = [0.6;4];
obs{i}.p = [1;1];
obs{i}.x0 = [5;-6];
obs{i}.sf = [1.0];
obs{i}.th_r = 60*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 10;
obs{i}.perturbation.dx = [0;20];   
obs{i}.perturbation.w = 0;  

% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');


%% Demo: 2D - moving objects
opt_sim.dt = 0.003; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
% close all; clc;
% 1 movign object.
fprintf('Start 2D-Simulation \n');

x_attr = [0;0];
ds_handle = @(x) linearStableDS(x);
% fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
%                           obs_modulation_convergence(x,xd,obs, varargin);
fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
                          obs_modulation_convergence_attractor(x,xd,obs, ds_handle, varargin);
N = 16;
x0 = [ones(1,N)*10 ; linspace(-8,8,N)];

% Place obstacles
obs = [];
i=1;
obs{i}.a = [0.6;4];
obs{i}.p = [1;1];
obs{i}.x0 = [6;0];
obs{i}.sf = [1.0];
obs{i}.th_r = 60*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 10;
obs{i}.perturbation.dx = [0;0];   
obs{i}.perturbation.w = 5;  

% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');

