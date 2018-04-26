function Concave_obstacles_time_evolution
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
opt_sim.dt = 0.05; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined

%% Concave Obstacles
%close all;
clc;
opt_sim.dt = 0.01; %integration time steps
opt_sim.i_max = 400; %maximum number of iterations

taileffect = true;

%close all; clc;
clc;
fprintf('Start 2D-Simulation \n');

x_attractor = [0;0];
%ds_handle = @(x) linearStableDS(x, x_attractor);
ds_handle = @(x) LocalMinimum_DS(x,2);
%fn_handle = @(x) parallelFlow_DS(x,-10);


fn_handle_objAvoidance= @(x,xd,obs,varargin) ...
                            obs_modulation_convergence(x,xd,obs, varargin);

N = 20;
x0 = [-ones(1,N)*5; linspace(-5,6,N)];
%x0 = [3.;.2];
    
% Place obstacles
obs = [];

%obstacle 1
i=1;
obs{i}.a = [1;4];
obs{i}.p = [1;1];
obs{i}.x0 = [-3;0];
obs{i}.sf = [1];
obs{i}.th_r = 0*pi/180;
obs{i}.x_center = [0.0;0];
obs{i}.tailEffect = taileffect;
% obs{i}.perturbation.t0 = 0;
% obs{i}.perturbation.tf = 3;.
% obs{i}.perturbation.dx = [-6;0];   
% obs{i}.perturbation.w = 0;  
% 
% i=2;
% obs{i}.a = [1;1];
% obs{i}.p = [1;1];
% obs{i}.x0 = [5;3];
% obs{i}.sf = [1];
% obs{i}.th_r = 0*pi/180;
% obs{i}.x_center = [0.0;0];
% obs{i}.tailEffect = taileffect;

if isfield(opt_sim, 'obstacle');
    opt_sim = rmfield(opt_sim, 'obstacle');
end
%opt_sim = rmfield(opt_sim, 'obstacle');
opt_sim.attractor = x_attractor;

opt_sim.saveFig = true;             
%opt_sim = rmfield(opt_sim, 'attractor' );

% Simulation Parameters
N_x = 25;  N_y = N_x;
x_range = [-9,1]; y_range = [-5,5];

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
opt_sim.simulationName = 'three_ellipse_intersection'

Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

%%
fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;

%[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);
Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

%%
fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
opt_sim.obstacle = obs;
% Nonlinear function
fn_handle_objAvoidance= @(x,xd,obs,varargin) ...
                            obs_modulation_elastic(x,xd,obs, ds_handle, varargin);
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
Simulation(x0,[],ds_handle,opt_sim); % NOT good IC                      

