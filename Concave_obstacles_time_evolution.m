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
opt_sim.dt = 0.03; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined

%% Concave Obstacles
close all
opt_sim.dt = 0.01; %integration time steps
opt_sim.i_max = 800; %maximum number of iterations

taileffect = true;

%close all; clc;
clc;
fprintf('Start 2D-Simulation \n');

x_attractor = [0;0];
ds_handle = @(x) linearStableDS(x, x_attractor);
%fn_handle = @(x) parallelFlow_DS(x,-10);

%fn_handle_objAvoidance= @(x,xd,obs,varargin) ...
%                          obs_modulation_ellipsoid_3(x,xd,obs, varargin);
fn_handle_objAvoidance= @(x,xd,obs,varargin) ...
                          obs_modulation_convergence(x,xd,obs, varargin);
                      
N = 5;
x0 = [ones(1,N)*8 ; linspace(-5,6,N)];

% Place obstacles
obs = [];

%obstacle 1
i=1;
obs{i}.a = [1;1];
obs{i}.p = [1;1];
obs{i}.x0 = [-1;0];
obs{i}.sf = [1];
obs{i}.th_r = 30*pi/180;
obs{i}.x_center = [0.0;0];
obs{i}.tailEffect = taileffect;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 3;
obs{i}.perturbation.dx = [-6;0];   
obs{i}.perturbation.w = 0;  

i=2;
obs{i}.a = [1;1];
obs{i}.p = [1;1];
obs{i}.x0 = [1;-0];
obs{i}.sf = [1];
obs{i}.th_r = 0*pi/180;
obs{i}.x_center = [0.0;0];
obs{i}.tailEffect = taileffect;

opt_sim.obstacle = obs;
opt_sim.attractor = x_attractor;

opt_sim.saveFig = true;             
%opt_sim = rmfield(opt_sim, 'attractor' );

% Simulation Parameters
N_x = 25;  N_y = N_x;
x_range = [-3,3]; y_range = [-3,3];

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
opt_sim.simulationName = 'three_ellipse_intersection'
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;

%[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);
Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

%% Concave Obstacles
opt_sim.dt = 0.003; %integration time steps
opt_sim.i_max = 800; %maximum number of iterations

taileffect = true;

%close all; clc;
clc;
fprintf('Start 2D-Simulation \n');

x_attractor = [0;0];
ds_handle = @(x) linearStableDS(x, x_attractor);
%fn_handle = @(x) parallelFlow_DS(x,-10);

%fn_handle_objAvoidance= @(x,xd,obs,varargin) ...
%                          obs_modulation_ellipsoid_3(x,xd,obs, varargin);
fn_handle_objAvoidance= @(x,xd,obs,varargin) ...
                          obs_modulation_convergence(x,xd,obs, varargin);
                      
N = 20;
x0 = [ones(1,N)*26 ; linspace(-15,20,N)];

% Place obstacles
obs = [];

%obstacle 1
i=1;
obs{i}.a = [1.;9];
obs{i}.p = [1;1];
obs{i}.x0 = [12;10];
% obs{i}.x0 = [12;6];
obs{i}.sf = [1];
obs{i}.th_r = 60*pi/180;
obs{i}.x_center = [0.0;0];
obs{i}.tailEffect = taileffect;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 2;
obs{i}.perturbation.dx = [0;0];   
obs{i}.perturbation.w = 2;  

i=2;
obs{i}.a = [1.2;7];
obs{i}.p = [1;1];
obs{i}.x0 = [12;.0];
obs{i}.sf = [1];
obs{i}.th_r = 60*pi/180;
obs{i}.x_center = [0.0;0];
obs{i}.tailEffect = taileffect;

% i=3;
% obs{i}.a = [1.2;1];
% obs{i}.p = [1;1];
% obs{i}.x0 = [20;-10];
% obs{i}.sf = [1];
% obs{i}.th_r = 60*pi/180;
% obs{i}.x_center = [0.0;0];
% obs{i}.tailEffect = taileffect;

opt_sim.obstacle = obs;
opt_sim.attractor = x_attractor;

opt_sim.saveFig = true;             
%opt_sim = rmfield(opt_sim, 'attractor' );

% Simulation Parameters
N_x = 25;  N_y = N_x;
x_range = [-3,28]; y_range = [-14,14];

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
opt_sim.simulationName = 'three_ellipse_intersection';
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;

%[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);
Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

%% Concave Obstacles
opt_sim.dt = 0.003; %integration time steps
opt_sim.i_max = 500; %maximum number of iterations

taileffect = true;

close all; clc;
fprintf('Start 2D-Simulation \n');

x_attractor = [0;0];
ds_handle = @(x) linearStableDS(x, x_attractor);

%fn_handle = @(x) parallelFlow_DS(x,-10);
fn_handle_objAvoidance= @(x,xd,obs,varargin) ...
                          obs_modulation_convergence(x,xd,obs, varargin);
N = 20;
x0 = [ones(1,N)*26 ; linspace(-15,20,N)];

% Place obstacles
obs = [];

% obstacle 1
i=1;
obs{i}.a = [1.;9];
obs{i}.p = [1;1];
obs{i}.x0 = [12;18];
%obs{i}.x0 = [12;6];
obs{i}.sf = [1];
obs{i}.th_r = -60*pi/180;
obs{i}.x_center = [0.0;0];
obs{i}.tailEffect = taileffect;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 1.6;
obs{i}.perturbation.dx = [0;-20];  
% 
i=2;
obs{i}.a = [1.2;7];
obs{i}.p = [1;1];
obs{i}.x0 = [12;.0];
obs{i}.sf = [1];
obs{i}.th_r = 60*pi/180;
obs{i}.x_center = [0.0;0];
obs{i}.tailEffect = taileffect;

opt_sim.obstacle = obs;
opt_sim.attractor = x_attractor;

opt_sim.saveFig = true;             
%opt_sim = rmfield(opt_sim, 'attractor' );

% Simulation Parameters
N_x = 25;  N_y = N_x;
x_range = [-3,28]; y_range = [-14,14];
%x_range = [-10,10]; y_range = [-10,10];

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);

opt_sim.simulationName = 'three_ellipse_intersection'

%[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);
Simulation(x0,[],ds_handle,opt_sim); % NOT good IC



