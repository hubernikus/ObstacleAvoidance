function convexObstacle_vectorField
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
opt_sim.dt = 0.003; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined

%% Ellipse with good centering
%close all; clc;
fprintf('Start 2D-Simulation \n');

ds_handle = @(x) linearStableDS(x);
fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
                          obs_modulation_convergence(x,xd,obs,varargin);
% fn_handle_objAvoidance = @(x,xd, obs, varargin) ...
%                           obs_modulation_elastic(x,xd,obs, ds_handle, varargin);
% fn_handle_objAvoidance = @(x,xd, obs, varargin) ...
%                           obs_modulation_convergence_attractor(x,xd,obs, ds_handle, varargin);
                      
% Place obstacles
obs = [];
i=1;
obs{i}.a = [2.3;2.3];
obs{i}.p = [1;1];
obs{i}.x0 = [3.5;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 3;
obs{i}.perturbation.dx = [0,0];   
obs{i}.perturbation.w = 0;  

% Start simulation
opt_sim.obstacle = obs;
opt_sim.ds_handle = ds_handle;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
opt_sim.saveFig = true;


% Simulation Parameters
N_x = 100;  N_y = N_x;

scale = 1;
dx = 3;
dy = 0;
x_range = dx+scale*[-4.5,4.5]; y_range = dy+scale*[-4.2,4.2];

opt_sim.simulationName = 'linearSys_circle_notMoving'
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);

%% Moving cylinderVelocity
i = 1;

opt_sim.obstacle{i}.perturbation.dx = [-3,3];   
opt_sim.simulationName = 'linearSys_circle_movingFast'
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);

%%  Adapted Velocity
fn_handle_objAvoidance = @(x,xd, obs, varargin) ...
                obs_modulation_convergence_attractor(x,xd,obs, ds_handle, varargin);
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
opt_sim.saveFig = false; % Leave false, because save fig extern
opt_sim.simulationName = 'linearSys_circle_movingFast_adaptedVelocity';

[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)

% 
i=1;
dirVelocity = opt_sim.obstacle{i}.perturbation.dx;
centerObs = opt_sim.obstacle{i}.x0;

if( ~dirVelocity(2))
    warning('Include special condition ')
end

mTang = -dirVelocity(1)/dirVelocity(2);

yVal(1) = (x_range(1)-centerObs(1))*mTang+centerObs(2);
yVal(2) = (x_range(2)-centerObs(1))*mTang+centerObs(2);

plot(x_range, yVal, 'r--', 'LineWidth', 4)
xlim(x_range); ylim(y_range);

print(strcat('fig_vector/',opt_sim.simulationName), '-depsc','-r300');

%%  Convergence
fn_handle_objAvoidance = @(x,xd, obs, varargin) ...
                obs_modulation_convergence_attractor(x,xd,obs, ds_handle, varargin);
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
opt_sim.saveFig = true; 
opt_sim.simulationName = 'linearSys_circle_movingFast_attractor';
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)

%% Ellipse with good centering
close all; clc;
fprintf('Start 2D-Simulation \n');

ds_handle = @(x) linearStableDS(x);
fn_handle_objAvoidance = @(x,xd, obs, varargin) ...
                          obs_modulation_convergence_attractor(x,xd,obs, ds_handle, varargin);
N = 20;
x0 = [ones(1,N)*20 ; linspace(-15,15,N)];

% Place obstacles
obs = [];
i=1;
obs{i}.a = [2.6;2.6];
obs{i}.p = [1;1];
obs{i}.x0 = [3.5;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 3;
obs{i}.perturbation.dx = [-2;3];   
obs{i}.perturbation.w = 0;  


% Start simulation
opt_sim.obstacle = obs;
opt_sim.ds_handle = ds_handle;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
opt_sim.attractor= [0;0];
opt_sim.ds_type = 'nonlinear';
opt_sim.saveFig = true;

% Simulation Parameters
N_x = 40;  N_y = N_x;

scale = 1;
dx = 3;
dy = 0;
x_range = dx+scale*[-4.5,4.5]; y_range = dy+scale*[-4.2,4.2];

opt_sim.simulationName = 'linearSys_circle_movingFast'
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)


%% Ellipse with good centering
close all; clc;
fprintf('Start 2D-Simulation \n');

ds_handle = @(x) linearStableDS(x);
fn_handle_objAvoidance = @(x,xd,obs,b_contour,varargin) ...
                          obs_modulation_convergence_attractor(x,xd,obs,b_contour, varargin);
N = 100;
x0 = [ones(1,N)*20 ; linspace(-15,15,N)];

% Place obstacles
obs = [];
i=1;
obs{i}.a = [4;1];
obs{i}.p = [1;1];
obs{i}.x0 = [3.5;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = -30*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 3;
obs{i}.perturbation.dx = [0;0];   
obs{i}.perturbation.w = 5;  


% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
opt_sim.saveFig = true;

% Simulation Parameters
N_x = 40;  N_y = N_x;

scale = 1;
dx = 3;
dy = 0;
x_range = dx+scale*[-4.5,4.5]; y_range = dy+scale*[-4.2,4.2];

opt_sim.simulationName = 'linearSys_ellipse_rotating'
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)


%% Ellipse with good centering
% Set default simulation parameters
opt_sim.dt = 0.003; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined


close all; clc;
fprintf('Start 2D-Simulation \n');

ds_handle = @(x) linearStableDS(x);
fn_handle_objAvoidance = @(x,xd,obs,b_contour,varargin) ...
                          obs_modulation_convergence(x,xd,obs,b_contour, varargin);
N = 100;
x0 = [ones(1,N)*20 ; linspace(-15,15,N)];

% Place obstacles
obs = [];
i=1;
obs{i}.a = [4;0.5];
obs{i}.p = [3;3];
obs{i}.x0 = [3;-2.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 90*pi/180;

i=2;
obs{i}.a = [4;0.5];
obs{i}.p = [3;3];
obs{i}.x0 = [5;2.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 90*pi/180;

i=3;
obs{i}.a = [6;0.5];
obs{i}.p = [3;3];
obs{i}.x0 = [3;4.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;

i=4;
obs{i}.a = [6;0.5];
obs{i}.p = [3;3];
obs{i}.x0 = [6;-4.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;
% obs{i}.perturbation.t0 = 0;
% obs{i}.perturbation.tf = 3;
% obs{i}.perturbation.dx = [0;0];   
% obs{i}.perturbation.w = 0;  


% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
opt_sim.saveFig = true;

% Simulation Parameters
N_x = 40;  N_y = N_x;

scale = 1.5;
dx = 3;
dy = 0;
%x_range = dx+scale*[-4.5,4.5]; y_range = dy+scale*[-4.2,4.2];
x_range=[-4,13];
y_range=[-8,11];


opt_sim.simulationName = 'linearSys_ellipse_rotating'
%[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)
fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],ds_handle,opt_sim); % NOT good IC


