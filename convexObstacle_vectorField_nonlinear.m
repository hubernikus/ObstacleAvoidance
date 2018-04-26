Lfunction convexObstacle_vectorField
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
close all; clc;
fprintf('Start 2D-Simulation \n');

ds_handle = @(x) LocalMinimum_DS(x);
fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
                          obs_modulation_convergence(x,xd,obs,varargin);
% Place obstacles
obs = [];
i=1;
obs{i}.a = [1.5;4];
obs{i}.p = [1;1];
obs{i}.x0 = [-3.5;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;

% Start simulation
opt_sim.obstacle = obs;
opt_sim.ds_handle = ds_handle;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
opt_sim.saveFig = true;
opt_sim.saveInitialFig = false;

opt_sim.attractor = [0;0];
% Simulation Parameters
N_x = 100;  N_y = N_x;

scale = 6;
dx = -3;
dy = 0;
x_range = dx+scale*[-1.125,1.125]; y_range = dy+scale*[-1.05,1.05];

opt_sim.simulationName = 'nonlinearDS_ellipse'
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);


%%  Convergence
opt_sim.saveFig = true;

fn_handle_objAvoidance = @(x,xd, obs, varargin) ...
                obs_modulation_elastic(x,xd,obs, ds_handle, varargin);
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
%opt_sim.simulationName = 'nonlinearDS_ellipse_elastic';
opt_sim.simulationName = 'nonlinearDS_ellipse_elastic_attractor';
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)


%% Ellipse with good centering
close all; clc;
fprintf('Start 2D-Simulation \n');

ds_handle = @(x) LocalMinimum_DS(x);
fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
                          obs_modulation_convergence(x,xd,obs,varargin);
% Place obstacles
obs = [];
i=1;
obs{i}.a = [1.5;4];
obs{i}.p = [1;1];
obs{i}.x0 = [-3.5;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;

% Start simulation
opt_sim.obstacle = obs;
opt_sim.ds_handle = ds_handle;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
opt_sim.saveFig = false;
opt_sim.saveInitialFig = false;

% Simulation Parameters
N_x = 100;  N_y = N_x;

scale = 2.5;
dx = -5;
dy = 2;
x_range = dx+scale*[-1.125,1.125]; y_range = dy+scale*[-1.05,1.05];

opt_sim.simulationName = 'nonlinearDS_ellipse_zoom'
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);

%
ds_handle = @(x) linearStableDS(x);

opt_sim.simulationName = 'linearDS_ellipse_zoom'
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);





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
                          obs_modulation_convergence(x,xd,obs,b_contour, varargin);
N = 20;
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
obs{i}.perturbation.w = 3;  


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


%% Visualization attraction region
close all; clc;

fprintf('Start 2D-Simulation \n');

ds_handle = @(x) globally_stable_DS(x);
fn_handle_objAvoidance = @(x,xd, obs, varargin) ...
                obs_modulation_elastic(x,xd,obs, ds_handle, varargin);
                %obs_modulation_convergence(x,xd,obs, varargin);
                %obs_modulation_elastic(x,xd,obs, ds_handle, varargin);
% Place obstacles
obs = [];
i=1;
obs{i}.a = [0.3;3];
obs{i}.p = [1;1];
obs{i}.x0 = [-2;1.0];
obs{i}.sf = [1.0];
obs{i}.th_r = -40*pi/180;

% Start simulation
opt_sim.obstacle = obs;
opt_sim.ds_handle = ds_handle;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
opt_sim.saveFig = true;
opt_sim.saveInitialFig = true;

opt_sim.attractor = [0;0];
% Simulation Parameters
N_x = 100;  N_y = N_x;

scale = 4;
dx = -3;
dy = 0;
x_range = dx+scale*[-1.125,1.125]; y_range = dy+scale*[-1.05,1.05];

opt_sim.simulationName = 'visualization_attraction_region'
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);


colOrange = [255,165,0]/255;
opt_sim.obstacle{it_obs}.sf = 2; 
[x_obs_boundary, x_obs_sf] = obs_draw_ellipsoid(opt_sim.obstacle,50);
it_obs = 1;

figure(1);
patchs2 = patch([x_obs_sf(1,:,it_obs),x_obs_boundary(1,:,it_obs)],[x_obs_sf(2,:,it_obs),x_obs_boundary(2,:,it_obs)],colOrange, 'FaceAlpha',0.4); hold on;
print(strcat('fig_vector/',opt_sim.simulationName,'_withObstacle'),'-depsc')

figure(2);
patchs = patch(x_obs_boundary(1,:,it_obs),x_obs_boundary(2,:,it_obs),colOrange, 'FaceAlpha',0.4); hold on;
print(strcat('fig_vector/',opt_sim.simulationName,'_noObstacle'),'-depsc')