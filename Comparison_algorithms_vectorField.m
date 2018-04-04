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
opt_sim.dt = 0.003; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined

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
obs{i}.a = [0.5;2.6];
obs{i}.p = [1;1];
obs{i}.x0 = [3;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;

% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
opt_sim.saveFig = true;

% Simulation Parameters
N_x = 50;  N_y = N_x;
x_range = [-1,8]; y_range = [-4,4];

opt_sim.simulationName = 'linearDS_ellipse_centerInMiddle'
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)

figure(1)
i = 1;
plot(10*(opt_sim.obstacle{i}.x0(1)+opt_sim.obstacle{i}.a(1)*opt_sim.obstacle{i}.x_center(1))*[-1,1], ...
     10*(opt_sim.obstacle{i}.a(2)*opt_sim.obstacle{i}.x_center(2))*[-1,1], ...
     '--','Color',[0.7,0.,0.2], 'LineWidth',1.5)
xlim(x_range); ylim(y_range);
print(strcat('fig_vector/attractor_center.eps'),'-depsc','-r300')

fprintf('End 2D-Simulation \n');


%%
i = 1;
opt_sim.obstacle{i}.x_center = [0.2;0.8];
opt_sim.obstacle{i}.x_center_dyn = [3.;2];
opt_sim.simulationName = 'linearDS_ellipse_centerToRight';
opt_sim.saveFig = true;

[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);

figure(1)
i = 1;
plot(10*(opt_sim.obstacle{i}.x0(1)+opt_sim.obstacle{i}.a(1)*opt_sim.obstacle{i}.x_center(1))*[-1,1], ...
     10*(opt_sim.obstacle{i}.a(2)*opt_sim.obstacle{i}.x_center(2))*[-1,1], ...
     '--','Color',[0.7,0.,0.2], 'LineWidth',1.5)
xlim(x_range); ylim(y_range);
print(strcat('fig_vector/attractor_topleft.eps'),'-depsc','-r300')

%% Concave Obstacles
taileffect = true;

close all; clc;
fprintf('Start 2D-Simulation \n');

ds_handle = @(x) linearStableDS(x);
%fn_handle = @(x) parallelFlow_DS(x,-10);
fn_handle_objAvoidance= @(x,xd,obs,b_contour,varargin) ...
                          obs_modulation_convergence(x,xd,obs,b_contour, varargin);
N = 20;
x0 = [ones(1,N)*18 ; linspace(-15,15,N)];

% Place obstacles
obs = [];

% obstacle 1
i=1;
obs{i}.a = [1.;6];
obs{i}.p = [1;1];
obs{i}.x0 = [10;-4];
obs{i}.sf = [1.];
obs{i}.th_r = 40*pi/180;
obs{i}.x_center = [0.0;0.95];
opt_sim.dt = 0.003; %integration time steps
%obs{i}.x_center_dyn = [6.3518; 0.439];

obs{i}.tailEffect = taileffect;

i=2;
obs{i}.a = [1.4;7];
obs{i}.p = [1;1];
obs{i}.x0 = [10;-3];
obs{i}.sf = [1];
obs{i}.th_r = -60*pi/180;
obs{i}.x_center = [0.65;-.75];
%obs{i}.x_center_dyn = [6.3518; 0.439];
obs{i}.tailEffect = taileffect;


% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
opt_sim.attractor = [0;0];

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);

%Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

opt_sim.saveFig = false;

% Simulation Parameters
N_x = 100;  N_y = N_x;
x_range = [-1,25]; y_range = [-13,11];

opt_sim.simulationName = 'two_ellipse_intersection'

[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);
%Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

%% Concave Obstacles
taileffect = true;

close all; clc;
fprintf('Start 2D-Simulation \n');

x_attractor = [13;0];
ds_handle = @(x) linearStableDS(x, x_attractor);
%fn_handle = @(x) parallelFlow_DS(x,-10);
fn_handle_objAvoidance= @(x,xd,obs,b_contour,varargin) ...
                          obs_modulation_convergence(x,xd,obs,b_contour, varargin);
N = 20;
x0 = [ones(1,N)*30 ; linspace(-15,15,N)];

% Place obstacles
obs = [];

% obstacle 1
i=1;
obs{i}.a = [1.;6];
obs{i}.p = [1;1];
obs{i}.x0 = [10;-4];
obs{i}.sf = [1.];
obs{i}.th_r = 40*pi/180;
%obs{i}.x_center = [0.0;0.9];
obs{i}.tailEffect = taileffect;

i=2;
obs{i}.a = [1.4;7];
obs{i}.p = [1;1];
obs{i}.x0 = [10;3.5];
obs{i}.sf = [1.];
obs{i}.th_r = -60*pi/180;
%obs{i}.x_center = [.55;-0.7];
obs{i}.tailEffect = taileffect;

% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);

%Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

opt_sim.saveFig = false;
opt_sim.attractor = x_attractor;

% Simulation Parameters
N_x = 100;  N_y = N_x;
x_range = [-1,25]; y_range = [-13,11];

opt_sim.simulationName = 'twoEllipse_concaveConvergence'


[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);


%% Concave Obstacles
taileffect = true;

close all; clc;
fprintf('Start 2D-Simulation \n');

x_attractor = [13;0];
ds_handle = @(x) linearStableDS(x, x_attractor);
%fn_handle = @(x) parallelFlow_DS(x,-10);
fn_handle_objAvoidance= @(x,xd,obs,b_contour,varargin) ...
                          obs_modulation_convergence(x,xd,obs,b_contour, varargin);
N = 20;
x0 = [ones(1,N)*30 ; linspace(-15,15,N)];

% Place obstacles
obs = [];

% obstacle 1
i=1;
obs{i}.a = [1.;6];
obs{i}.p = [1;1];
obs{i}.x0 = [5;0];
obs{i}.sf = [1.];
obs{i}.th_r = 20*pi/180;
obs{i}.x_center = [0.0;0.9];
obs{i}.tailEffect = taileffect;

i=2;
obs{i}.a = [1.4;7];
obs{i}.p = [1;1];
obs{i}.x0 = [-5;0];
obs{i}.sf = [1.];
obs{i}.th_r = -30*pi/180;
obs{i}.x_center = [.55;0.7];
obs{i}.tailEffect = taileffect;

% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);

%Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

opt_sim.saveFig = true;
opt_sim.attractor = x_attractor;

% Simulation Parameters
N_x = 100;  N_y = N_x;
x_range = [-13,13]; y_range = [-12,13];

opt_sim.simulationName = 'twoEllipse_perpendicular'

[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);

%% Concave Obstacles
taileffect = true;

close all; clc;
fprintf('Start 2D-Simulation \n');

x_attractor = [20;0];
ds_handle = @(x) linearStableDS(x, x_attractor);
%fn_handle = @(x) parallelFlow_DS(x,-10);
fn_handle_objAvoidance= @(x,xd,obs,b_contour,varargin) ...
                          obs_modulation_convergence(x,xd,obs,b_contour, varargin);
N = 20;
x0 = [ones(1,N)*30 ; linspace(-15,15,N)];

% Place obstacles
obs = [];

% obstacle 1
i=1;
obs{i}.a = [1.;10];
obs{i}.p = [1;1];
obs{i}.x0 = [12;4];
obs{i}.sf = [1.1];
obs{i}.th_r = 0*pi/180;
obs{i}.x_center = [0.0;0];
obs{i}.tailEffect = taileffect;

i=2;
obs{i}.a = [1.4;8];
obs{i}.p = [1;1];
obs{i}.x0 = [12;.0];
obs{i}.sf = [1.1];
obs{i}.th_r = -60*pi/180;
obs{i}.x_center = [0.0;0];
obs{i}.tailEffect = taileffect;

i=3;
obs{i}.a = [0.9;10];
obs{i}.p = [1;1];
obs{i}.x0 = [12;.0];
obs{i}.sf = [1.1];
obs{i}.th_r = 60*pi/180;
obs{i}.x_center = [0.0;0];
obs{i}.tailEffect = taileffect;

% Start simulation
opt_sim.obstacle = obs;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);

%Simulation(x0,[],ds_handle,opt_sim); % NOT good IC

opt_sim.saveFig = false;
opt_sim.attractor = x_attractor;

% Simulation Parameters
N_x = 30;  N_y = N_x;
x_range = [-6,30]; y_range = [-16,16];

opt_sim.simulationName = 'twoEllipse_perpendicular'

[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);


