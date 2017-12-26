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
% adding simulation tool-folder to path
if isempty(regexp(path,['lib_simulation_tools' pathsep], 'once'))
    addpath([pwd, '/lib_simulation_tools']);
end

% Set default simulation parameters
opt_sim.dt = 0.03; %integration time steps
opt_sim.i_max = 250; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined


%% Demo: 2D - moving objects
close all; clc;
% 1 movign object.
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) stableLimitcycle_DS(x);

N = 20;
%x0 = [ones(1,N)*1 ; linspace(-1,1,N)];
x0 = rand(2,10)*4-2;
%x0 = x0(:,5);

% Place obstacles
obs = [];

% obstacle 1
i=1;
obs{i}.a = [0.6;0.4];
obs{i}.p = [1;1];
obs{i}.x0 = [-1;-1];
obs{i}.sf = [1.2];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 6;
obs{i}.perturbation.dx = [-0.0;0.3];  
%obs{i}.perturbation.w = 1;  


% Start simulation

opt_sim.obstacle = obs;

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');


%% --- ISSUES....
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

% Place obstacles
obs = [];

% obstacle 1
i=1;

obs{i}.a = [1;5];
obs{i}.p = [1;1];
obs{i}.x0 = [10;-4];
obs{i}.sf = [1.2];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 6;
obs{i}.perturbation.dx = [-1;2];  
%obs{i}.perturbation.w = 1;  

% Start simulation

i=2; % object 2
obs{i}.a = [2;3];
obs{i}.p = [1;1];
obs{i}.x0 = [13;7];
obs{i}.sf = [1.2];
obs{i}.th_r = -30*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 6;
obs{i}.perturbation.dx = [-6;0];  
%obs{i}.perturbation.w = 3;  

opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');


%% --- ISSUES....
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

% Place obstacles
obs = [];

% obstacle 1
i=1;

obs{i}.a = [1;5];
obs{i}.p = [1;1];
obs{i}.x0 = [10;-4];
obs{i}.sf = [1.2];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 6;
obs{i}.perturbation.dx = [-1;2];  
%obs{i}.perturbation.w = 1;  

% Start simulation

i=2; % object 2
obs{i}.a = [2;3];
obs{i}.p = [1;1];
obs{i}.x0 = [13;7];
obs{i}.sf = [1.2];
obs{i}.th_r = -30*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 6;
obs{i}.perturbation.dx = [-5;-1];  
%obs{i}.perturbation.w = 3;  

opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');


%%  Rotating objects
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

% Place obstacles
obs = [];

% obstacle 1
i=1;

obs{i}.a = [1;5];
obs{i}.p = [1;1];
obs{i}.x0 = [10;6];
obs{i}.sf = [1.2];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 12;
obs{i}.perturbation.dx = [0;0];  
obs{i}.perturbation.w = 1;  


opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');

%%  Rotating objects
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

% Place obstacles
obs = [];

% obstacle 1
i=1;
obs{i}.a = [1;3];
obs{i}.p = [1;1];
obs{i}.x0 = [4;8];
obs{i}.sf = [1.2];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 12;
obs{i}.perturbation.dx = [4;-1];  
obs{i}.perturbation.w = 1;  
i=2;
obs{i}.a = [2;3];
obs{i}.p = [1;1];
obs{i}.x0 = [8;-5];
obs{i}.sf = [1.2];
obs{i}.th_r = 20*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 12;
obs{i}.perturbation.dx = [-1;2];  
obs{i}.perturbation.w = -2;  

opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');

%%  Rotating objects
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

% Place obstacles
obs = [];

% obstacle 1
i=1;

obs{i}.a = [1;5];
obs{i}.p = [1;1];
obs{i}.x0 = [10;-4];
obs{i}.sf = [1.2];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 8;
obs{i}.perturbation.dx = [-1;2];  
obs{i}.perturbation.w = 1;  

% Start simulation
i=2; % object 2
obs{i}.a = [2;3];
obs{i}.p = [1;1];
obs{i}.x0 = [13;7];
obs{i}.sf = [1.2];
obs{i}.th_r = -30*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 8;
obs{i}.perturbation.dx = [1;-3];  
obs{i}.perturbation.w = 3;  

opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])



%%  Constant vel to origin 
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

% Place obstacles
obs = [];

opt_sim.dt = 0.03; %integration time steps
opt_sim.i_max = 500; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance

% obstacle 1
i=1;

obs{i}.a = [3;3];
obs{i}.p = [1;1];
obs{i}.x0 = [10;5];
obs{i}.sf = [1.2];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 4;
obs{i}.perturbation.tf = 12;
obs{i}.perturbation.dx = [-4;0];  
%obs{i}.perturbation.w = 1;  

opt_sim.obstacle = obs;

fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC


fprintf('End 2D-Simulation \n');


%%  Rotating object close to origin 
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

opt_sim.dt = 0.01; %integration time steps
opt_sim.i_max = 500; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance


% Place obstacles
obs = [];

% obstacle 1
i=1;

obs{i}.a = [1;5];
obs{i}.p = [1;1];
obs{i}.x0 = [6;3];
obs{i}.sf = [1.2];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 12;
obs{i}.perturbation.dx = [0;0];  
obs{i}.perturbation.w = 1;  


opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');


%%  2 convex to concave
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

% Place obstacles
obs = [];

% obstacle 1
i=1;
obs{i}.a = [1;4];
obs{i}.p = [1;1];
obs{i}.x0 = [5;-2];
obs{i}.sf = [1.2];
obs{i}.th_r = -60*pi/180;
i=2;
obs{i}.a = [1;5];
obs{i}.p = [1;1];
obs{i}.x0 = [4;3];
obs{i}.sf = [1.2];
obs{i}.th_r = 45*pi/180;


opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');


%%  2 convex to concave
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

% Place obstacles
obs = [];

% obstacle 1
i=1;
obs{i}.a = [1;4];
obs{i}.p = [1;1];
obs{i}.x0 = [8;-2];
obs{i}.sf = [1.2];
obs{i}.th_r = 60*pi/180;
i=2;
obs{i}.a = [1;5];
obs{i}.p = [1;1];
obs{i}.x0 = [7;3];
obs{i}.sf = [1.2];
obs{i}.th_r = -45*pi/180;


opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj','position',[200 100 700 700]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');

%% Demo: concave
%close all; clc;

fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N =10;
x0 = [ones(1,N)*10 ; linspace(-10,10,N)];
x0 = x0(:,5)

% Place obstacles
obs = [];

% obstacle 1
obs{1}.a = [1;4];
obs{1}.p = [1;1];
obs{1}.x0 = [4;0.5];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 0*pi/180;
obs{1}.concaveAngle = pi*0.3;

% obs{1}.perturbation.t0 = 0;
% obs{1}.perturbation.tf = 5;
% obs{1}.perturbation.dx = [3;0];  
% Start simulation
opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj_posx','position',[100 100 800 600]);


opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC
axis equal;
%xlim([-15 15]); ylim([-15 15])
%close al
plot(obs{1}.x0(1),obs{1}.x0(2), 'ro')
fprintf('End 2D-Simulation \n');


fprintf('End 2D-Simulation \n');
%%
close all

% obstacle 1
obs{1}.a = [1;4];
obs{1}.p = [1;1];
obs{1}.x0 = [0;0];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 0*pi/180;
obs{1}.concaveAngle = pi*0.2;

[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs,101);
    

figure('Position',[0 0 1000 1000]);
% Draw object
patch(x_obs(1,:),x_obs(2,:),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6]); hold on;
plot(x_obs_sf(1,:),x_obs_sf(2,:),'k-x','linewidth',0.5);
%print('fig/quiverPlot_DS_LS_circle_fluidMechanics','-depsc')
axis equal;
plot(obs{1}.x0(1),obs{1}.x0(2), 'ro')
