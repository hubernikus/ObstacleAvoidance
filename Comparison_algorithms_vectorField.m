function DS_comparison_algorithms_conference
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
opt_sim.obstacle = []; %no obstacle is defined

%%
% Set default simulation parameters
opt_sim.dt = 0.025; %integration time steps
opt_sim.i_max = 400; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation

%%
clc; close all;

xAttr = [12;-0];
ds_handle = @(x) linearStableDS(x,xAttr);

obs = []
obs{1}.a = [10;1];
obs{1}.p = [1;1];
obs{1}.x0 = [0;0];

obs{1}.sf = [1.0;1.0];
obs{1}.th_r = 40*pi/180;

N_x = 41; % Number of samples
N_y = N_x;
x_range = [-10,14];
y_range = [-10,10];

opt_sim.obstacle = obs;
%opt_sim.simulationName = 'oneStaticCirc';

[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)

%%
clc; close all;

ds_handle = @(x) linearStableDS(x);


obs = []
obs{1}.a = [2;2];
obs{1}.p = [1;1];
obs{1}.x0 = [-2;3];

obs{1}.sf = [1.2;1.2];
obs{1}.th_r = -30*pi/180;

N_x = 41; % Number of samples
N_y = N_x;
x_range = [-10,5];
y_range = [-3,11];

opt_sim.obstacle = obs;
opt_sim.simulationName = 'oneStaticCirc';

[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)

%%
clc; close all;

ds_handle = @(x) linearStableDS(x);


obs = []
obs{1}.a = [1;2];
obs{1}.p = [1;1];
obs{1}.x0 = [-2;3];

obs{1}.sf = [1.2;1.2];
obs{1}.th_r = -30*pi/180;

N_x = 41; % Number of samples
N_y = N_x;
x_range = [-12,5];
y_range = [-2,14];

opt_sim.obstacle = obs;
opt_sim.simulationName = 'oneStaticEllipse_medium';
    
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)

%%
clc; close all;

ds_handle = @(x) linearStableDS(x);


obs = []
obs{1}.a = [1;2];
obs{1}.p = [1;1];
obs{1}.x0 = [-2;3];

obs{1}.sf = [1.2;1.2];
obs{1}.th_r = -30*pi/180;

N_x = 41; % Number of samples
N_y = N_x;
x_range = [-8,3];
y_range = [-2,8];

opt_sim.obstacle = obs;
opt_sim.simulationName = 'oneStaticEllipse_close';
    
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)


%%
clc; close all;

ds_handle = @(x) linearStableDS(x);

obs = []
obs{1}.a = [1;4];
obs{1}.p = [1;1];
obs{1}.x0 = [-2;3];

obs{1}.sf = [1.0;1.0];
obs{1}.th_r = -30*pi/180;

N_x = 41; % Number of samples
N_y = N_x;
x_range = [-8,3];
y_range = [-2,8];

opt_sim.obstacle = obs;
opt_sim.simulationName = 'oneStaticEllipse_long';
    
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)



%%
clc; close all;

ds_handle = @(x) linearStableDS(x);

% Place obstacles
obs = [];

% obstacle 1
i=1;
obs{i}.a = [1;3];
obs{i}.p = [1;1];
obs{i}.x0 = [5;-2];
obs{i}.sf = [1.2];
obs{i}.th_r = -60*pi/180;
i=2;
obs{i}.a = [2;2.5];
obs{i}.p = [1;1];
obs{i}.x0 = [3;2];
obs{i}.sf = [1.2];
obs{i}.th_r = 45*pi/180;
i=3;
obs{i}.a = [4;1];
obs{i}.p = [1;1];
obs{i}.x0 = [-3;-2];
obs{i}.sf = [1.2];
obs{i}.th_r = 90*pi/180;
i=4;
obs{i}.a = [2;1];
obs{i}.p = [1;1];
obs{i}.x0 = [1;-7];
obs{i}.sf = [1.2];
obs{i}.th_r = 0*pi/180;

N_x = 60; % Number of samples
N_y = N_x;
x_range = [-10,11];
y_range = [-11,8];

opt_sim.obstacle = obs;
opt_sim.attractor = [0,0];

opt_sim.simulationName = 'singleAttractor_severalObstacles';    
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)

%%
close all; clear all;

ds_handle = @(x) ellipseLimit_cycle(x);

N = 10;
R = 2;
phi = (1:N)/N*2*pi;
x0 = [cos(phi)*R ; sin(phi)*R].*rand(2,N)*2;

%x0 = x0(:,5);

% Place obstacles
obs = [];

% % obstacle 1
i=1;
obs{i}.a = [0.8;0.6];
obs{i}.p = [1;1];
obs{i}.x0 = [2;0];
obs{i}.sf = [1.2];
obs{i}.th_r = 60*pi/180;
i=2;
obs{i}.a = [0.5;1];
obs{i}.p = [1;1];
obs{i}.x0 = [2;3];
obs{i}.sf = [1.2];
obs{i}.th_r = -45*pi/180;

opt_sim.obstacle = obs;
opt_sim.attractor = 'None';
opt_sim.timeSteps = [0];

N_x = 60; % Number of samples
N_y = N_x;
x_range = [-3,5.5];
y_range = [-3,5];
opt_sim.saveFig = 1;

opt_sim.simulationName = 'limitCicle_twoObst';    
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)

%%
close all; clc;

ds_handle = @(x) linearStableDS(x);

N = 10;
R = 2;
phi = (1:N)/N*2*pi;
x0 = [cos(phi)*R ; sin(phi)*R].*rand(2,N)*2;

%x0 = x0(:,5);

% Place obstacles
obs = [];

% Place obstacles
obs = [];

% obstacle 1
i=1;

obs{i}.a = [1;5];
obs{i}.p = [1;1];
obs{i}.x0 = [5;-4];
obs{i}.sf = [1.2];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 8;
obs{i}.perturbation.dx = [-1;4];  
obs{i}.perturbation.w = -1;  

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

opt_sim.timeSteps = [0,2];
opt_sim.attractor = [0,0];

N_x = 60; % Number of samples
N_y = N_x;
x_range = [-5,26];
y_range = [-13,15];

opt_sim.simulationName = 'twoMovingRotatingObs';    
opt_sim.saveFig = 1;

tic
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)
time_total = toc


%%
close all; clc;

ds_handle = @(x) linearStableDS(x);

N = 10;
R = 2;
phi = (1:N)/N*2*pi;
x0 = [cos(phi)*R ; sin(phi)*R].*rand(2,N)*2;

%x0 = x0(:,5);

% Place obstacles
obs = [];

% Place obstacles
obs = [];

% obstacle 1
i=1;
obs{i}.a = [3;3];
obs{i}.p = [1;1];
obs{i}.x0 = [10;4];
obs{i}.sf = [1.2];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 4;
obs{i}.perturbation.tf = 10;
obs{i}.perturbation.dx = [-4;0];  
%obs{i}.perturbation.w = 1;  

opt_sim.obstacle = obs;

opt_sim.timeSteps = [0,2];
opt_sim.attractor = [0,0];

N_x = 60; % Number of samples
N_y = N_x;
x_range = [-10,16];
y_range = [-10,14];

opt_sim.simulationName = 'leftMoving_circle';    
opt_sim.saveFig = 1;

tic
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)
time_total = toc


%%
close all; clc;

ds_handle = @(x) linearStableDS(x);

N = 10;
R = 2;
phi = (1:N)/N*2*pi;
x0 = [cos(phi)*R ; sin(phi)*R].*rand(2,N)*2;

%x0 = x0(:,5);

% Place obstacles
obs = [];

% Place obstacles
obs = [];

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
obs{i}.perturbation.tf = 10;
obs{i}.perturbation.dx = [0;0];  
obs{i}.perturbation.w = 1;  

opt_sim.obstacle = obs;

opt_sim.timeSteps = [0,2.9];
opt_sim.attractor = [0,0];

N_x = 60; % Number of samples
N_y = N_x;
x_range = [-5,26];
y_range = [-13,15];

opt_sim.simulationName = 'rotating_ellipse';    
opt_sim.saveFig = 1;

tic
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)
time_total = toc


%%
close all; clc;

ds_handle = @(x) linearStableDS(x);

N = 10;
R = 2;
phi = (1:N)/N*2*pi;
x0 = [cos(phi)*R ; sin(phi)*R].*rand(2,N)*2;

%x0 = x0(:,5);

% Place obstacles
obs = [];

% Place obstacles
obs = [];

% Place obstacles
obs = [];

% obstacle 1
i=1;

obs{i}.a = [1;5];
obs{i}.p = [1;1];
obs{i}.x0 = [10;-10];
obs{i}.sf = [1.2];
obs{i}.th_r = 30*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 3;
obs{i}.perturbation.dx = [0;10];  

opt_sim.obstacle = obs;

opt_sim.timeSteps = [0,1.3];
opt_sim.attractor = [0,0];

N_x = 60; % Number of samples
N_y = N_x;
x_range = [-5,26];
y_range = [-13,15];

opt_sim.simulationName = 'fastMovingEllipse';    
opt_sim.saveFig = 1;

tic
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)
time_total = toc

%%
close all; clc;


xa = [3;2];
ds_handle = @(x) linearStableDS(x,xa);

N = 10;
R = 2;
phi = (1:N)/N*2*pi;
x0 = [cos(phi)*R ; sin(phi)*R].*rand(2,N)*2;

%x0 = x0(:,5);

% Place obstacles
obs = [];

% obstacle 1
i=1;

obs{i}.a = [1;4];
obs{i}.p = [1;1];
obs{i}.x0 = [0;0];
obs{i}.sf = [1];
obs{i}.th_r = 0*pi/180;


opt_sim.obstacle = obs;

opt_sim.attractor = xa;
opt_sim.timeSteps = [0];

opt_sim.saveFig = 1;
opt_sim.plotSaddleTrajectory = 1;

N_x = 60; % Number of samples
N_y = N_x;
x_range = [-9,9];
y_range = [-8,8];



opt_sim.simulationName = 'staticEllipse';    
opt_sim.saveFig = 1;

tic
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)
time_total = toc

%% ------------------------------------------------------------------------
close all; clc;

xAttract = [0;3];
ds_handle = @(x) linearStableDS(x, xAttract);

N = 10;
R = 2;
phi = (1:N)/N*2*pi;
x0 = [cos(phi)*R ; sin(phi)*R].*rand(2,N)*2;

%x0 = x0(:,5);

% Place obstacles
obs = [];

% Place obstacles

% obstacle 1
i=1;

obs{i}.a = [1;1];
obs{i}.p = [1;1];
obs{i}.x0 = [0;0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;
%obs{i}.perturbation.t0 = 0;
%obs{i}.perturbation.tf = 3;
%obs{i}.perturbation.dx = [0;10];  o

opt_sim.obstacle = obs;

opt_sim.timeSteps = [0,1.3];
opt_sim.attractor = [0,0];

N_x = 4; % Number of samples
N_y = N_x;
x_range = [-4,4];
y_range = [-4,4];

opt_sim.simulationName = 'comparisonnvPython';    
opt_sim.saveFig = false;
opt_sim.timeSteps = [0]

[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)

% ------------------------------------------------------------------------
%%


opt_sim.timeSteps = [0,2];
opt_sim.attractor = [0,0];

N_x = 60; % Number of samples
N_y = N_x;
x_range = [-5,26];
y_range = [-13,15];


%%
headLength = 30;
headWidth = 30;
%ah = annotation('arrow',[0.3,0.5],[0.3,0.3])%,'LineWidth',3,...
ah = annotation('arrow', 'HeadLength',headLength,'HeadWidth',headWidth);
        %'HeadLength',4, 'HeadWidth',4);
set(ah, 'parent', gca);
set(ah,'position',[0.4,0.4, 10,-10])

