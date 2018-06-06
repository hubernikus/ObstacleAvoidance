function Tutorial_Obstacle_Avoidance_2018
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%          Copyright (c) 2018 S. Lukas Huber, LASA Lab, EPFL,         %%%
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
warning('off','all') % turn warnings offjr
%% preparing the obstacle avoidance module
%adding the obstacle avoidance folder to the MATLAB path directories
if isempty(regexp(path,['lib_obstacle_avoidance' pathsep], 'once'))
    addpath([pwd, '/lib_obstacle_avoidance']);
end
if isempty(regexp(path,['DynamicalSystems' pathsep], 'once'))
    addpath([pwd, '/DynamicalSystems']);
end
if isempty(regexp(path,['lib_simulation_tools' pathsep], 'once'))
    addpath([pwd, '/lib_simulation_tools']);
end

%%

fn_handle = @(x) linearStableDS_const(x); %defining the function handle
x0 = [-18*ones(1,15);linspace(-10,10,15)]; %set of initial points
% A set of parameters that should be defined for the simulation
opt_sim.dt = 0.02; %integration time steps

opt_sim.i_max = 600; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation


clear obs;
obs{1}.a = [1.2; 3];
obs{1}.p = [2;1];
obs{1}.x0 = [-8;0];
obs{1}.sf = [1.2];
obs{1}.th_r = 0*pi/180;
opt_sim.obstacle = obs;


obs{1}.x0 = [-4 ;0];
obs{2} = obs{1};
obs{2}.x0 = [-12;3];
obs{2}.th_r = 90*pi/180;
obs{3} = obs{1};
obs{3}.x0 = [-12;-3];
obs{3}.th_r = -90*pi/180;
disp('press any key to draw the streamlines in the presence of three obstacles ...')

opt_sim.obstacle = obs;
fig(1) = figure('name','1st demo: Multiple obstacle avoidance','position',[000 00 1500 800]);
opt_sim.figure = fig(1); 


opt_sim.saveAnimation=true;
Simulation_comparison(x0,[],fn_handle,opt_sim);

close all;

%% pause
% obstacles are intersecting
disp(' ')
disp(' ')
disp('In the second demo, intersecting obstacles are observed')
disp('the shape of the obstacles varied, and furthermore')
disp('they are placed such that they intersect with each other.')
disp('First lets observe an intersection, where the concave region')
disp('is facing towards the incoming initital DS.')



obs = [];
obs{1}.a = [1.2; 3.5];
obs{1}.p = [1;1];
obs{1}.x0 = [-8;-2.5];
obs{1}.sf = [1.2];
obs{1}.th_r = 110*pi/180;

obs{2}.a = [1.2;3.5];
obs{2}.p = [1;1];
obs{2}.x0 = [-6;1];
obs{2}.sf = [1.2];
obs{2}.th_r = 30*pi/180;

obs{1}.x0 = [-10;0.5];
obs{1}.th_r = -40*pi/180;

obs{2}.x0 = [-6;1];
obs{2}.th_r = 30*pi/180;

disp('press any key to draw the streamlines in the presence of a concave obstacle ...')

% pause
opt_sim.obstacle = obs;
fig(1) = figure('name','2nd demo: Concave regions (2)','position',[000 00 2000 1000]);
opt_sim.figure = fig(1); 
Simulation_comparison(x0,[],fn_handle,opt_sim);


%% pause

disp(' ')
disp(' ')
disp('In following demo, we observe two moving obstacles. The first ')
disp('is moving in such a way, that it collides with the second one and ')
disp('temporarily forms a concave region. The center of the obstacles ')
disp('circle has to dynamically adapt to compensate for this.')
opt_sim.dt = 0.01; %integration time steps

obs = []
obs{1}.th_r = 60*pi/180;
obs{1}.sf = [1.2];
obs{1}.a = [1.2;4];
obs{1}.p = [1;1];
obs{1}.x0 = [-8;8];
obs{1}.perturbation.w = 0;  
obs{1}.perturbation.t0 = 0;
obs{1}.perturbation.tf = 4;
obs{1}.perturbation.dx = [0;-5];


obs{2} = obs{1};
obs{2}.x0 = [-8;-2];
obs{2}.th_r = -80*pi/180;
obs{2}.perturbation.dx = [0;-0];

disp('press any key to draw the streamlines in the presence of three obstacles ...')

% pause
opt_sim.obstacle = obs;
fig(3) = figure('name','3rd demo: Multiple, moving obstacles','position',[000 00 2000 1000]);
opt_sim.figure = fig(3); 
Simulation_comparison(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')

%% pause

disp(' ')
disp(' ')
disp('In following demo, we observe two moving obstacles. The first ')
disp('is moving in such a way, that it collides with the second one and ')
disp('temporarily forms a concave region. The center of the obstacles ')
disp('circle has to dynamically adapt to compensate for this.')
opt_sim.dt = 0.01; %integration time steps

obs = []
obs{1}.th_r = -60*pi/180;
obs{1}.sf = [1.2];
obs{1}.a = [2;4.5];
obs{1}.p = [1;1];
obs{1}.x0 = [-8;-4];



obs{2} = obs{1};
obs{2}.x0 = [-8;3];
obs{2}.a = [1;5];
obs{2}.th_r = 60*pi/180;
obs{2}.perturbation.w = -2;  
obs{2}.perturbation.t0 = 0;
obs{2}.perturbation.tf = 4;
obs{2}.perturbation.dx = [0;0];

disp('press any key to draw the streamlines in the presence of three obstacles ...')

% pause
opt_sim.obstacle = obs;
fig(3) = figure('name','rotating_ellipse','position',[000 00 2000 1000]);
opt_sim.figure = fig(3); 
Simulation_comparison(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')

