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

%% Set default simulation parameters
opt_sim.obstacle = []; %no obstacle is defined

fn_handle_objAvoidance = @(x,xd,obs, varargin) ...
                          obs_modulation_convergence(x,xd,obs, varargin);
                      
N_points = 200;
                      

opt_sim.saveFig = false;

%% Ellipse with good centering
close all; clc;
fprintf('Start 2D-Simulation at: %s \n', datestr(datetime('now') ) ); tic;

opt_sim.x_attractor = [0;0];

% Turn into function
ds_handle = @(x,y) linearStableDS_erf([x;y],opt_sim.x_attractor);
%ds_handle = @(x,y) linearStableDS([x;y],opt_sim.x_attractor);

% Place obstacles
obs = [];
i=1;
obs{i}.a = [0.2;5.5];
obs{i}.p = [1;1];
obs{i}.x0 = [8;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 75*pi/180;
obs{i}.x_center = [0.0;0.0];

x_range = [-2,15]; y_range = [-8.8,8.8];

opt_sim.simulationName = 'elongatedEllipse_diag'
opt_sim.obstacle = obs;

divergence_analysis(ds_handle, fn_handle_objAvoidance, ...
                                  N_points, x_range, y_range, ...
                                  opt_sim)

timeEnd = toc; fprintf('\nEnd 2D-Simulation in %3.4f seconds. \n', timeEnd);

%% Ellipse with good centering
close all; clc;
fprintf('Start 2D-Simulation at: %s \n', datestr(datetime('now') ) ); tic;

opt_sim.x_attractor = [0;0];

% Turn into function
ds_handle = @(x,y) linearStableDS_erf([x;y],opt_sim.x_attractor);
%ds_handle = @(x,y) linearStableDS([x;y],opt_sim.x_attractor);

% Place obstacles
obs = [];
i=1;
obs{i}.a = [0.2;5.5];
obs{i}.p = [1;1];
obs{i}.x0 = [8;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;
obs{i}.x_center = [0.0;0.0];

x_range = [-2,15]; y_range = [-8.8,8.8];

opt_sim.simulationName = 'elongatedEllipse_vert'
opt_sim.obstacle = obs;

divergence_analysis(ds_handle, fn_handle_objAvoidance, ...
                                  N_points, x_range, y_range, ...
                                  opt_sim)

timeEnd = toc; fprintf('\nEnd 2D-Simulation in %3.4f seconds. \n', timeEnd);

%% Ellipse with good centering
close all; clc;
fprintf('Start 2D-Simulation at: %s \n', datestr(datetime('now') ) ); tic;

opt_sim.x_attractor = [0;0];

% Turn into function
ds_handle = @(x,y) linearStableDS_erf([x;y],opt_sim.x_attractor);
%ds_handle = @(x,y) linearStableDS([x;y],opt_sim.x_attractor);

% Place obstacles
obs = [];
i=1;
obs{i}.a = [5;5];
obs{i}.p = [1;1];
obs{i}.x0 = [8;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;
obs{i}.x_center = [0.0;0.0];

x_range = [-2,15]; y_range = [-8.8,8.8];

opt_sim.simulationName = 'circle'
opt_sim.obstacle = obs;

divergence_analysis(ds_handle, fn_handle_objAvoidance, ...
                                  N_points, x_range, y_range, ...
                                  opt_sim)

timeEnd = toc; fprintf('\nEnd 2D-Simulation in %3.4f seconds. \n', timeEnd);

%% Ellipse with good centering
close all; clc;
fprintf('Start 2D-Simulation at: %s \n', datestr(datetime('now') ) ); tic;

opt_sim.x_attractor = [0;0];

% Turn into function
ds_handle = @(x,y) linearStableDS_erf([x;y],opt_sim.x_attractor);
%ds_handle = @(x,y) linearStableDS([x;y],opt_sim.x_attractor);

% Place obstacles
obs = [];
i=1;
obs{i}.a = [0.2;5.5];
obs{i}.p = [1;1];
obs{i}.x0 = [8;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 90*pi/180;
obs{i}.x_center = [0.0;0.0];

x_range = [-2,15]; y_range = [-8.8,8.8];
opt_sim.saveFig = false;
opt_sim.simulationName = 'elongatedEllipse'
opt_sim.obstacle = obs;

divergence_analysis(ds_handle, fn_handle_objAvoidance, ...
                                  N_points, x_range, y_range, ...
                                  opt_sim)

timeEnd = toc; fprintf('\nEnd 2D-Simulation in %3.4f seconds. \n', timeEnd);

%% Ellipse with good centering
close all; clc;
fprintf('Start 2D-Simulation at: %s \n', datestr(datetime('now') ) ); tic;

N_points = 200;

opt_sim.x_attractor = [0;0];

% Turn into function
ds_handle = @(x,y) linearStableDS_erf([x;y],opt_sim.x_attractor);
%ds_handle = @(x,y) linearStableDS([x;y],opt_sim.x_attractor);

% Place obstacles
obs = [];
i=1;
obs{i}.a = [1;3];
obs{i}.p = [1;1];
obs{i}.x0 = [8;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 60*pi/180;
obs{i}.x_center = [0.0;0.0];

x_range = [-2,15]; y_range = [-8.8,8.8];

opt_sim.simulationName = 'normal_ellipse'
saveFig = false;

opt_sim.obstacle = obs;

divergence_analysis(ds_handle, fn_handle_objAvoidance, ...
                                  N_points, x_range, y_range, ...
                                  opt_sim)
%                             
% Draw lines
m_cl = (obs{1}.x0(2)-opt_sim.x_attractor(2))/ ...
               (obs{1}.x0(1)-opt_sim.x_attractor(1));
           
dy = opt_sim.x_attractor(2);

%
plot([x_range(1),obs{1}.x0(1)],...
     [dy+m_cl*x_range(1),dy+m_cl*obs{1}.x0(1)], ...
     '-','LineWidth',3,'Color',[34,139,34]/255); hold on;
 
plot([x_range(2),obs{1}.x0(1)],...
     [dy+m_cl*x_range(2),dy+m_cl*obs{1}.x0(1)], ...
     '-','LineWidth',3,'Color',[255,140,0]/255)
 

m_top = tan(atan(m_cl)+dPhi);
dy_top = dy+m_top*(opt_sim.x_attractor(1) - obs{1}.x0(1));

m_bot = tan(atan(m_cl)-dPhi);
dy_bot = dy+m_bot*(opt_sim.x_attractor(1) - obs{1}.x0(1) );


plot([x_range(1),obs{1}.x0(1)],...
     [dy_top+m_top*x_range(1),dy_top+m_top*obs{1}.x0(1)], ...
     '--','LineWidth',2,'Color',[34,139,34]/255); hold on;
 
plot([x_range(1),obs{1}.x0(1)],...
     [dy_bot+m_bot*x_range(1),dy_bot+m_bot*obs{1}.x0(1)], ...
     '--','LineWidth',2,'Color',[34,139,34]/255); hold on;

print(strcat('fig_vector/divergence_showConvergenceRegion','.eps'),'-depsc')

timeEnd = toc; fprintf('\nEnd 2D-Simulation in %3.4f seconds. \n', timeEnd);
