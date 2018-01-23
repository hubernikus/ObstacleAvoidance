function DS_comparison_algorithms
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

ds_handle = @(x) linearStableDS(x);

opt_sim.obstacle = []; %no obstacle is defined
obs{1}.a = [2;2];
obs{1}.p = [1;1];
obs{1}.x0 = [-2;3];

obs{1}.sf = [1.2;1.2];
obs{1}.th_r = -30*pi/180;

N_x = 41; % Number of samples
N_y = N_x;
x_range = [-14,7];
y_range = [-5,15];

opt_sim.obstacle = obs;
opt_sim.simulationName = 'oneStaticCirc';

[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)

%%
clc; close all;

ds_handle = @(x) linearStableDS(x);

opt_sim.obstacle = []; %no obstacle is defined
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
opt_sim.simulationName = 'oneStaticEllipse_medium';
    
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)


%%
clc; close all;

ds_handle = @(x) linearStableDS(x);

obs = []
obs{1}.a = [1;4];
obs{1}.p = [1;1];
obs{1}.x0 = [-2;3];

obs{1}.sf = [1.2;1.2];
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

N_x = 10; % Number of samples
N_y = N_x;
x_range = [-3,5.5];
y_range = [-3,5];

opt_sim.simulationName = 'limitCicle_twoObst';    
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)

%%
close all; clear all;

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

opt_sim.timeSteps = [0,4];

N_x = 10; % Number of samples
N_y = N_x;
x_range = [-3,20];
y_range = [-10,10];

opt_sim.simulationName = 'twoMovingRotatingObs';    
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim)


%% Mehsgrids
close all; % clear variables
clc; 

clear obs;
obs{1}.a = [1;6];
obs{1}.p = [1;1];
obs{1}.x0 = [-2;3];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = -30*pi/180;
n= 1;
%obs1{n}.extra.ind = 2;
%obs1{n}.extra.C_Amp = 0.01;
%obs1{n}.extra.R_Amp = 0.0;

dim = 2; 

N_x = 51; % Number of samples
N_y = N_x; X = [];
N_tot = N_x*N_y;


[X,Y] = meshgrid(linspace(-10,2,N_x), linspace(-10,10,N_y));

xd_hat = zeros(dim, N_x, N_y);
xd_fluid = zeros(dim, N_x, N_y);
xd_ellips = zeros(dim, N_x, N_y);
xd_rot = zeros(dim, N_x, N_y);

xd_obs = [0;0]; % assumed to not moving
w_obs = [0]; % assumed to not moving

b_contour = 0; % Not contouring object

for ix = 1:N_x
    for iy = 1:N_y
        coll = obs_check_collision(obs,[X(ix,iy);Y(ix,iy)]);
        if(coll)
            xd_hat(:,ix,iy)=0;
            xd_fluid(:,ix,iy)=0;
            xd_ellips(:,ix,iy)=0;
            xd_rot(:,ix,iy)=0;
        else
            xd_hat(:,ix,iy) = stableDS([X(ix,iy);Y(ix,iy)]);
            xd_fluid(:,ix,iy) = obs_modulation_fluidMechanics([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs, b_contour, xd_obs,w_obs);
            xd_ellips(:,ix,iy) = obs_modulation_ellipsoid([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs, b_contour, xd_obs,w_obs);
            xd_rot(:,ix,iy) = obs_modulation_rotation([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs, b_contour, xd_obs,w_obs);
        end
    end
end

% 
relativeChangeSqr_fluid = sum(sum(sum((xd_fluid - xd_hat).^2)))/N_tot;
relativeChangeSqr_ellip = sum(sum(sum((xd_ellips - xd_hat).^2)))/N_tot;
relativeChangeSqr_rot = sum(sum(sum((xd_rot - xd_hat).^2)))/N_tot;

[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs,50);

figure('Position',[0 0 1000 1000]);
set(groot,'DefaultAxesFontSize',14)
set(groot,'DefaultLineLineWidth',1.0)
patch(x_obs(1,:),x_obs(2,:),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6]); hold on;
plot(x_obs_sf(1,:),x_obs_sf(2,:),'k--','linewidth',0.5);

% Initial system
%quiver(X(:,:), Y(:,:), squeeze(xd_hat(1,:,:)), squeeze(xd_hat(2,:,:)), 'k'), hold on;
%figure
%streamslice(X(:,:), Y(:,:), squeeze(xd_ellips(1,:,:)), squeeze(xd_ellips(2,:,:)), 'b')
streamslice(X(:,:), Y(:,:), squeeze(xd_fluid(1,:,:)), squeeze(xd_fluid(2,:,:)), 'g')
%streamslice(X(:,:), Y(:,:), squeeze(xd_rot(1,:,:)), squeeze(xd_rot(2,:,:)), 'r')
%quiver(X(:,:), Y(:,:), squeeze(xd_ellips(1,:,:)), squeeze(xd_ellips(2,:,:)), 'b')
%quiver(X(:,:), Y(:,:), squeeze(xd_fluid(1,:,:)), squeeze(xd_fluid(2,:,:)), 'g')
%quiver(X(:,:), Y(:,:), squeeze(xd_rot(1,:,:)), squeeze(xd_rot(2,:,:)), 'r')
xlim([X(1,1),X(end,end)]);ylim([Y(1,1),Y(end,end)])
%legend('','','Original System', 'Ellipse Modulation', 'Ellipse Modulation Adaptded')
%drawArrow([-4,-4],[-5,-8],'linewidth',3,'color','r')
axis equal;
%legend('Object','Safety Margin','Original DS', 'DMM', 'IFD', 'LRS')
%print('fig/quiverPlot_DS_LS_circle_fluidMechanics','-depsc')


simulationName = 'relativeChange_1';
dataAnalysis = true;
if(dataAnalysis)
    fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
    fprintf(fileID,'Modulation Type & Relative Change of Dynamical System \\\\ \\hline \n');
    fprintf(fileID,'Fluid % 3.4f \\\\ \\hline \n', relativeChangeSqr_fluid);
    fprintf(fileID,'Rotation % 3.4f \\\\ \\hline \n', relativeChangeSqr_rot);
    fprintf(fileID,'Ellipse % 3.4f \\\\ \\hline \n', relativeChangeSqr_ellip);
    fclose(fileID);
end
