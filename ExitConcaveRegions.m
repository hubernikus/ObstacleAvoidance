function ExitConcaveRegions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%          Semester Project - HUBER Lukas, LASA Lab, EPFL  
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%
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

% Set default simulation parameters
opt_sim.dt = 0.005; %integration time steps
opt_sim.i_max = 300; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined

%% Mehsgrids
close all; clear variables

obs1{1}.a = [6;1];
obs1{1}.p = [1;1];
obs1{1}.x0 = [-4;0];
obs1{1}.sf = [1.2;1.2];
obs1{1}.th_r = 90*pi/180;

dim = 2; 
N_x = 21; % Number of samples
N_y = 21; 

X = [];
[X(:,:),Y(:,:)] = meshgrid(linspace(-10,0,N_x), linspace(-10,10,N_y));

xd_hat = zeros(dim, N_x, N_y);
xd = zeros(dim, N_x, N_y);

xd_obs = [0;0]; % assumed to not mobe

for ix = 1:N_x
    for iy = 1:N_y
        xd_hat(:,ix,iy) = linearStableDS([X(ix,iy);Y(ix,iy)]);
        xd(:,ix,iy) = obs_modulation_rotation([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy),obs1,xd_obs);
    end
end

%
[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs1,50);



figure('Position',[0 0 1000 1000]);
patch(x_obs(1,:),x_obs(2,:),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6]); hold on;
plot(x_obs_sf(1,:),x_obs_sf(2,:),'k--','linewidth',0.5);
quiver(X(:,:), Y(:,:), squeeze(xd_hat(1,:,:)), squeeze(xd_hat(2,:,:)), 'k'), hold on;
quiver(X(:,:), Y(:,:), squeeze(xd(1,:,:)), squeeze(xd(2,:,:)), 'r')
xlim([X(1,1),X(end,end)]);ylim([Y(1,1),Y(end,end)])

print('fig/quiverPlot_DS_LS_object61','-depsc')



%% Mehsgrids

obs = [];
obs1{1}.a = [6;1];
obs1{1}.p = [1;1];
obs1{1}.x0 = [-4;2];
obs1{1}.sf = [1.2;1.2]; 
obs1{1}.th_r = 45*pi/180;
% obs1{2}.a = [6;1];
% obs1{2}.p = [1;1];
% obs1{2}.x0 = [-4;-2];
% obs1{2}.sf = [1.2;1.2];
% obs1{2}.th_r = 45*pi/180;

dim = 2; 

N_x = 21; % Number of samples
N_y = 21; 

X = [];
[X(:,:),Y(:,:)] = meshgrid(linspace(-10,0,N_x), linspace(-10,10,N_y));

xd_hat = zeros(dim, N_x, N_y);
xd = zeros(dim, N_x, N_y);

xd_obs = [0;0]; % assumed to not mobe

for ix = 1:N_x
    for iy = 1:N_y
        xd_hat(:,ix,iy) = linearStableDS([X(ix,iy);Y(ix,iy)]);
        xd(:,ix,iy) = obs_modulation_rotation([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy),obs1,xd_obs);
    end
end

%
[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs1,50);



figure('Position',[0 0 1000 1000]);
patch(x_obs(1,:),x_obs(2,:),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6]); hold on;
plot(x_obs_sf(1,:),x_obs_sf(2,:),'k--','linewidth',0.5);
quiver(X(:,:), Y(:,:), squeeze(xd_hat(1,:,:)), squeeze(xd_hat(2,:,:)), 'k'), hold on;
quiver(X(:,:), Y(:,:), squeeze(xd(1,:,:)), squeeze(xd(2,:,:)), 'r')
xlim([X(1,1),X(end,end)]);ylim([Y(1,1),Y(end,end)])

print('fig/quiverPlot_DS_LS_object61','-depsc')

%% Specific points
%close all;

x1 = [-8; 1];
x2 = [-8; -1];
x3 = [-7; 2];
x4 = [-7; -2];

xList = { x1, x2, x3, x4};

xd = []; phi_c =[]; h_x = []; kappa = [];
var_gen = []; 


for ix = 1:4%length(xList}
    x = xList{ix};
    [xd_hat] = linearStableDS(x)
    [xd(:,ix),phi_c(ix), h_x(ix), kappa(ix), var_gen(:,ix)] ...
            = obs_modulation_rotation(x,xd_hat,obs1,xd_obs, true);
end

var_gen = var_gen

plot(0,0,'ob'); hold on;


%% Demo: 2D - multiple objects overlapping
close all;

fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

%x0 = randn(2,15)*10; % randomly distributed points
%N_samples = 20;
%phi0 = (1:N_samples)/N_samples*2*pi;
%r0 = 10;
% x0 = [r0* cos(phi0); r0*sin(phi0)];
x0 = [-10;0];
x0 = [ones(1,27)*-10 ; -13:1:13];


% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [6;-1];
obs{1}.p = [1;1];
obs{1}.x0 = [-5;0];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 90*pi/180;
%obs{i}.partition = [];
% obs{1}.perturbation.t0 = 0;
% obs{1}.perturbation.tf = 10;
% obs{1}.perturbation.dx = [0;-1];  

% obstacle 2
% obs{2}.a = [6;1];
% obs{2}.p = [1;1];
% obs{2}.x0 = [-8;-3];
% obs{2}.sf = [1.2;1.2];
% obs{2}.th_r = 45*pi/180;
% obs{2}.perturbation.t0 = 0;
% obs{2}.perturbation.tf = 10;

% obs{2}.perturbation.dx = [0;1];


% Start simulation
opt_sim.obstacle = obs;


fig(1) = figure('name','localRotation_avoidCicrcle','position',[100 550 560 420]);
%xlim([-15 15]); ylim([-15 15])

opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');

%%
%% Demo: 2D - multiple objects overlapping
clc;
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 30;
%phi0 = (1:N_samples)/N_samples*2*pi;
%r0 = 10;
%x0 = [r0* cos(phi0); r0*sin(phi0)];
x0 = [ones(1,15)*-10 ; -14:2:14];

% Place obstacles
obs = [];
i = 1;
obs{i}.a = [1;4];
obs{i}.p = [1;1];
obs{i}.x0 = [-5;2];
obs{i}.sf = [1.1;1.1];
obs{i}.th_r = 45*pi/180;
i = 2;
obs{i}.a = [1;4];
obs{i}.p = [1;1];
obs{i}.x0 = [-5;-4];
obs{i}.sf = [1.1;1.1];
obs{i}.th_r = -45*pi/180;



% Start simulation
opt_sim.obstacle = obs;

fig(1) = figure('name','localRotation_avoidEllipses_startingOutside','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC
fprintf('End 2D-Simulation \n');


%% Demo: 2D - multiple objects overlapping
clc;
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 30;
%phi0 = (1:N_samples)/N_samples*2*pi;
%r0 = 10;
%x0 = [r0* cos(phi0); r0*sin(phi0)];
x0 = [linspace(-8,-4,4),linspace(-8,-6,4);
      linspace(-5,-1,4),linspace(3,-2,4)];

% Place obstacles
obs = [];
i = 1;
obs{i}.a = [1;4];
obs{i}.p = [1;1];
obs{i}.x0 = [-5;2];
obs{i}.sf = [1.1;1.1];
obs{i}.th_r = 45*pi/180;
i = 2;
obs{i}.a = [1;4];
obs{i}.p = [1;1];
obs{i}.x0 = [-5;-4];
obs{i}.sf = [1.1;1.1];
obs{i}.th_r = -45*pi/180;



% Start simulation
opt_sim.obstacle = obs;

fig(1) = figure('name','localRotation_avoidEllipses_startingInside','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC
fprintf('End 2D-Simulation \n');



%% Demo: 2D - multiple objects overlapping
clc; %close all;
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 20;
%y = -10:1:10;
%x0 = [-10*ones(1,length(y)); y]; 
y = 0;
x0 = [linspace(1,1,1);
      linspace(1,1,1)]; 


% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [1; 2];
obs{1}.p = [1; 1];
%obs{1}.partition = [-pi 0];
obs{1}.x0 = [-6;1];
obs{1}.sf = [1.1];
obs{1}.th_r = 0*pi/180;
obs{1}.perturbation.t0 = 0;
obs{1}.perturbation.tf = 2;
obs{1}.perturbation.dx = [0;-1];  
% obstacle 2
% obs{2}.a = [2 2;0.4 1];
% obs{2}.p = `[2 1;1 1];
% obs{2}.partition = [-pi 0;0 pi];
% obs{2}.x0 = [-5;-3];
% obs{2}.sf = [1.2;1.2];
% obs{2}.th_r = -90*pi/180;
% % obstacle 3
% obs{3}.a = [1.2 1.2;0.4 1];
% obs{3}.p = [2 1;1 1];
% obs{3}.partition = [-pi 0;0 pi];
% obs{3}.x0 = [-5;-1];
% obs{3}.sf = [2;2];
% obs{3}.th_r = 60*pi/180;


% Start simulation
opt_sim.obstacle = obs;

fig(1) = figure('name','FlocalRotation_avoidEllipse','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC
fprintf('End 2D-Simulation \n');


end