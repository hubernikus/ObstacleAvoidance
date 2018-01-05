function AvoidMovingObject
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
opt_sim.i_max = 400; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined


%% Mehsgrids
close all; % clear variables
clc; 

obs = [];
obs1{1}.a = [1;4];
obs1{1}.p = [1;1];
obs1{1}.x0 = [-2;3];
obs1{1}.sf = [1.2;1.2];
obs1{1}.th_r = -30*pi/180;
n= 1;
obs1{n}.extra.ind = 2;
obs1{n}.extra.C_Amp = 0.01;
obs1{n}.extra.R_Amp = 0.0;

dim = 2; 

N_x = 21; % Number of samples
N_y = N_x; X = [];
N_tot = N_x*N_y;

[X(:,:),Y(:,:)] = meshgrid(linspace(-10,10,N_x), linspace(-6,15,N_y));

xd_hat = zeros(dim, N_x, N_y);
xd_fluid = zeros(dim, N_x, N_y);
xd_ellips = zeros(dim, N_x, N_y);
xd_rot = zeros(dim, N_x, N_y);

xd_obs = [0;0]; % assumed to not moving
w_obs = [0]; % assumed to not moving

b_contour = 0; % Not contouring object

for ix = 1:N_x
    for iy = 1:N_y
        coll = obs_check_collision(obs1,[X(ii,jj);Y(ii,jj)]);
        if(coll)
            xd_hat(:,ii,jj)=0;
            xd_fluid(:,ii,jj)=0;
            xd_ellips(:,ii,jj)=0;
            xd_rot(:,ii,jj)=0;
        else
            xd_hat(:,ix,iy) = stableDS([X(ix,iy);Y(ix,iy)]);
            xd_fluid(:,ix,iy) = obs_modulation_fluidMechanics([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs1, b_contour, xd_obs,w_obs);
            xd_ellips(:,ix,iy) = obs_modulation_ellipsoid([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs1, b_contour, xd_obs,w_obs);
            xd_rot(:,ix,iy) = obs_modulation_rotation([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs1, b_contour, xd_obs,w_obs);
        end
    end
end

% 
relativeChangeSqr_fluid = sum(sum(sum((xd_fluid - xd_hat).^2)))/N_tot;
relativeChangeSqr_ellip = sum(sum(sum((xd_ellips - xd_hat).^2)))/N_tot;
relativeChangeSqr_rot = sum(sum(sum((xd_rot - xd_hat).^2)))/N_tot;

[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs1,50);

figure('Position',[0 0 1000 1000]);
patch(x_obs(1,:),x_obs(2,:),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6]); hold on;
plot(x_obs_sf(1,:),x_obs_sf(2,:),'k--','linewidth',0.5);

% Initial system
quiver(X(:,:), Y(:,:), squeeze(xd_hat(1,:,:)), squeeze(xd_hat(2,:,:)), 'k'), hold on;
%figure
quiver(X(:,:), Y(:,:), squeeze(xd_ellips(1,:,:)), squeeze(xd_ellips(2,:,:)), 'b')
quiver(X(:,:), Y(:,:), squeeze(xd_fluid(1,:,:)), squeeze(xd_fluid(2,:,:)), 'g')
quiver(X(:,:), Y(:,:), squeeze(xd_rot(1,:,:)), squeeze(xd_rot(2,:,:)), 'r')
xlim([X(1,1),X(end,end)]);ylim([Y(1,1),Y(end,end)])
%legend('','','Original System', 'Ellipse Modulation', 'Ellipse Modulation Adaptded')
%drawArrow([-4,-4],[-5,-8],'linewidth',3,'color','r')
axis equal;
legend('','','Original', 'Ellipsoid', 'Fluid', 'Rotation')
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
%% Mehsgrids

obs = [];
obs1{1}.a = [6;1];
obs1{1}.p = [1;1];
obs1{1}.x0 = [-4;2];
obs1{1}.sf = [1.2;1.2]; 
obs1{1}.th_r = 45*pi/180;

dim = 2; 

N_x = 21; % Number of samples
N_y = 21; 

X = [];
[X(:,:),Y(:,:)] = meshgrid(linspace(-10,3,N_x), linspace(-10,10,N_y));

xd_hat = zeros(dim, N_x, N_y);
xd_ellips = zeros(dim, N_x, N_y);
xd_ellips2 = zeros(dim, N_x, N_y);

xd_obs = [0;0]; % assumed to not mobe

for ix = 1:N_x
    for iy = 1:N_y
        xd_hat(:,ix,iy) = linearStableDS([X(ix,iy);Y(ix,iy)]);
        b_contour = 0;
        xd_ellips2(:,ix,iy) = obs_modulation_ellipsoid_adapted([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs1, b_contour, xd_obs);
        xd_ellips(:,ix,iy) = obs_modulation_ellipsoid([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs1, b_contour, xd_obs);
    end
end

%
[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs1,50);




figure('Position',[0 0 1000 1000]);
patch(x_obs(1,:),x_obs(2,:),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6]); hold on;
plot(x_obs_sf(1,:),x_obs_sf(2,:),'k--','linewidth',0.5);
quiver(X(:,:), Y(:,:), squeeze(xd_hat(1,:,:)), squeeze(xd_hat(2,:,:)), 'k'), hold on;
quiver(X(:,:), Y(:,:), squeeze(xd_ellips(1,:,:)), squeeze(xd_ellips(2,:,:)), 'b')
quiver(X(:,:), Y(:,:), squeeze(xd_ellips2(1,:,:)), squeeze(xd_ellips2(2,:,:)), 'g')
xlim([X(1,1),X(end,end)]);ylim([Y(1,1),Y(end,end)])
%legend('','','Original System', 'Ellipse Modulation', 'Ellipse Modulation Adaptded')

print('fig/quiverPlot_DS_LS_ellipse_modulation','-depsc')



%%
%% Demo: 2D - multiple objects overlapping
close all; clc;

fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

%x0 = randn(2,15)*10; % randomly distributed points
%N_samples = 20;
%phi0 = (1:N_samples)/N_samples*2*pi;
%r0 = 10;
% x0 = [r0* cos(phi0); r0*sin(phi0)];
%x0 = [-10;-4];
x0 = [ones(1,27)*-10 ; -13:1:13];


% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [6;-1];
obs{1}.p = [1;1];
obs{1}.x0 = [-5;0];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 90*pi/180;


% Start simulation
opt_sim.obstacle = obs;


fig(1) = figure('name','khansariAdapt_moveAroundCircle','position',[100 550 560 420]);
%xlim([-15 15]); ylim([-15 15])

opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');

%%
%% Demo: 2D - multiple objects overlapping
close all;

fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

x0 = [-10;0];
x0 = [ones(1,27)*-10 ; -13:1:13];


% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [3;-1];
obs{1}.p = [1;1];
obs{1}.x0 = [-8;0];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 90*pi/180;
obs{1}.perturbation.t0 = 0;
obs{1}.perturbation.tf = 2;
obs{1}.perturbation.dx = [0;-10];  


% Start simulation
opt_sim.obstacle = obs;


fig(1) = figure('name','khansariAdapt_moveAroundCircle_moving','position',[100 550 560 420]);
%xlim([-15 15]); ylim([-15 15])

opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

%xlim([-15 15]); ylim([-15 15])

fprintf('End 2D-Simulation \n');
%% Demo: 2D - multiple objects overlapping
clc;
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 20;
%phi0 = (1:N_samples)/N_samples*2*pi;
%r0 = 10;
%x0 = [r0* cos(phi0); r0*sin(phi0)];
x0 = [ones(1,15)*-8 ; -14:2:14];

% Place obstacles
obs = [];
i = 1;
% obs{i}.a = [1 1;3 3];
% obs{i}.p = [1 1;1 1];
% obs{i}.partition = [-pi 0;0 pi];
% obs{i}.x0 = [-6;-5];
% obs{i}.sf = [1.2;1.2];
% obs{i}.th_r = 0*pi/180;
% i = 2;
obs{i}.a = [1;4];
obs{i}.p = [1;1];
%obs{i}.partition = [;0 pi];
obs{i}.x0 = [-6;1];
obs{i}.sf = [1.1;1.1];
obs{i}.th_r = 0*pi/180;
% % % obstacle 3
% obs{3}.a = [1.2 1.2;0.4 1];
% obs{3}.p = [2 1;1 1];
% obs{3}.partition = [-pi 0;0 pi];
% obs{3}.x0 = [-5;-1];
% obs{3}.sf = [2;2];
% obs{3}.th_r = 60*pi/180;


% Start simulation
opt_sim.obstacle = obs;

fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC
fprintf('End 2D-Simulation \n');

%% Demo: 2D - multiple objects overlapping
clc; close all;
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 20;
%y = -10:1:10;
%x0 = [-10*ones(1,length(y)); y]; 
y = 0;
x0 = [-10*ones(1,length(y)); y]; 


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

fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC
fprintf('End 2D-Simulation \n');


end