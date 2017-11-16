function DS_firstTries_LH
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
if isempty(regexp(path,['Example_DSs' pathsep], 'once'))
    addpath([pwd, '/Example_DSs']);
end

% Set default simulation parameters
opt_sim.dt = 0.01; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined

%adding the obstacle avoidance folder to the MATLAB path directories
if isempty(regexp(path,['lib_obstacle_avoidance' pathsep], 'once'))
    addpath([pwd, '/lib_obstacle_avoidance']);
end
%adding the example folder to the MATLAB path directories
if isempty(regexp(path,['Example_DSs' pathsep], 'once'))
    addpath([pwd, '/Example_DSs']);
end


%% Demo: 2D - linear fixed point attractor
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 20;
phi0 = (1:N_samples)/N_samples*2*pi;
r0 = 10;
x0 = [r0* cos(phi0); r0*sin(phi0)];

% Start simulation
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim);



%% Demo: 3D - linear fixed point attractor
fprintf('Start 3D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(3,15)*10; % randomly distributed points
N_ang = 10;
ang = (1:N_ang)/N_ang*2*pi;
r0 = 10;
x0 = zeros(3,N_ang^2);
for ii = 1:N_ang
    for jj = 1:N_ang
        x0(:,jj+ii*(N_ang-1)) = [r0* cos(ang(ii)).*cos(ang(jj)); r0*sin(ang(ii)).*cos(ang(jj)); r0*sin(ang(jj))];
    end
end

% Start simulation
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim);

%% Demo: 3D - nonlinear fixed point attractor 
fprintf('Start 3D-Simulation of linear system\n');

fn_handle = @(x) stableDS_3D(x);
x0 = randn(3,15)*10; % randomly distributed points


opt_sim.obstacle = []; %no obstacle is defined

% Start simulation
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim);

%% Demo: 3D - nonlinear limit cycle
fprintf('Start 3D-Simulation <<Limit Cycle>> \n');

fn_handle = @(x) stableLimitcycle_DS(x);
x0 = (rand(3,15)-0.5)*20; % randomly distributed points !!! NO BIGER THAN 10, for attraction...

% Start simulation
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim);


%% Demo: 2D - linear fixed point attractor with objects
fprintf('Start 2D-Simulation \n');


% Dynamical system dot(x) = f(x)
fn_handle = @(x) linearStableDS(x);

% Insitial points
N_x = 20;
phi0  = (1:N_x)/N_x*2*pi;
r0 = 10;
x0 = [cos(phi0)*r0;sin(phi0)*r0];

% Obstacle1
obs{1}.a = 2*[1.2 1.2;0.4 1];
obs{1}.p = [2 1;1 1];
obs{1}.partition = [-pi 0;0 pi];
obs{1}.x0 = [-6.5;0.1];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 0*pi/180;
opt_sim.obstacle = obs;
% Obstacle
obs{2}.a = 1.5*[1.2 1.2;0.4 1];
obs{2}.p = [2 1;1 1];
obs{2}.partition = [-pi 0;0 pi];
obs{2}.x0 = [4;4];
obs{2}.sf = [1.2;1.2];
obs{2}.th_r = 60*pi/180;
opt_sim.obstacle = obs;
% Obstacle
obs{3}.a = [1.2 1.2;0.4 1];
obs{3}.p = [2 1;1 1];
obs{3}.partition = [-pi 0;0 pi];
% obs{3}.x0 = [-6;-4];
obs{3}.x0 = [-6;-2]; %% UNCOMMENTING LEADS TO non convex space...
obs{3}.sf = [1.2;1.2];
obs{3}.th_r = -100*pi/180;
opt_sim.obstacle = obs;

% Start simulation
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % not working, cause singularities


fprintf('End 2D-Simulation with obstacles. \n')

%% Demo: 3D - nonlinear fixed point attractor 
close all;

fprintf('Start 3D-Simulation of linear system\n');

fn_handle = @(x) linearStableDS(x);

%x0 = randn(3,15)*10; % randomly distributed points

N_x = 10;
x0 = zeros(3,N_x^2);
for ii = 1:N_x
    for jj = 1:N_x
        x0(:,ii+(jj-1)*N_x) = [cos(ii/N_x*2*pi)*[cos(jj/N_x*2*pi);sin(jj/N_x*2*pi)];sin(ii/N_x*2*pi)];
    end
end
r0 = 8;
x0 = r0*x0;

obs = []
obs{1}.a  = [1;2;3];
obs{1}.p  = [3;2;2];
obs{1}.x0 = [4;0.5;2];
obs{1}.sf = [1;1;1];
obs{1}.th_r = [0 0 0]*pi/180;
obs{1}.partition = [-pi pi];

opt_sim.obstacle = []; %no obstacle is defined
opt_sim.obstacle = obs;
%fn_handle = @(x) stableLimitcycle_DS(x);
%x0 = (rand(3,15)-0.5)*20; % randomly distributed points !!! NO BIGER THAN 10, for attraction...

% Start simulation
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim);






%% Demo: 2D - multiple objects
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 20;
phi0 = (1:N_samples)/N_samples*2*pi;
r0 = 10;
x0 = [r0* cos(phi0); r0*sin(phi0)];


% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [1.2 1.2;0.4 1];
obs{1}.p = [2 1;1 1];
obs{1}.partition = [-pi 0;0 pi];
obs{1}.x0 = [-8;0];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 0*pi/180;
% obstacle 2    
obs{2}.a = [1.2 1.2;0.4 1];
obs{2}.p = [2 1;1 1];
obs{2}.partition = [-pi 0;0 pi];
obs{2}.x0 = [0;7];
obs{2}.sf = [1.2;1.2];
obs{2}.th_r = -60*pi/180;
% obstacle 3
obs{3}.a = [1.2 1.2;0.4 1];
obs{3}.p = [2 1;1 1];
obs{3}.partition = [-pi 0;0 pi];
obs{3}.x0 = [+1;-4];
obs{3}.sf = [2;2];
obs{3}.th_r = 60*pi/180;
opt_sim.obstacle = obs;


% Start simulation
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim);
fprintf('End 2D-Simulation \n');

%% Demo: 2D - multiple objects overlapping
clc;
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
obs{1}.a = [1; 4];
obs{1}.p = [1; 1];
%obs{1}.partition = [-pi 0];
obs{1}.x0 = [-6;1];
obs{1}.sf = [1.1];
obs{1}.th_r = 0*pi/180;
% obstacle 2
% obs{2}.a = [2 2;0.4 1];
% obs{2}.p = [2 1;1 1];
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