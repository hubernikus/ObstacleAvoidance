%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%          Semester Project - HUBER Lukas, LASA Lab, EPFL
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%clear all;
%%
clc; close all; clear all;

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
opt_sim.dt = 0.01; %integration time steps
opt_sim.i_max = 400; %maximum number of iterations
opt_sim.tol = 0.1; % convergence tolerance
opt_sim.plot = true; % enabling the animation
opt_sim.model = 1; % first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined


%% Demo: 2D - linear fixed point attractor
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 20;
phi0 = (1:N_samples)/N_samples*2*pi;
r0 = 10;
x0 = [r0* cos(phi0); r0*sin(phi0)];


% Place obstacles
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



%% Demo: 2D - multiple objects overlapping -> moving into object
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 10;
phi0 = (1:N_samples)/N_samples*2*pi;
r0 = 10;
%x0 = [r0* cos(phi0); r0*sin(phi0)];
x0 = [-10*ones(1,10); (1:N_samples)/N_samples*20-8];

% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [1 1;2 2];
obs{1}.p = [1 1;1 1];
obs{1}.partition = [-pi 0;0 pi];
obs{1}.x0 = [-6;1];
obs{1}.sf = [1.2;1.2];
obs{1}.rho = 1.5;
obs{1}.th_r = 0*pi/180;
% obstacle 2
obs{2}.a = [0.5 0.5; 2 2];
obs{2}.p = [1 1;1 1];
obs{2}.partition = [-pi 0;0 pi];
obs{2}.x0 = [-8;-5];
obs{2}.sf = [1.2;1.2];
obs{2}.rho = 1.5;  % Reactivity
obs{2}.th_r = 0*pi/180;
obs{2}.perturbation.t0 = 0;
obs{2}.perturbation.tf = 10;
obs{2}.perturbation.dx = [0;1];


opt_sim.obstacle = obs;

% Start simulation
close all;
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim);
fprintf('End 2D-Simulation \n');



%% Demo: 2D - multiple objects overlapping -> moving into object
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 10;
phi0 = (1:N_samples)/N_samples*2*pi;
r0 = 10;
%x0 = [r0* cos(phi0); r0*sin(phi0)];
x0 = [-10*ones(1,10); (1:N_samples)/N_samples*20-8];

% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [1 1;2 2];
obs{1}.p = [1 1;1 1];
obs{1}.partition = [-pi 0;0 pi];
obs{1}.x0 = [0;0];
obs{1}.sf = [1.2;1.2];
obs{1}.rho = 1.5;
obs{1}.th_r = 0*pi/180;
obs{1}.perturbation.t0 = 0;
obs{1}.perturbation.tf = 10;
obs{1}.perturbation.dx = [-1;0];


opt_sim.obstacle = obs;

% Start simulation
close all;
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim);
fprintf('End 2D-Simulation \n');


%% Demo: 2D - multiple objects overlapping -> moving into object
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 10;
phi0 = (1:N_samples)/N_samples*2*pi;
r0 = 10;
%x0 = [r0* cos(phi0); r0*sin(phi0)];
x0 = [-20*ones(1,10); (1:N_samples)/N_samples*20-8];

% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [5 5;5 5];
obs{1}.p = [1 1;1 1];
obs{1}.partition = [-pi 0;0 pi];
obs{1}.x0 = [-17;0];
obs{1}.sf = [1.2;1.2];
obs{1}.rho = 1.5;
obs{1}.th_r = 0*pi/180;
obs{1}.perturbation.t0 = 0;
obs{1}.perturbation.tf = 10;
obs{1}.perturbation.dx = [5;0];


opt_sim.obstacle = obs;

% Start simulation
close all;
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim);
fprintf('End 2D-Simulation \n');

%% Demo: 3D - nonlinear fixed point attractor
close all;

fprintf('Start 3D-Simulation of linear system\n');

fn_handle = @(x) linearStableDS(x);

%x0 = randn(3,15)*10; % randomly distributed points

N_x = 10;
x0 = zeros(3,N_x^2);
for ii = 1:N_x
    for jj = 1:N_x
        x0(:,ii+(jj-1)*N_x) = [cos(ii/N_x*pi)*[cos(jj/N_x*2*pi);sin(jj/N_x*2*pi)];sin(ii/N_x*pi)];
    end
end
r0 = 8;
x0 = r0*x0;

obs = []
obs{1}.a  = [10;10;1];
obs{1}.p  = [100;100;100];
obs{1}.x0 = [0;0;-2];
obs{1}.sf = [1;1;1];
obs{1}.th_r = [0 0 0]*pi/180;
obs{1}.partition = [-pi pi];
obs{2}.a  = [1;2;3];
obs{2}.p  = [3;2;2];
obs{2}.x0 = [4;0.5;2];
obs{2}.sf = [1;1;1];
obs{2}.th_r = [0 0 0]*pi/180;
obs{2}.partition = [-pi pi];
obs{3}.a  = [1;2;3];
obs{3}.p  = [3;2;2];
obs{3}.x0 = [-4;-4;2];
obs{3}.sf = [1;1;1];
obs{3}.th_r = [0 0 0]*pi/180;
obs{3}.partition = [-pi pi];
obs{3}.perturbation.t0 = 0;
obs{3}.perturbation.tf = 10;
obs{3}.perturbation.dx = [0;1;0];


opt_sim.obstacle = []; %no obstacle is defined
opt_sim.obstacle = obs;
%fn_handle = @(x) stableLimitcycle_DS(x);
%x0 = (rand(3,15)-0.5)*20; % randomly distributed points !!! NO BIGER THAN 10, for attraction...

% Start simulation
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim);

%% Demo: 2D - multiple objects overlapping -> moving into object
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 10;
phi0 = (1:N_samples)/N_samples*2*pi;
r0 = 10;
%x0 = [r0* cos(phi0); r0*sin(phi0)];
x0 = [-10*ones(1,10); (1:N_samples)/N_samples*20-8];


% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [1 1;2 2];
obs{1}.p = [1 1;1 1];
obs{1}.partition = [-pi 0;0 pi];
obs{1}.x0 = [-6;1];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 0*pi/180;
% obstacle 2
obs{2}.a = [0.5 0.5; 2 2];
obs{2}.p = [1 1;1 1];
obs{2}.partition = [-pi 0;0 pi];
obs{2}.x0 = [-6;0];
obs{2}.sf = [1.2;1.2];
obs{2}.th_r = 0*pi/180;
% obs{2}.perturbation.t0 = 0;
% obs{2}e.perturbation.tf = 10;
% obs{2}.perturbation.dx = [0;1];


opt_sim.obstacle = obs;

% Start simulation
close all;
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation_improved(x0,[],fn_handle,opt_sim);
fprintf('End 2D-Simulation \n');

%% Demo: 2D - multiple objects overlapping -> moving into object
fprintf('Start 2D-Simulation \n'); 

fn_handle = @(x) linearStableDS(x); 
%x0 = randn(2,15)*10; % randomly distributed points
N_samples = 10;
phi0 = (1:N_samples)/N_samples*2*pi;
r0 = 10;
%x0 = [r0* cos(phi0); r0*sin(phi0)];
x0 = [-10*ones(1,10); (1:N_samples)/N_samples*20-8];


% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [1 1;2 2];
obs{1}.p = [1 1;1 1];
obs{1}.partition = [-pi 0;0 pi];
obs{1}.x0 = [-6;1];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 0*pi/180;
% obstacle 2
obs{2}.a = [0.5 0.5; 2 2];
obs{2}.p = [1 1;1 1];
obs{2}.partition = [-pi 0;0 pi];
obs{2}.x0 = [-6;0];
obs{2}.sf = [1.2;1.2];
obs{2}.th_r = 0*pi/180;
% obs{2}.perturbation.t0 = 0; 
% obs{2}e.perturbation.tf = 10; 
% obs{2}.perturbation.dx = [0;1]; 


opt_sim.obstacle = obs;

% Start simulation
close all;
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation_improved(x0,[],fn_handle,opt_sim);
fprintf('End 2D-Simulation \n');

%% Demo: 2D - multiple objects overlapping -> moving into object
fprintf('Start 2D-Simulation \n'); 

% Set default simulation parameters
opt_sim.dt = 0.05; %integration time steps
opt_sim.i_max = 100; %maximum number of iterations
opt_sim.tol = 0.05; % convergence tolerance
opt_sim.plot = true; % enabling the animation
opt_sim.model = 1; % first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined

fn_handle = @(x) linearStableDS(x); 
N_samples = 10;
phi0 = (1:N_samples)/N_samples*2*pi;
r0 = 10;
x0 = [-10*ones(1,10); (1:N_samples)/N_samples*20-8];

% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [1 1;2 2];
obs{1}.p = [1 1;1 1];
obs{1}.partition = [-pi 0;0 pi];
obs{1}.x0 = [-6;1];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 0*pi/180;



opt_sim.obstacle = obs;

% Start simulation
close all;
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
opt_sim.figure = fig(1);
Simulation_improved(x0,[],fn_handle,opt_sim);
fprintf('End 2D-Simulation \n');
