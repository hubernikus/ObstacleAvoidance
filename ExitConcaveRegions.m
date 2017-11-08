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
if isempty(regexp(path,['DynamicalSystems' pathsep], 'once'))
    addpath([pwd, '/DynamicalSystems']);
end

% Set default simulation parameters
opt_sim.dt = 0.1; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined


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


% Place obstacles
obs = [];
% obstacle 1
obs{1}.a = [6;1];
obs{1}.p = [1;1];
%obs{i}.partition = [];
obs{1}.x0 = [-4;-1];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 90*pi/180;
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


fig(1) = figure('name','First demo: Streamlines of the original DS','position',[100 550 560 420]);
xlim([-15 15]); ylim([-15 15])

opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC

xlim([-15 15]); ylim([-15 15])



fprintf('End 2D-Simulation \n');



end