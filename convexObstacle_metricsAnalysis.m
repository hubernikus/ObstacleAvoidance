function convexObstacle_metricsAnalysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%          Semester Project - HUBER Lukas, LASA Lab, EPFL  
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


%clc; close all; clear variables;
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

% Set default simulation parameters
opt_sim.dt = 0.003; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined

%% Ellipse with good centering
weightType = 'inverseGamma';
opt_sim.weightType = weightType;



%close all; clc;
fprintf('Start 2D-Simulation \n');

ds_handle = @(x) linearStableDS(x);
fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
                          obs_modulation_convergence(x,xd,obs,varargin);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-15,15,N)];

% Place obstacles
obs = [];
i=1;
obs{i}.a = [4;4];
obs{i}.p = [1;1];
obs{i}.x0 = [10;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;

i=2;
obs{i}.a = [2;6];
obs{i}.p = [1;1];
obs{i}.x0 = [-9;7];
obs{i}.sf = [1.0];
obs{i}.th_r = -30*pi/180;

i=3;
obs{i}.a = [1;5];
obs{i}.p = [1;1];
obs{i}.x0 = [8;-8];
obs{i}.sf = [1.0];
obs{i}.th_r = 45*pi/180;

i=4;
obs{i}.a = [2;2];
obs{i}.p = [1;1];
obs{i}.x0 = [-2;-7];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;

% Start simulation
opt_sim.obstacle = obs;
opt_sim.ds_handle = ds_handle;
opt_sim.obstacleAvoidanceFunction = fn_handle_objAvoidance;
opt_sim.saveFig = true;

% Simulation Parameters
N_x = 100;  N_y = N_x;

scale = 15;
dx = 3;
dy = 0;
x_range = dx+scale*[-1.125,1.125]; y_range = dy+scale*[-1.05,1.05];

opt_sim.simulationName = strcat('fourObstacles_weightType_', weightType)
[metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, opt_sim);

% Add y-vals
y_vals = [-7.5,0,10];
for iy = 1:length(y_vals)
    plot(x_range,[y_vals(iy),y_vals(iy)],'k--','LineWidth',3)
end
set(gca,'ytick',y_vals'); 
print(strcat('fig_vector/',opt_sim.simulationName,'_yTicks'), '-depsc','-r300');

%% 
clc;


%figure('Position', [200,200,300,300] )

% Dimenson of space R^dim
dim = size(obs{1}.x0,1);

% Evaluate importance of each obstacle
x_vals = linspace(x_range(1), x_range(2), N_x);

% Calculate rotation matrices
rot_matrices = zeros(dim, dim, length(obs));
for it_obs = 1:length(obs)
    if isfield(obs{it_obs},'th_r')
        rot_matrices(:,:,it_obs) = compute_R(dim,obs{it_obs}.th_r);
    else
        rot_matrices(:,:,it_obs) = eye(dim);
    end
end

% Distance to obstacle
Gamma = zeros(length(obs),1);
distMeas_min = 1; % Gamma = 1 on surface

N_obs = length(obs); % Number of obstacles

ds_weights = zeros(length(obs), N_x, length(y_vals));

for iy = 1:length(y_vals)
    y = y_vals(iy);
    
    for ix = 1:N_x
        for it_obs = 1:N_obs
            x_t = rot_matrices(:,:,it_obs)'*([x_vals(ix); y_vals(iy)] - obs{it_obs}.x0);
            
            Gamma(it_obs) = sum((x_t./obs{it_obs}.a).^(2*obs{it_obs}.p)); 
        end
        ds_weights(:, ix, iy) = compute_weights(Gamma, N_obs, distMeas_min, weightType);
    end
end


%%
clc;

for iy = 1:length(y_vals)
    figure('Position',[100 100 400 250]);
    plot(x_vals, ds_weights(1,:,iy), 'Color',[204,0,0]/255); hold on;   % Red
    plot(x_vals, ds_weights(2,:,iy), 'Color', [0,153,0]/255)            % Green
    plot(x_vals, ds_weights(3,:,iy), 'Color', [0,0,204]/255)            % Blue
    plot(x_vals, ds_weights(4,:,iy), 'Color', [255,128,0]/255)          % Orange
    plot(x_vals, sum(ds_weights(:,:,iy),1), 'k--')

    %set(gca,'xtick',[],'ytick',[0,1]); box on;
    set(gca,'ytick',[0,1]); 
    box off;
    xlim([x_vals(1),x_vals(end)])
    ylim([0,1])
    ylabel('Weight'); xlabel('$\xi_1$','interpreter','latex')
    
    yVal_str = num2str(round(abs(y_vals(iy))));
    if y_vals(iy) < 0
        yVal_str = strcat('neg',yVal_str);
    end
    print(strcat('fig_vector/',opt_sim.simulationName,'_yVal_',yVal_str), '-depsc','-r300');
end