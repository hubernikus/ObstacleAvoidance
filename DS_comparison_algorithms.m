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
opt_sim.dt = 0.03; %integration time steps
opt_sim.i_max = 100; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined

funcHandle = {};
funcHandle{1} = @(x,xd,obs,b_contour,varargin) obs_modulation_ellipsoid(x,xd,obs,b_contour, varargin);
funcHandle{2} = @(x,xd,obs,b_contour,varargin) obs_modulation_fluidMechanics(x,xd,obs,b_contour, varargin);
funcHandle{3} = @(x,xd,obs,b_contour,varargin) obs_modulation_rotation(x,xd,obs,b_contour, varargin);
funcHandle{4} = @(x,xd,obs,b_contour,varargin) obs_modulation_ellipsoid(x,xd,obs,b_contour, varargin);
avoidanceType = {'ellipsoid','fluid','rotation','none'};



%% Mehsgrids
close all; % clear variables
clc; 

clear obs;
obs1{1}.a = [1;4];
obs1{1}.p = [1;1];
obs1{1}.x0 = [-2;3];
obs1{1}.sf = [1.2;1.2];
obs1{1}.th_r = -30*pi/180;
n= 1;
%obs1{n}.extra.ind = 2;
%obs1{n}.extra.C_Amp = 0.01;
%obs1{n}.extra.R_Amp = 0.0;

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
        coll = obs_check_collision(obs1,[X(ix,iy);Y(ix,iy)]);
        if(coll)
            xd_hat(:,ix,iy)=0;
            xd_fluid(:,ix,iy)=0;
            xd_ellips(:,ix,iy)=0;
            xd_rot(:,ix,iy)=0;
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
set(groot,'DefaultAxesFontSize',14)
set(groot,'DefaultLineLineWidth',1.0)
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
legend('Object','Safety Margin','Original DS', 'DMM', 'IFD', 'LRS')
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


%%  Rotating objects
fprintf('Start 2D-Simulation \n');

% Set default simulation parameters
opt_sim.dt = 0.03; %integration time steps
opt_sim.i_max = 120; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined


fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

% Place obstacles
obs = [];

% obstacle 1
i=1;
obs{i}.a = [1;3];
obs{i}.p = [1;1];
obs{i}.x0 = [4;8];
obs{i}.sf = [1.2];
obs{i}.th_r = 0*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 4;
obs{i}.perturbation.dx = [4;-1];  
obs{i}.perturbation.w = 1;  
i=2;
obs{i}.a = [2;3];
obs{i}.p = [1;1];
obs{i}.x0 = [8;-5];
obs{i}.sf = [1.2];
obs{i}.th_r = 20*pi/180;
obs{i}.perturbation.t0 = 0;
obs{i}.perturbation.tf = 6;
obs{i}.perturbation.dx = [-1;2];  
obs{i}.perturbation.w = -2;  

opt_sim.obstacle = obs;

nbSPoints = size(x0,2);

simulationName = 'twoMovingRotatingObs';

fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
fprintf(fileID,'Object Avoidance Type & Relative Change [m2/s2] & Number of Trajectories with Penetration [%%] & Penetration Steps [] & Convergence Time [s] & Convergence Distance [m] & Normalized Energy [J/kg] & Computational Time [ms] \\\\ \\hline \n');

for ii = 1:3
    opt_sim.obstacleAvoidanceFunction = funcHandle{ii};
    fig(ii) = figure('name',strcat(simulationName,'_',avoidanceType{ii}),'position',[200 100 700 700]);
    opt_sim.figure = fig(ii);
    
    [~,~,time,~,~,metrics] = Simulation(x0,[],fn_handle,opt_sim); 
    
    fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);
end
%fclose(fileID);
%
%fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
ii = 4;
opt_sim.obstacleAvoidanceFunction = funcHandle{4};
opt_simInit = opt_sim;
opt_simInit.obstacle = [];
fig(ii) = figure('name',strcat(simulationName,'_none'),'position',[200 100 700 700]);
opt_simInit.figure = fig(ii);
[~,~,~,~,~,metrics] = Simulation(x0,[],fn_handle,opt_simInit); 
fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);

fclose(fileID);

%%  Rotating objects
fprintf('Start 2D-Simulation \n');

% Set default simulation parameters
opt_sim.dt = 0.025; %integration time steps
opt_sim.i_max = 400; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined


fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

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

nbSPoints = size(x0,2);

simulationName = 'oneMovingRotatingObs';

fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
fprintf(fileID,'Object Avoidance Type & Relative Change [m2/s2] & Number of Trajectories with Penetration [%%] & Penetration Steps [] & Convergence Time [s] & Convergence Distance [m] & Normalized Energy [J/kg] & Computational Time [ms] \\\\ \\hline \n');

for ii = 1:3
    opt_sim.obstacleAvoidanceFunction = funcHandle{ii};
    fig(ii) = figure('name',strcat(simulationName,'_',avoidanceType{ii}),'position',[200 100 700 700]);
    opt_sim.figure = fig(ii);
    
    [~,~,time,~,~,metrics] = Simulation(x0,[],fn_handle,opt_sim); 
    
    fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);
end
%fclose(fileID);
%
%fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
ii = 4;
opt_sim.obstacleAvoidanceFunction = funcHandle{4};
opt_simInit = opt_sim;
opt_simInit.obstacle = [];
fig(ii) = figure('name',strcat(simulationName,'_none'),'position',[200 100 700 700]);
opt_simInit.figure = fig(ii);
[~,~,~,~,~,metrics] = Simulation(x0,[],fn_handle,opt_simInit); 
fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);

fclose(fileID);


%%  Constant vel to origin 
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

% Place obstacles
obs = [];

opt_sim.dt = 0.025; %integration time steps
opt_sim.i_max = 400; %maximum number of iterations
opt_sim.tol = 0.00001; %convergence tolerance

% obstacle 1
i=1;

obs{i}.a = [2;4];
obs{i}.p = [1;1];
obs{i}.x0 = [10;4];
obs{i}.sf = [1.2];
obs{i}.th_r = 60*pi/180;
% obs{i}.perturbation.t0 = 4;
% obs{i}.perturbation.tf = 10;
% obs{i}.perturbation.dx = [-4;0];  
%obs{i}.perturbation.w = 1;  

opt_sim.obstacle = obs;

nbSPoints = size(x0,2);

simulationName = 'staticEllipse';

fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
fprintf(fileID,'Object Avoidance Type & Relative Change [m2/s2] & Number of Trajectories with Penetration [%%] & Penetration Steps [] & Convergence Time [s] & Convergence Distance [m] & Normalized Energy [J/kg] & Computational Time [ms] \\\\ \\hline \n');

for ii = 1:3
    opt_sim.obstacleAvoidanceFunction = funcHandle{ii};
    fig(ii) = figure('name',strcat(simulationName,'_',avoidanceType{ii}),'position',[200 100 700 700]);
    opt_sim.figure = fig(ii);
    
    [~,~,time,~,~,metrics] = Simulation(x0,[],fn_handle,opt_sim); 
    
    fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);
end
%fclose(fileID);
%
%fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
ii = 4;
opt_sim.obstacleAvoidanceFunction = funcHandle{4};
opt_simInit = opt_sim;
opt_simInit.obstacle = [];
fig(ii) = figure('name',strcat(simulationName,'_none'),'position',[200 100 700 700]);
opt_simInit.figure = fig(ii);
[~,~,~,~,~,metrics] = Simulation(x0,[],fn_handle,opt_simInit); 
fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);

fclose(fileID);

fprintf('End 2D-Simulation \n');


%%  Constant vel to origin 
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N = 20;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

% Place obstacles
obs = [];

opt_sim.dt = 0.025; %integration time steps
opt_sim.i_max = 400; %maximum number of iterations
opt_sim.tol = 0.00001; %convergence tolerance

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

nbSPoints = size(x0,2);

simulationName = 'leftMoving_circle';

fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
fprintf(fileID,'Object Avoidance Type & Relative Change [m2/s2] & Number of Trajectories with Penetration [%%] & Penetration Steps [] & Convergence Time [s] & Convergence Distance [m] & Normalized Energy [J/kg] & Computational Time [ms] \\\\ \\hline \n');

for ii = 1:3
    opt_sim.obstacleAvoidanceFunction = funcHandle{ii};
    fig(ii) = figure('name',strcat(simulationName,'_',avoidanceType{ii}),'position',[200 100 700 700]);
    opt_sim.figure = fig(ii);
    
    [~,~,time,~,~,metrics] = Simulation(x0,[],fn_handle,opt_sim); 
    
    fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);
end
%fclose(fileID);
%
%fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
ii = 4;
opt_sim.obstacleAvoidanceFunction = funcHandle{4};
opt_simInit = opt_sim;
opt_simInit.obstacle = [];
fig(ii) = figure('name',strcat(simulationName,'_none'),'position',[200 100 700 700]);
opt_simInit.figure = fig(ii);
[~,~,~,~,~,metrics] = Simulation(x0,[],fn_handle,opt_simInit); 
fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);

fclose(fileID);

fprintf('End 2D-Simulation \n');

%%  Rotating object close to origin 
fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N = 15;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);

opt_sim.dt = 0.025; %integration time steps
opt_sim.i_max = 400; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance


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

simulationName = 'rotating_circle';

fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
fprintf(fileID,'Object Avoidance Type & Relative Change [m2/s2] & Number of Trajectories with Penetration [%%] & Penetration Steps [] & Convergence Time [s] & Convergence Distance [m] & Normalized Energy [J/kg] & Computational Time [ms] \\\\ \\hline \n');

for ii = 1:3
    opt_sim.obstacleAvoidanceFunction = funcHandle{ii};
    fig(ii) = figure('name',strcat(simulationName,'_',avoidanceType{ii}),'position',[200 100 700 700]);
    opt_sim.figure = fig(ii);
    
    [~,~,time,~,~,metrics] = Simulation(x0,[],fn_handle,opt_sim); 
    
    fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);
end
%fclose(fileID);
%
%fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
ii = 4;
opt_sim.obstacleAvoidanceFunction = funcHandle{4};
opt_simInit = opt_sim;
opt_simInit.obstacle = [];
fig(ii) = figure('name',strcat(simulationName,'_none'),'position',[200 100 700 700]);
opt_simInit.figure = fig(ii);
[~,~,~,~,~,metrics] = Simulation(x0,[],fn_handle,opt_simInit); 
fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);

fclose(fileID);

fprintf('End 2D-Simulation \n');

%%  Rotating object close to origin 
fprintf('Start 2D-Simulation \n');

opt_sim.dt = 0.01; %integration time steps
opt_sim.i_max = 400; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance

fn_handle = @(x) linearStableDS(x);

N = 15;
x0 = [ones(1,N)*20 ; linspace(-10,20,N)];
%x0 = x0(:,5);


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

simulationName = 'fastMovingEllipse';

fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
fprintf(fileID,'Object Avoidance Type & Relative Change [m2/s2] & Number of Trajectories with Penetration [%%] & Penetration Steps [] & Convergence Time [s] & Convergence Distance [m] & Normalized Energy [J/kg] & Computational Time [ms] \\\\ \\hline \n');

for ii = 1:3
    opt_sim.obstacleAvoidanceFunction = funcHandle{ii};
    fig(ii) = figure('name',strcat(simulationName,'_',avoidanceType{ii}),'position',[200 100 700 700]);
    opt_sim.figure = fig(ii);
    
    [~,~,time,~,~,metrics] = Simulation(x0,[],fn_handle,opt_sim); 
    
    fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);
end
%fclose(fileID);
%
%fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
ii = 4;
opt_sim.obstacleAvoidanceFunction = funcHandle{4};
opt_simInit = opt_sim;
opt_simInit.obstacle = [];
fig(ii) = figure('name',strcat(simulationName,'_none'),'position',[200 100 700 700]);
opt_simInit.figure = fig(ii);
[~,~,~,~,~,metrics] = Simulation(x0,[],fn_handle,opt_simInit); 
fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);

fclose(fileID);

fprintf('End 2D-Simulation \n');

%%  4 obstacles
fprintf('Start 2D-Simulation \n');


opt_sim.dt = 0.01; %integration time steps
opt_sim.i_max = 400; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance


fn_handle = @(x) linearStableDS(x);

N = 30;
R = 10;
phi = (1:N)/N*2*pi
x0 = [cos(phi)*R ; sin(phi)*R];
%x0 = x0(:,5);

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

opt_sim.obstacle = obs;


simulationName = 'fourStaticObjects';

fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
fprintf(fileID,'Object Avoidance Type & Relative Change [m2/s2] & Number of Trajectories with Penetration [%%] & Penetration Steps [] & Convergence Time [s] & Convergence Distance [m] & Normalized Energy [J/kg] & Computational Time [ms] \\\\ \\hline \n');

for ii = 1:3
    opt_sim.obstacleAvoidanceFunction = funcHandle{ii};
    fig(ii) = figure('name',strcat(simulationName,'_',avoidanceType{ii}),'position',[200 100 700 700]);
    opt_sim.figure = fig(ii);
    
    [~,~,time,~,~,metrics] = Simulation(x0,[],fn_handle,opt_sim); 
    
    fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);
end
%fclose(fileID);
%
%fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
ii = 4;
opt_sim.obstacleAvoidanceFunction = funcHandle{4};
opt_simInit = opt_sim;
opt_simInit.obstacle = [];
fig(ii) = figure('name',strcat(simulationName,'_none'),'position',[200 100 700 700]);
opt_simInit.figure = fig(ii);
[~,~,~,~,~,metrics] = Simulation(x0,[],fn_handle,opt_simInit); 
fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);

fclose(fileID);

fprintf('End 2D-Simulation \n');



%%  2 convex to concave
fprintf('Start 2D-Simulation \n');

opt_sim.dt = 0.01; %integration time steps
opt_sim.i_max = 400; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance

fn_handle = @(x) ellipseLimit_cycle(x);

N = 10;
R = 2;
phi = (1:N)/N*2*pi
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


simulationName = 'limitCycle';

fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
fprintf(fileID,'Object Avoidance Type & Relative Change [m2/s2] & Number of Trajectories with Penetration [%%] & Penetration Steps [] & Convergence Time [s] & Convergence Distance [m] & Normalized Energy [J/kg] & Computational Time [ms] \\\\ \\hline \n');

for ii = 1:3
    opt_sim.obstacleAvoidanceFunction = funcHandle{ii};
    fig(ii) = figure('name',strcat(simulationName,'_',avoidanceType{ii}),'position',[200 100 700 700]);
    opt_sim.figure = fig(ii);
    
    [~,~,time,~,~,metrics] = Simulation(x0,[],fn_handle,opt_sim); 
    
    fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);
end
%fclose(fileID);
%
%fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
ii = 4;
opt_sim.obstacleAvoidanceFunction = funcHandle{4};
opt_simInit = opt_sim;
opt_simInit.obstacle = [];
fig(ii) = figure('name',strcat(simulationName,'_none'),'position',[200 100 700 700]);
opt_simInit.figure = fig(ii);
[~,~,~,~,~,metrics] = Simulation(x0,[],fn_handle,opt_simInit); 
fprintf(fileID,'%s & %3.4f & %2.1f %%  & %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);

fclose(fileID);

fprintf('End 2D-Simulation \n');

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

[X(:,:),Y(:,:)] = meshgrid(linspace(-10,10,N_x), linspace(-6,15,N_y));

xd_hat = zeros(dim, N_x, N_y);
xd_fluid = zeros(dim, N_x, N_y);
xd_ellips = zeros(dim, N_x, N_y);

xd_obs = [0;0]; % assumed to not moving
w_obs = [0]; % assumed to not moving

for ix = 1:N_x
    for iy = 1:N_y
        xd_hat(:,ix,iy) = stableDS([X(ix,iy);Y(ix,iy)]);
        b_contour = 0;
        xd_fluid(:,ix,iy) = obs_modulation_fluidMechanics([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs1, b_contour, xd_obs,w_obs);
        xd_ellips(:,ix,iy) = obs_modulation_ellipsoid([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs1, b_contour, xd_obs,w_obs);
    end
end
[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs1,50);

for ix = 1:size(X,1)
    for iy = 1:size(X,2)
        coll = obs_check_collision(obs1,[X(ix,iy);Y(ix,iy)]);
        if(coll)
            xd_hat(:,ix,iy)=0;
            xd_fluid(:,ix,iy)=0;
            xd_ellips(:,ix,iy)=0;
        end
        
    end
end




figure('Position',[0 0 1000 1000]);
% Draw object
patch(x_obs(1,:),x_obs(2,:),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6]); hold on;
plot(x_obs_sf(1,:),x_obs_sf(2,:),'k--','linewidth',0.5);

% Initial system
quiver(X(:,:), Y(:,:), squeeze(xd_hat(1,:,:)), squeeze(xd_hat(2,:,:)), 'k'), hold on;
% Modulated system
%figure
quiver(X(:,:), Y(:,:), squeeze(xd_ellips(1,:,:)), squeeze(xd_ellips(2,:,:)), 'b')
quiver(X(:,:), Y(:,:), squeeze(xd_fluid(1,:,:)), squeeze(xd_fluid(2,:,:)), 'g')
xlim([X(1,1),X(end,end)]);ylim([Y(1,1),Y(end,end)])
%legend('','','Original System', 'Ellipse Modulation', 'Ellipse Modulation Adaptded')
%drawArrow([-4,-4],[-5,-8],'linewidth',3,'color','r')
axis equal;
%print('fig/quiverPlot_DS_LS_circle_fluidMechanics','-depsc')

%% Demo: concave
%close all; clc;

fprintf('Start 2D-Simulation \n');

fn_handle = @(x) linearStableDS(x);

N =10;
x0 = [ones(1,N)*10 ; linspace(-10,10,N)];
x0 = x0(:,5)

% Place obstacles
obs = [];

% obstacle 1
obs{1}.a = [1;4];
obs{1}.p = [1;1];
obs{1}.x0 = [4;0.5];
obs{1}.sf = [1.2;1.2];
obs{1}.th_r = 0*pi/180;
obs{1}.concaveAngle = pi*0.3;

% obs{1}.perturbation.t0 = 0;
% obs{1}.perturbation.tf = 5;
% obs{1}.perturbation.dx = [3;0];  
% Start simulation
opt_sim.obstacle = obs;


fig(1) = figure('name','fluidDynamics_model_movingObj_posx','position',[100 100 800 600]);


opt_sim.figure = fig(1);
Simulation(x0,[],fn_handle,opt_sim); % NOT good IC
axis equal;
%xlim([-15 15]); ylim([-15 15])
%close al
plot(obs{1}.x0(1),obs{1}.x0(2), 'ro')
fprintf('End 2D-Simulation \n');


fprintf('End 2D-Simulation \n');

