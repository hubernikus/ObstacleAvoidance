function Visualization_divergence
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%          Semester Project - HUBER Lukas, LASA Lab, EPFL  
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
                          obs_modulation_convergence(x,xd,obs,varargin);

% fn_handle_objAvoidance = @(x,xd,obs, varargin) ...
%                           obs_modulation_convergence_attractor(x,xd,obs,ds_handle, varargin);

N_points = 200;

%opt_sim.color_map = redblue(200);
%test = four_colors(5);
opt_sim.color_map = four_colors(100);

opt_sim.saveFig = true;
opt_sim.colorAxis = 3.1*[-1,1]-2;
%opt_sim.colorAxis = 3.1*[-1,1];
fprintf('\nInit finished \n')

%% Ellipse with good centering
close all; clc;
fprintf('Start 2D-Simulation at: %s \n', datestr(datetime('now') ) ); tic;

opt_sim.x_attractor = [0;0];

% Turn into function
%ds_handle = @(x,y) linearStableDS_const([x;y],opt_sim.x_attractor);
%ds_handle = @(x,y) linearStableDS_erf([x;y],opt_sim.x_attractor);
ds_handle = @(x,y) linearStableDS([x;y],opt_sim.x_attractor);

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

opt_sim.simulationName = 'fourColors_elongatedEllipse_diag'
opt_sim.obstacle = obs;

divergence_analysis(ds_handle, fn_handle_objAvoidance, ...
                                  N_points, x_range, y_range, ...
                                  opt_sim)

timeEnd = toc; fprintf('\nEnd 2D-Simulation in %3.4f seconds. \n', timeEnd);


% %%
% %clc; close all
% N_resol = 100;
% N_vec = floor(N_resol/10);
% 
% % Place obstacles
% obs = [];
% i=1;
% %obs{i}.a = [0.2;5.5];
% obs{i}.a = [0.2;5.5];
% obs{i}.p = [1;1];
% obs{i}.x0 = [8;0.0];
% obs{i}.sf = [1.0];
% obs{i}.th_r = 0*pi/180;
% obs{i}.x_center = [0.0;0.0];
% 
% x_range = [-2,15]; y_range = [-8.8,8.8];
% 
% x_vec = linspace(x_range(1), x_range(2),N_resol);
% y_vec = linspace(y_range(1), y_range(2),N_resol);
% 
% [X,Y] = meshgrid(x_vec, y_vec);
% divVal = zeros(size(X));
% V = zeros(size(X));
% U = zeros(size(X));
% Gamma = zeros(size(X));
% T1 = zeros(size(X));
% T2 = zeros(size(X));
% 
% for ix = 1:N_resol
%     for iy= 1:N_resol
%         [divVal(ix,iy), U(ix,iy),V(ix,iy),Gamma(ix,iy),T1(ix,iy),T2(ix,iy)] ...
%             = divergence_velocityField([X(ix,iy); Y(ix,iy)], obs);
%     end
% end
%     
% figure;
% 
% %imagesc(x_range, y_range, Gamma); hold on;
% imagesc(x_range, y_range, divVal); hold on;
% colormap(opt_sim.color_map);
% colorbar;
% caxis(2*[-1,1]-1)
% 
% plot(0,0,'k+','LineWidth',4); hold on;
% %N_U = floor(N_resol/10);
% %ind = 1:N_U:N_resol;
% %quiver(X(ind,ind),Y(ind,ind),U(ind,ind),V(ind,ind),'k')
% streamslice(X, Y, U, V, 'k')
% %quiver(X(ind,ind),Y(ind,ind),T1(ind,ind),T2(ind,ind),'b')
% 
% axis equal;
% 
% %[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs,50);
% %plot(x_obs(1,:,1),x_obs(2,:,1),'k')
% 
% xlim(x_range); ylim(y_range);


%% Ellipse with good centering
%close all; clc;
clc;
fprintf('Start 2D-Simulation at: %s \n', datestr(datetime('now') ) ); tic;

opt_sim.x_attractor = [0;0];

% Turn into function
%ds_handle = @(x,y) linearStableDS_erf([x;y],opt_sim.x_attractor);
ds_handle = @(x,y) linearStableDS([x;y],opt_sim.x_attractor);

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

opt_sim.analysis_type = 'analytic';
opt_sim.simulationName = 'fourColors_elongatedEllipse_vert'
opt_sim.obstacle = obs;

divergence_analysis(ds_handle, fn_handle_objAvoidance, ...
                                  N_points, x_range, y_range, ...
                                  opt_sim)

timeEnd = toc; fprintf('\nEnd 2D-Simulation in %3.4f seconds. \n', timeEnd);

%% Ellipse with good centering
%close all; clc;
clc;
fprintf('Start 2D-Simulation at: %s \n', datestr(datetime('now') ) ); tic;

opt_sim.x_attractor = [0;0];

% Turn into function
ds_handle = @(x,y) linearStableDS([x;y],opt_sim.x_attractor);
%ds_handle = @(x,y) linearStableDS_erf([x;y],opt_sim.x_attractor);

% Place obstacles
obs = [];
i=1;
obs{i}.a = [5;5];
obs{i}.p = [1;1];
obs{i}.x0 = [8;0.0];
obs{i}.sf = [1.0];
obs{i}.th_r = 0*pi/180;
obs{i}.x_center = [0.0;0.0];

x_range = 1.6*[-2,15]; y_range = 1.6*[-8.8,8.8];

%opt_sim.analysis_type = 'numeric';
opt_sim.analysis_type = 'analytic';
opt_sim.simulationName = 'fourColors_circle'
opt_sim.obstacle = obs;


divergence_analysis(ds_handle, fn_handle_objAvoidance, ...
                                  N_points, x_range, y_range, ...
                                  opt_sim)

timeEnd = toc; fprintf('\nEnd 2D-Simulation in %3.4f seconds. \n', timeEnd);

%% Ellipse with good centering
%close all; clc;
clc;
fprintf('Start 2D-Simulation at: %s \n', datestr(datetime('now') ) ); tic;

opt_sim.x_attractor = [0;0];

% Turn into function
%ds_handle = @(x,y) linearStableDS_erf([x;y],opt_sim.x_attractor);
ds_handle = @(x,y) linearStableDS([x;y],opt_sim.x_attractor);

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
opt_sim.saveFig = true;
opt_sim.simulationName = 'fourColors_elongatedEllipse'

%opt_sim.analysis_type = 'numeric';
opt_sim.analysis_type = 'analytic';

opt_sim.obstacle = obs;

divergence_analysis(ds_handle, fn_handle_objAvoidance, ...
                                  N_points, x_range, y_range, ...
                                  opt_sim)

timeEnd = toc; fprintf('\nEnd 2D-Simulation in %3.4f seconds. \n', timeEnd);



%%
close all
for ii=1:7 %1:3
    
    figure;
    switch ii
        case 1
            imagesc(xValues, yValues, denominator); hold on; % Create plot
            caxis(2*[-1,1]) % set axis range
        case 2 
            imagesc(xValues, yValues, determinantNominator); hold on; % Create plot
            caxis(2*[-1,1]) % set axis range
        case 3
            imagesc(xValues, yValues, traceNominator); hold on; % Create plot
            caxis(2*[-1,1]) % set axis range
        case 4
            imagesc(xValues, yValues, trace); hold on; % Create plot
            caxis(2*[-1,1]) % set axis range
        case 5 
            imagesc(xValues, yValues, determinant); hold on; % Create plot
            caxis(2*[-1,1]) % set axis range
        case 6
            imagesc(xValues, yValues, squeeze(eigValues(1,:,:)) ); hold on; % Create plot
            caxis(2*[-1,1]) % set axis range
        case 7
            imagesc(xValues, yValues, squeeze( eigValues(2,:,:)) ); hold on; % Create plot
            caxis(2*[-1,1]) % set axis range
    end
    map_4cols = four_colors(N, green, red); % Create desired coloring
    colormap(map_4cols);
    colorbar; % show bar


    [x_obs, x_obs_sf] = obs_draw_ellipsoid(obs,50);
    for it_obs = 1:size(x_obs,3)
        patch(x_obs(1,:,it_obs),x_obs(2,:,it_obs),[0.2 0.2 0.2],'FaceAlpha',0.8); hold on;
        plot(x_obs_sf(1,:,it_obs),x_obs_sf(2,:,it_obs),'k-','LineWidth',3)
    end
    axis equal;
    grid on;
    xlim(x_range); ylim(y_range);

%     if length(zerosTrace)
%         plot(zerosTrace(1,:), zerosTrace(2,:), '*', 'Color',[0.7, 0,0]) % Red
%     end
%     if length(zerosDet)
%         plot(zerosDet(1,:), zerosDet(2,:),'o', 'Color',[0, 0.7,0]) % Green
%     end
%     if length(poles)
%         plot(poles(1,:), poles(2,:), 'x','Color',[0, 0, 0.7]) % Blue
%     end
end
t_numericalZero = toc;

%fprintf('Numerical evaluation took %d seconds. \n', t_numericalZero)

%% Ellipse with good centering
if false % Only for explanatory purpose, does not ned to be evaluated with others
    close all; clc;
    fprintf('Start 2D-Simulation at: %s \n', datestr(datetime('now') ) ); tic;

    N_points = 200;

    opt_sim.color_map = redblue(100);

    opt_sim.x_attractor = [0;0];
    opt_sim.colorAxis = 3.1*[-1,1]-2;

    % Turn into function
    %ds_handle = @(x,y) linearStableDS_erf([x;y],opt_sim.x_attractor);
    ds_handle = @(x,y) linearStableDS([x;y],opt_sim.x_attractor);
    %ds_handle = @(x,y) linearStableDS_const([x;y],opt_sim.x_attractor);

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

    opt_sim.simulationName = 'noColors_normal_ellipse'
    opt_sim.saveFig = false;

    opt_sim.analysis_type = 'analytic';

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

    dPhi = 10/180*pi;

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
end