function [] = divergence_analysis(ds_handle, fn_handle_objAvoidance, ...
                                  N_points, x_range, y_range, options)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Only implemented for 2D so far
dim = 2;

if isfield(options, 'attractor') % if no attractor -> 'None'
    attractor = options.attractor;
else
    attractor = zeros(dim,1);
end

if isfield(options, 'simulationName') % if no attractor -> 'None'
    simulationName = options.simulationName;
else
    simulationName = '_default';
end

if isfield(options, 'saveFig') % if no attractor -> 'None'
    saveFig = options.saveFig;
else
    % Don't save per default
    saveFig = 0;
end


if isfield(options, 'obstacle')
    obs = options.obstacle;
    obsExist = true;
    for it_obs = 1:size(obs,2)
        if ~isfield(obs, 'x_center')
            obs{it_obs}.x_center = [0;0];
        end
    end
    w = zeros(size(obs,2),2);
    xd = zeros(dim, size(obs,2));
else
    obsExist = false;
end

figPosition = [100 100 675 600];

% Simulation Parameters
N_x = N_points;  N_y = N_points;
x_val = linspace(x_range(1), x_range(2), N_x);
y_val = linspace(y_range(1), y_range(2), N_y);

[X,Y] = meshgrid(x_val,y_val);
U = zeros(size(X)); V = zeros(size(X));

if obsExist
    collisionMatrix = obs_check_collision(obs, X, Y);
else
    collisionMatrix = zeros(size(X));
end

for ix = 1:N_x
    for iy = 1:N_y
%         if collisionMatrix(ix,iy)
%             U(ix,iy) = 0; V(ix,iy) = 0;
%         else
            x_dot = ds_handle(X(ix,iy), Y(ix,iy) );
            if obsExist
                x_dot = fn_handle_objAvoidance([X(ix,iy);Y(ix,iy)], x_dot, obs, w, xd );
            end
            U(ix,iy) = x_dot(1); 
            V(ix,iy) = x_dot(2);
%         end
    end
end

%figure;
%streamslice(X, Y, U, V); hold on;
%quiver(X, Y, u, v); hold on;
%streamline(X, Y, u, v); hold on;
%xlim(x_range); ylim(y_range);
U = U.*not(collisionMatrix);
V = V.*not(collisionMatrix);

% Divergence
div = divergence(X,Y,U,V);
div(isnan(div)) = 0;
%div = div.*not(collisionMatrix);

%
safetyMatrix = ones(size(collisionMatrix));

for ix = 2:N_x-1
    for iy = 2:N_y-1
        if(collisionMatrix(ix,iy))
            for dx = -1:1
                for dy = -1:1
                    safetyMatrix(ix+dx,iy+dy)=0;
                end
            end
        end
    end
end
        
map_redBlue = redblue(200);

divSafety = div.*safetyMatrix;
%divSafety = div.*not(collisionMatrix);

%divRange = max(max(abs(divSafety)));
divRange = 2.1;

figure('Position', figPosition);
% Colormap
imagesc(x_range, y_range, divSafety); hold on;
colormap(map_redBlue);
colorbar;
caxis(divRange*[-1,1])

% Vectorfield
stream = streamslice(X, Y, U, V,'k'); hold on;
set( stream, 'Color', [0.4 0.4 0.4] );

if obsExist
    % Draw obstacle
    [x_obs, x_obs_sf] = obs_draw_ellipsoid(obs,50);
    for it_obs = 1:size(x_obs,3)
        patch(x_obs(1,:,it_obs),x_obs(2,:,it_obs),[0.6 1 0.6],'FaceAlpha',1)
        plot(x_obs_sf(1,:,it_obs),x_obs_sf(2,:,it_obs),'k--')
    end

    % Plot Obstacle Center
    cosAng = cos(obs{it_obs}.th_r);
    sinAng = sin(obs{it_obs}.th_r);
    R = [cosAng, -sinAng; sinAng, cosAng];
    pos = obs{it_obs}.x0 + R*(obs{it_obs}.a.*obs{it_obs}.x_center);
    plot(pos(1),pos(2),'k+','LineWidth',2.3); hold on;
end

% Plot Attractor
if not(strcmp(attractor,'None'))
    plot(attractor(1), attractor(2), 'kh', 'LineWidth',5); hold on;
end

% Plot setup
set(groot,'DefaultAxesFontSize',12)
set(groot,'DefaultLineLineWidth',0.8)
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')
set(gca,'xtick',[],'ytick',[]); box on;
axis equal;
xlim(x_range); ylim(y_range);

if saveFig
    print(strcat('fig_vector/divergenceAnalysis_',simulationName,'.eps'),'-depsc')
end
end