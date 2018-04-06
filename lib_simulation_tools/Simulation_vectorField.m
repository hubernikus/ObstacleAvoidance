function [metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, ds_handle, varargin)

% General simualtion Parameters
figPosition = [100 100 450 400];

%% Set up Obstacle Avoidance Functions
funcHandle = {};
funcHandle{1} = @(x,xd,obs,varargin) obs_modulation_IFD(x,xd,obs, varargin);

avoidanceType = {'convergence','initial'};

%% Parsing inputs
if isempty(varargin)
    options = check_options();
else
    options = check_options(varargin{1}); % Checking the given options, and add ones are not defined.
end

% setting initial values
if isfield(options,'obstacle') && ~isempty(options.obstacle) %there is obstacle
    obs_bool = true;
    obs = options.obstacle;
    for n=1:length(obs)
        x_obs{n} = obs{n}.x0;
        if ~isfield(obs{n},'extra')
            obs{n}.extra.ind = 2;
            obs{n}.extra.C_Amp = 0.01;
            obs{n}.extra.R_Amp = 0.0;
        else
            if ~isfield(obs{n}.extra,'ind')
                obs{n}.extra.ind = 2;
            end
            if ~isfield(obs{n}.extra,'C_Amp')
                obs{n}.extra.C_Amp = 0.01;
            end
            if ~isfield(obs{n}.extra,'R_Amp')
                obs{n}.extra.R_Amp = 0.0;
            end
        end
    end
else
    obs_bool = false;
    obs = [];
    x_obs = NaN;
end

if isfield(options, 'timeSteps')
    timeSteps = options.timeSteps;
else
    timeSteps = [0];
end

if isfield(options, 'plotSaddleTrajectory')
    plotSaddleTraj = options.plotSaddleTrajectory;
else
    plotSaddleTraj = 0;
end


if isfield(options, 'saveFig')
    saveFig = options.saveFig;
    fprintf('Going to save figs... Or NOT? \n')
else
    saveFig = false;
    fprintf('No save fig...\n')
end


if isfield(options, 'saveInitialFig')
    saveInitialFig = options.saveInitialFig;
else
    saveInitialFig = false;
end


if isfield(options, 'attractor') % if no attractor -> 'None'
    attractor = options.attractor;
else
    attractor = [0,0];
end

xd_obs  = zeros(2,length(obs)); % Linear veloctiy
w_obs = zeros(1,length(obs));   % Angular rate
if isfield(options, 'obstacle')
    for it_obs = 1:length(options.obstacle)
        if isfield(options.obstacle{it_obs}, 'perturbation')
            if isfield(options.obstacle{it_obs}.perturbation, 'dx')
                xd_obs(:,it_obs) = options.obstacle{it_obs}.perturbation.dx;
            end
            if isfield(options.obstacle{it_obs}.perturbation, 'w')
                w_obs(:,it_obs) = options.obstacle{it_obs}.perturbation.w;
            end
        end
    end
end


%%

% Mehsgrids
dim = 2;

xSeq = linspace(x_range(1),x_range(2), N_x);
ySeq = linspace(y_range(1),y_range(2), N_y);

[X, Y] = meshgrid(xSeq, ySeq);

xd_bar = zeros(dim, N_x, N_y);
xd_fluid = zeros(dim, N_x, N_y);
xd_converge = zeros(dim, N_x, N_y);
xd_IFD = zeros(dim, N_x, N_y);
xd_noColl = zeros(dim,N_x, N_y);

b_contour = 0; % Not contouring object

for it_time = 1:length(timeSteps)
    close all;
    time = timeSteps(it_time);
    
    [x_obs_boundary, x_obs_sf] = obs_draw_ellipsoid(obs,50);
    
    % Find intersection of obstacles
    [intersection_sf, x_center_dyn, intersection_obs]  = obs_common_section(obs, x_obs_sf);
    for it_obs = intersection_obs
        %fprintf('not setting center \n')
        obs{it_obs}.x_center_dyn = x_center_dyn;
    end
    
    tic 
    [collisionMatrix,X_noCollision, Y_noCollision] = obs_check_collision(obs,X,Y); 
    for ix = 1:N_x
        for iy = 1:N_y
            xd_bar(:,ix,iy) = ds_handle([X(ix,iy);Y(ix,iy)]);
            if(collisionMatrix(ix,iy))
                xd_noColl(:,ix,iy) = ds_handle([X_noCollision(ix,iy);Y_noCollision(ix,iy)]);
                xd_converge(:,ix,iy)  = [0;0];
            else
                xd_noColl(:,ix,iy) = xd_bar(:,ix,iy);
                [xd_converge(:,ix,iy),~,compTime_ellips] = obs_modulation_convergence([X(ix,iy);Y(ix,iy)],xd_bar(:,ix,iy), obs,  xd_obs,w_obs);
            end
        end
    end

    time_obstacleAvoidance = toc

    N_tot = N_x*N_y;
    relativeChangeSqr_ellip = sum(sum((squeeze(sum(xd_converge - xd_bar)).*collisionMatrix).^2))/N_tot;
    
        
    
    %patchs = {};create list? maybe..
    %contours = {};
    
    % Draw the line of the saddle point
    if plotSaddleTraj
        
    end  
    
    if saveInitialFig; N_fig = 2;
    else; N_fig = 1; end
    
    for ii = 1:N_fig
        figs{ii} = figure('Position', figPosition);
        if plotSaddleTraj
            plot([X(1,1),0],[X(1,1)/attractor(1)*attractor(2),0],'r','LineWidth',3); hold on;
            plot([X(end,end),0],[X(end,end)/attractor(1)*attractor(2),0],'Color',[255,140,0]/255,'LineWidth',3)
        end
         
         if not(strcmp(attractor,'None'))
            plot(attractor(1), attractor(2), 'kh', 'LineWidth',5); hold on;
        end
        
        set(groot,'DefaultAxesFontSize',12)
        set(groot,'DefaultLineLineWidth',0.8)
        xlabel('$\xi_1$','interpreter','latex')
        ylabel('$\xi_2$','interpreter','latex')
        set(gca,'xtick',[],'ytick',[]); box on;
        %axis off;
        xlim([X(1,1),X(end,end)]);ylim([Y(1,1),Y(end,end)]);
        axis equal;
    end
    
    
     
    tic 
    % Modified System
    figure(figs{1});
    %streamslice(X(:,:), Y(:,:), squeeze(xd_converge(1,:,:)), squeeze(xd_converge(2,:,:)), 'b')
    quiver(X(:,:), Y(:,:), squeeze(xd_converge(1,:,:)), squeeze(xd_converge(2,:,:)), 'b')
    
    %streamslice(X_noCollision(:,:), Y_noCollision(:,:), squeeze(xd_ellips(1,:,:)), squeeze(xd_ellips(2,:,:)), 'b')

    %figure(figs{2});
    %streamslice(X(:,:), Y(:,:), squeeze(xd_fluid(1,:,:)), squeeze(xd_fluid(2,:,:)), 'g'); hold on;
    %streamslice(X_noCollision(:,:), Y_noCollision(:,:), squeeze(xd_fluid(1,:,:)), squeeze(xd_fluid(2,:,:)), 'g'); hold on;
    %plot(reshape(X.*not(collisionMatrix),[],1),reshape(Y.*not(collisionMatrix),[],1),'go'); hold on;
    %plot(reshape(X_noCollision,[],1),reshape(Y_noCollision,[],1),'go'); hold on;
    %plot(reshape(X.*(collisionMatrix),[],1),reshape(Y.*(collisionMatrix),[],1),'ro')
    
    %figure(figs{3});
    %streamslice(X(:,:), Y(:,:), squeeze(xd_IFD(1,:,:)), squeeze(xd_IFD(2,:,:)), 'r'); hold on;

    if saveInitialFig
        figure(figs{2});
        streamslice(X(:,:), Y(:,:), squeeze(xd_bar(1,:,:)), squeeze(xd_bar(2,:,:)), 'k'), hold on;
    end
    
    time_createStreaslice = toc;
    
    
    for ii = 1:1
        figure(figs{ii});
        for it_obs = 1:size(x_obs_boundary,3)
            %patchs = patch(x_obs_boundary(1,:,it_obs),x_obs_boundary(2,:,it_obs),0.*ones(1,size(x_obs_boundary,2)),[0.6 1 0.6]); hold on;
            patchs = patch(x_obs_boundary(1,:,it_obs),x_obs_boundary(2,:,it_obs),[0.6 1 0.6], 'FaceAlpha',0.2); hold on;
            contours = plot(x_obs_sf(1,:,it_obs),x_obs_sf(2,:,it_obs),'k--','LineWidth',1.2); hold on;
            
            mainAxis = get(patchs, 'Parent'); % Get main axis
            
            % Define arrow parameters
            rotCol = [0.6 0.0 0];
            velCol = [0.4 0.0 0.6];
            lw = 3;
            
            if(sum(xd_obs(:,it_obs))) % create velocity arrow
                arroVel = drawArrow([obs{it_obs}.x0(1),obs{it_obs}.x0(1)+xd_obs(1,it_obs)], ...
                          [obs{it_obs}.x0(2),obs{it_obs}.x0(2)+xd_obs(2,it_obs)],...
                          {'color', velCol, 'LineWidth',lw},mainAxis);
            end
                        
            if(w_obs(it_obs)) % create agnular rate arrow
               r_angRate = max(obs{it_obs}.a)*0.7;
               arc_angRate = w_obs(it_obs)/2*pi+pi/5;
               arc_angRate = min(abs(arc_angRate), pi*3/4)*sign(arc_angRate);
               arc_angRate = max(abs(arc_angRate),pi/5)*sign(arc_angRate);
               
               samp_it = 0:5;
               x_angRate = obs{it_obs}.x0(1)+ -r_angRate*sin(-arc_angRate/2+arc_angRate/samp_it(end)*samp_it);
               y_angRate = obs{it_obs}.x0(2) + r_angRate*cos(-arc_angRate/2+arc_angRate/samp_it(end)*samp_it);
               plot(x_angRate(1:end-1), y_angRate(1:end-1), 'color',rotCol,'LineWidth',lw); hold on;
               dx = (x_angRate(end)-x_angRate(end-1));
               dy = (y_angRate(end)-y_angRate(end-1));

               arrowAngRate = drawArrow(x_angRate(end-1:end),y_angRate(end-1:end),{'color', rotCol, 'LineWidth',lw},mainAxis); hold on;
               
               uistack(patchs,'down',2) % path in behind velocity arrows
            end
            
            % --- Draw the center used by the pseudo normal --- 
            if isfield(obs{it_obs}, 'x_center_dyn')
                plot(obs{it_obs}.x_center_dyn(1),obs{it_obs}.x_center_dyn(2),'k+','LineWidth',2.3); hold on;
            else
                if ~isfield(obs{it_obs}, 'x_center')
                    obs{it_obs}.x_center = [0;0];
                end
                cosAng = cos(obs{it_obs}.th_r);
                sinAng = sin(obs{it_obs}.th_r);
                R = [cosAng, -sinAng; sinAng, cosAng];
                pos = obs{it_obs}.x0 + R*(obs{it_obs}.a.*obs{it_obs}.x_center);
                plot(pos(1),pos(2),'k+','LineWidth',2.3); hold on;
            end
        end
        
        if intersection_sf
            patch(intersection_sf(1,:), intersection_sf(2,:),[0.4, 0.2, 0.3], 'FaceAlpha',0.5)
        end
    end
    
    tic
    if(saveFig) % save figures to file
        figure(figs{1});
        print(strcat('fig_vector/',options.simulationName,'_',avoidanceType{ii},'_time',num2str(round(10*time))), '-depsc','-r300');
        
        if(saveInitialFig)
            figure(figs{2});
            print(strcat('fig_vector/',options.simulationName,'_',avoidanceType{ii},'_time',num2str(round(10*time))), '-depsc','-r300');
        end
            
    end
    time_saveFigs = toc

    if(it_time<length(timeSteps))
        for n=1:length(obs) % integration of object (linear motion!) -- first order..
            obs{n}.th_r(:) =  obs{n}.th_r + w_obs(n)*(timeSteps(it_time+1) - time);
            obs{n}.x0(:) = obs{n}.x0 + xd_obs(:,n)*(timeSteps(it_time+1) - time);
            x_obs{n}(:,end+1) = obs{n}.x0(:);
        end
    end

%   if(it_time<length(timeSteps))
%         for n=1:length(obs)  % integrating obstacle motion
%             w_obs(n) = 0;
%             xd_obs(:,n) = [0;0];
%             if isfield(obs{n},'perturbation')
%                 %if time >= round(obs{n}.perturbation.t0)+1 && time <= round(obs{n}.perturbation.tf) && length(obs{n}.perturbation.dx)==d
%                     x_obs{n}(:,end+1) = x_obs{n}(:,end) + obs{n}.perturbation.dx*(timeSteps(it_time+1)-time);
%                     obs{n}.x0 = x_obs{n}(:,end);
%                     xd_obs(:,n) = obs{n}.perturbation.dx; % all velocities ????
% 
%                     if isfield(obs{n}.perturbation,'w') % Check rotational rate
%                         w_obs(n) = obs{n}.perturbation.w;
%                     end
%                 %end
%             end
%         end
%     end
    
end

%legend('Object','Safety Margin','Original DS', 'DMM', 'IFD', 'LRS')
%simulationName = 'relativeChange_1';
dataAnalysis = false;
if(dataAnalysis)
    fileID = fopen(strcat('simulationResults/table_',options.simulationName,'.txt'),'w');
    fprintf(fileID,'Modulation Type & Relative Change of Dynamical System \\\\ \\hline \n');
    fprintf(fileID,'Fluid % 3.4f \\\\ \\hline \n', relativeChangeSqr_fluid);
    fprintf(fileID,'Rotation % 3.4f \\\\ \\hline \n', relativeChangeSqr_rot);
    fprintf(fileID,'Ellipse % 3.4f \\\\ \\hline \n', relativeChangeSqr_ellip);
    fclose(fileID);
end

metrics = 0;

end