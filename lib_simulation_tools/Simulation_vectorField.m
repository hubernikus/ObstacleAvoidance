function [metrics] = Simulation_vectorField(x_range, y_range, N_x, N_y, fn_handle, varargin)

% General simualtion Parameters
saveSingleFigs = true;
figPosition = [100 100 450 400];

%% Set up Obstacle Avoidance Functions
funcHandle = {};
funcHandle{1} = @(x,xd,obs,b_contour,varargin) obs_modulation_ellipsoid(x,xd,obs,b_contour, varargin);
funcHandle{2} = @(x,xd,obs,b_contour,varargin) obs_modulation_fluidMechanics(x,xd,obs,b_contour, varargin);
funcHandle{3} = @(x,xd,obs,b_contour,varargin) obs_modulation_rotation(x,xd,obs,b_contour, varargin);

avoidanceType = {'DMM','IFD','LRS','_'};

%% parsing inputs
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


xd_obs  = zeros(2,length(obs)); % Linear veloctiy
w_obs = zeros(1,length(obs));   % Angular rate

% Mehsgrids
dim = 2;

xSeq = linspace(x_range(1),x_range(2), N_x);
ySeq = linspace(y_range(1),y_range(2), N_y);

[X, Y] = meshgrid(xSeq, ySeq);

xd_hat = zeros(dim, N_x, N_y);
xd_fluid = zeros(dim, N_x, N_y);
xd_ellips = zeros(dim, N_x, N_y);
xd_rot = zeros(dim, N_x, N_y);


b_contour = 0; % Not contouring object

for it_time = 1:length(timeSteps)
    time = timeSteps(it_time)
    % applying perturbation on the obstacles
    for n=1:length(obs) 
        w_obs(n) = 0;
        xd_obs(:,n) = [0;0];
        if isfield(obs{n},'perturbation')
            %if time >= round(obs{n}.perturbation.t0)+1 && time <= round(obs{n}.perturbation.tf) && length(obs{n}.perturbation.dx)==d
                x_obs{n}(:,end+1) = x_obs{n}(:,end) + obs{n}.perturbation.dx*(time-timeSteps(max(1,it_time-1)));
                obs{n}.x0 = x_obs{n}(:,end);
                xd_obs(:,n) = obs{n}.perturbation.dx; % all velocities ????

                if isfield(obs{n}.perturbation,'w') % Check rotational rate
                    w_obs(n) = obs{n}.perturbation.w;
                end
            %end
        end
    end
    
    for n=1:length(obs) % integration of object (linear motion!) -- first order..
      obs{n}.th_r(:) =  obs{n}.th_r + w_obs(n)*(time-timeSteps(max(1,it_time-1)));
      x_obs{n}(:,end+1) = x_obs{n}(:,end);
    end
    
    collisionMatrix = obs_check_collision(obs,X,Y); 
    for ix = 1:N_x
        for iy = 1:N_y
            xd_hat(:,ix,iy)= fn_handle([X(ix,iy);Y(ix,iy)]);
            if(collisionMatrix(ix,iy))
                xd_fluid(:,ix,iy)=0;
                xd_ellips(:,ix,iy)=0;
                xd_rot(:,ix,iy)=0;
            else
                [xd_fluid(:,ix,iy),~,~,compTime_fluid] = obs_modulation_fluidMechanics([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs, b_contour, xd_obs,w_obs);
                [xd_ellips(:,ix,iy),~,~,compTime_ellips] = obs_modulation_ellipsoid([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs, b_contour, xd_obs,w_obs);
                [xd_rot(:,ix,iy),~,~,compTime_rot] = obs_modulation_rotation([X(ix,iy);Y(ix,iy)],xd_hat(:,ix,iy), obs, b_contour, xd_obs,w_obs);
            end
        end
    end

    N_tot = N_x*N_y;
    relativeChangeSqr_fluid = sum(sum((squeeze(sum(xd_fluid - xd_hat)).*collisionMatrix).^2))/N_tot;
    relativeChangeSqr_ellip = sum(sum((squeeze(sum(xd_ellips - xd_hat)).*collisionMatrix).^2))/N_tot;
    relativeChangeSqr_rot = sum(sum((squeeze(sum(xd_rot - xd_hat)).*collisionMatrix).^2))/N_tot;

    [x_obs, x_obs_sf] = obs_draw_ellipsoid(obs,50);

    figs = {}
    for ii = 1:3
        figs{ii} = figure('Position', figPosition);
        for it_obs = 1:size(x_obs,3)
            patch(x_obs(1,:,it_obs),x_obs(2,:,it_obs),0.1*ones(1,size(x_obs,2)),[0.6 1 0.6]); hold on;
            plot(x_obs_sf(1,:,it_obs),x_obs_sf(2,:,it_obs),'k--','linewidth',0.5);
            
            if(xd_obs(it_obs))
                quiver(obs(it_obs).x0(1),obs(it_obs).x0(2), xd_obs(it_obs)(1), xd_obs(it_obs)(2), 'o'); 
            end
            
            if(w_obs(it_obs))
               r_angRat = max(obs{it_obs}.a)*0.7;
               arc_angRate = w_obs(it_obs);
               arc_angRate = min(arc_angRate, pi);
               arc_angRate = max(arc_angRate, pi/8);
               
               samp_it = 0:10;
               x_angRate = r_angRate*sin(-arc_angRate/2+arc_angRate/samp_it(end)*samp_it);
               y_angRate = r_angRate*cos(-arc_angRate/2+arc_angRate/samp_it(end)*samp_it);
               plot(x_angRate, y_angRate, 'b')
               dx = (x_angRate(end)-x_angRate(end-1))*0.1;
               dy = (y_angRate(end)-y_angRate(end-1))*0.1;
               quiver(x_angRate(end),y_angRate(end),dx,dy,'b')
            end
        end
        
    end

    ii = 4;
    figs{ii} = figure('Position',figPosition);

    % Modified System
    figure(figs{1});
    streamslice(X(:,:), Y(:,:), squeeze(xd_ellips(1,:,:)), squeeze(xd_ellips(2,:,:)), 'b')

    figure(figs{2});
    streamslice(X(:,:), Y(:,:), squeeze(xd_fluid(1,:,:)), squeeze(xd_fluid(2,:,:)), 'g')

    figure(figs{3});
    streamslice(X(:,:), Y(:,:), squeeze(xd_rot(1,:,:)), squeeze(xd_rot(2,:,:)), 'r'); hold on;

    figure(figs{4});
    streamslice(X(:,:), Y(:,:), squeeze(xd_hat(1,:,:)), squeeze(xd_hat(2,:,:)), 'k'), hold on;

    for ii = 1:4
        figure(figs{ii});
        set(groot,'DefaultAxesFontSize',12)
        set(groot,'DefaultLineLineWidth',0.8)
        xlabel('$\xi_1$','interpreter','latex')
        ylabel('$\xi_2$','interpreter','latex')
        set(gca,'xtick',[],'ytick',[])
        %axis off;
        xlim([X(1,1),X(end,end)]);ylim([Y(1,1),Y(end,end)]);
        axis equal;
        if(saveSingleFigs)
            print(strcat('fig_vector/',options.simulationName,'_',avoidanceType{ii},'_time',num2str(time)), '-depsc','-r300');
        end
    end
end

%legend('Object','Safety Margin','Original DS', 'DMM', 'IFD', 'LRS')
simulationName = 'relativeChange_1';
dataAnalysis = false;
if(dataAnalysis)
    fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
    fprintf(fileID,'Modulation Type & Relative Change of Dynamical System \\\\ \\hline \n');
    fprintf(fileID,'Fluid % 3.4f \\\\ \\hline \n', relativeChangeSqr_fluid);
    fprintf(fileID,'Rotation % 3.4f \\\\ \\hline \n', relativeChangeSqr_rot);
    fprintf(fileID,'Ellipse % 3.4f \\\\ \\hline \n', relativeChangeSqr_ellip);
    fclose(fileID);
end

metrics = 0;



end

