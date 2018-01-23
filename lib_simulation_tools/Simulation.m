function [x, xd, t, xT, x_obs, metrics] = Simulation(x0,xT,fn_handle, varargin)
%
% This function simulates motion that were learnt using SEDS, which defines
% a motins as a nonlinear time-independent asymptotically stable dynamical
% systems:
%                               xd=f(x)
%
% wherbitrary d dimensional variable, and xd is its first time
% derivative.re x is an a
%
% The function can be called using:
%       [x xd t]=Simulation(x0,xT,Priors,Mu,Sigma)
%
% or
%       [x xd t]=Simulation(x0,xT,Priors,Mu,Sigma,options)
%
% to also send a structure of desired options.
%
% Inputs -----------------------------------------------------------------
%   o x:       d x N matrix vector representing N different starting point(s)
%   o xT:      d x 1 Column vector representing the target point
%   o fn_handle:  A handle function that only gets as input a d x N matrix,
%                 and returns the output matrix of the same dimension. Note
%                 that the output variable is the first time derivative of
%                 the input variable.
%
%   o options: A structure to set the optional parameters of the simulator.
%              The following parameters can be set in the options:
%       - .dt:      integration time step [default: 0.02]
%       - .i_max:   maximum number of iteration for simulator [default: i_max=1000]
%       - .plot     setting simulation graphic on (true) or off (false) [default: true]
%       - .tol:     A positive scalar defining the threshold to stop the
%                   simulator. If the motions velocity becomes less than
%                   tol, then simulation stop5 [default: 0.001]
%       - .perturbation: a structure to apply pertorbations to the robot.
%                        This variable has the following subvariables:
%       - .perturbation.type: A string defining the type of perturbations.
%                             The acceptable values are:
%                             'tdp' : target discrete perturbation
%                             'tcp' : target continuous perturbation
%                             'rdp' : robot discrete perturbation
%                             'rcp' : robot continuous perturbation
%       - .perturbation.t0:   A positive scalar defining the time when the
%                             perturbation should be applied.
%       - .perturbation.tf:   A positive scalar defining the final time for
%                             the perturbations. This variable is necessary
%                      70       only when the type is set to 'tcp' or 'rcp'.
%       - .perturbation.dx:   A d x 1 vector defining the perturbation's
%                             magnitude. In 'tdp' and 'rdp', it simply
%                             means a relative displacement of the
%                             target/robot with the vector dx. In 'tcp' and
%                             'rcp', the target/robot starts moving with
%                             the velocity dx.
%       - .obstacleAvoidanceFunction:
%                    @(x,xd,obs,b_contour,varargin) obs_modulation_fluidMechanics(x,xd,obs,b_contour, varargin);
%                    @(x,xd,obs,b_contour,varargin) obs_modulation_ellipsoid(x,xd,obs,b_contour, varargin);
%  % problems in implementation@(x,xd,obs,b_contour,varargin) obs_modulation_ellipsoid_adapted(x,xd,obs,b_contour,varargin)
%                    @(x,xd,obs,b_contour,varargin) obs_modulation_ellipsoid_adapted(x,xd,obs,b_contour,varargin)
% Outputs ----------------------------------------------------------------
%   o x:       d x T x N matrix containing the position of N, d dimensional
%              trajectories with the length T.
%
%   o xd:      d x T x N matrix containing the velocity of N, d dimensional
%              trajectories with the length T.
%
%   o t:       1 x N vector containing the trajectories' time.
%
%   o xT:      A matrix recording the change in the target position. Useful
%              only when 'tcp' or 'tdp' perturbations applied.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2010 S. Mohammad Khansari-Zadeh, LASA Lab, EPFL,   %%%
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% The program is free for non-commercial academic use. Please contact the
% author if you are interested in using the software for commercial purposes.
% The software must not be modified or distributed without prior permission
% of the authors. Please acknowledge the authors in any academic publications
% that have made use of this code or part of it. Please use this BibTex
% reference:
% 
%   S. M. Khansari Zadeh and A. Billard, "Learning Stable Non-Linear 
%   Dynamical Systems with Gaussian Mixture Models", IEEE Transaction on
%   Robotics, vol. 27, num 5, p. 943-957.
%
% To get latest upadate of the software please visit
%                          http://lasa.epfl.ch/khansari
%
% Please send your feedbacks or questions to:
%                           mohammad.khansari_at_epfl.ch

% Additional variable - introduce in funciton header


%% Set up metrics
nbSPoints=size(x0,2); %number of starting points. This enables to simulatneously run several trajectories


movieSave = false;
figSave = true;

maxVal = false;

dataAnalysis = true;
simulationName = get(gcf,'Name');

N_pigSave = 5; % Number of pictures being saved
picNum = 0; % number of picuture already saved
    
if movieSave
    % Figure Name
    filename = sprintf(strcat('animations/',simulationName,'.gif'));
    
    % Initialization of avi video object
    nFrames = 20;
    vidObj = VideoWriter(strcat('animations/', simulationName, '.avi'));
    vidObj.Quality = 100;
    vidObj.FrameRate = 20;
    open(vidObj);
end

metrics.relativeChangeSqr = zeros(1,nbSPoints);
metrics.compTime = zeros(1,nbSPoints);
metrics.penetrationNum = zeros(nbSPoints,1);  % Number of timesteps with penetration
metrics.convergeTime = zeros(nbSPoints,1);  % Discrete integration of the time
metrics.convergeDist = zeros(nbSPoints,1);  % Discrete integration of the distance
metrics.convergeEnergy = zeros(nbSPoints,1);  % Discrete integration of energy


%% parsing inputs
if isempty(varargin)
    options = check_options();
else
    options = check_options(varargin{1}); % Checking the given options, and add ones are not defined.
end

if ~isfield(options,'obstacleAvoidanceFunction') % integration time step
    fprintf('Default function handle \n')
    obsFunc_handle = @(x,xd,obs,b_contour,varargin) obs_modulation_fluidMechanics(x,xd,obs,b_contour, varargin);
    %obsFunc_handle = @(x,xd,obs,b_contour,varargin) obs_modulation_ellipsoid(x,xd,obs,b_contour, varargin);
    
else
    obsFunc_handle = options.obstacleAvoidanceFunction;
end


if options.model == 2 %2nd order
    d=size(x0,1)/2; %dimension of the model
    if isempty(xT)
        xT = zeros(2*d,1);
    elseif 2*d~=size(xT,1)
        disp('Error: length(x0) should be equal to length(xT)!')
        x=[];xd=[];t=[];
        return
    end
else
    d=size(x0,1); %dimension of the model
    if isempty(xT)
        xT = zeros(d,1);
    elseif d~=size(xT,1)
        disp('Error: length(x0) should be equal to length(xT)!')
        x=[];xd=[];t=[];
        return
    end
end

%% setting initial values
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
    b_contour = zeros(1,nbSPoints);
else
    obs_bool = false;
    obs = [];
    x_obs = NaN;
end

%initialization
for iSim=1:nbSPoints
    x(:,1,iSim) = x0(:,iSim);
end
if options.model == 2; %2nd order
    xd = zeros(d,1,nbSPoints);
else
    xd = zeros(size(x));
end
if size(xT) == size(x0)
    XT = xT;
else
    XT = repmat(xT,1,nbSPoints); %a matrix of target location (just to simplify computation)
end

            
t=0; %starting time

if options.plot %plotting options
    if isfield(options,'figure')
        sp.fig = options.figure;
    else
        sp = [];
    end
    if d>1
        sp = plot_results('i',sp,x,xT,obs);
    else
        sp = plot_results('i',sp,[x;0],[xT;0],obs);
    end
end

%% Simulation
iSim=1; % simulation iteration variable
while true
    %Finding xd using fn_handle.
    if options.timeDependent
        tt = repmat(t(iSim),1,nbSPoints);
        if nargin(fn_handle) == 1   
            xd(:,iSim,:)=reshape(fn_handle([squeeze(x(:,iSim,:))-XT;tt]),[d 1 nbSPoints]);
        else
            xd(:,iSim,:)=reshape(fn_handle(tt,squeeze(x(:,iSim,:))-XT),[d 1 nbSPoints]);
        end
    else
        xd(:,iSim,:)=reshape(fn_handle(squeeze(x(:,iSim,:))-XT),[d 1 nbSPoints]);
    end
    
    xd_init = squeeze(xd(:,end,:));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This part if for the obstacle avoidance module
    if obs_bool
        % applying perturbation on the obstacles
        for n=1:length(obs) 
            w_obs(n) = 0;
            xd_obs(:,n) = [0;0];
            if isfield(obs{n},'perturbation')
                if iSim >= round(obs{n}.perturbation.t0/options.dt)+1 && iSim <= round(obs{n}.perturbation.tf/options.dt) && length(obs{n}.perturbation.dx)==d
                    x_obs{n}(:,end+1) = x_obs{n}(:,end) + obs{n}.perturbation.dx*options.dt;
                    obs{n}.x0 = x_obs{n}(:,end);
                    xd_obs(:,n) = obs{n}.perturbation.dx; % all velocities ????
                    
                    if isfield(obs{n}.perturbation,'w') % Check rotational rate
                        w_obs(n) = obs{n}.perturbation.w;
                    end

                    if options.plot %plotting options
                        plot_results('o',sp,x,xT,n,obs{n}.perturbation.dx*options.dt, obs);
                    end
                end
            end
        end
        for j=1:nbSPoints % j - iteration over number of starting points
           %[xd(:,iSim,j), b_contour(j)] = obs_modulation_ellipsoid(x(:,iSim,j),xd(:,iSim,j),obs,b_contour(j),xd_obs);
           %[xd(:,iSim,j), b_contour(j)] = obs_modulation_ellipsoid_adapted(x(:,iSim,j),xd(:,iSim,j),obs,b_contour(j),xd_obs);
           %[xd(:,iSim,j), b_contour(j)] = obs_modulation_fluidMechanics(x(:,iSim,j),xd(:,iSim,j),obs,b_contour(j),xd_obs, w_obs);
           [xd(:,iSim,j), b_contour(j), ~, compTime_temp] = obsFunc_handle(x(:,iSim,j),xd(:,iSim,j),obs,b_contour(j),xd_obs, w_obs);
        end
        
        metrics.compTime(j) = metrics.compTime(j) + compTime_temp;
        
        for n=1:length(obs) % integration of object (linear motion!) -- first order.. 
            obs{n}.th_r(:) =  obs{n}.th_r + w_obs(n)*options.dt;
            x_obs{n}(:,end+1) = x_obs{n}(:,end);
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Integration
        
    if options.model == 2; %2nd order
        x(d+1:2*d,iSim+1,:)=x(d+1:2*d,iSim,:)+xd(:,iSim,:)*options.dt;
        x(1:d,iSim+1,:)=x(1:d,iSim,:)+x(d+1:2*d,iSim,:)*options.dt;
        
    else
        x(:,iSim+1,:)=x(:,iSim,:)+xd(:,iSim,:)*options.dt;
    end
    t(iSim+1)=t(iSim)+options.dt;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Applying perturbation if any
    switch options.perturbation.type
        case 'tdp' %applying target discrete perturbation
            if iSim == round(options.perturbation.t0/options.dt)+1 && length(options.perturbation.dx)==d
                xT(:,end+1) = xT(:,end) + options.perturbation.dx;
                XT = repmat(xT(:,end),1,nbSPoints);
                if options.plot %plotting options
                    plot_results('t',sp,x,xT);
                end
            else
                xT(:,end+1) = xT(:,end);
            end
        case 'rdp' %applying robot discrete perturbation
            if iSim == round(options.perturbation.t0/options.dt)+1 && length(options.perturbation.dx)==d
                x(:,iSim+1,:) = x(:,iSim+1,:) + repmat(options.perturbation.dx,[1 1 nbSPoints]);
            end
        case 'tcp' %applying target continuous perturbation
            if iSim >= round(options.perturbation.t0/options.dt)+1 && iSim <= round(options.perturbation.tf/options.dt) && length(options.perturbation.dx)==d
                xT(:,end+1) = xT(:,end) + options.perturbation.dx*options.dt;
                XT = repmat(xT(:,end),1,nbSPoints);
                if options.plot %plotting options
                    %plot_results('t',sp,x,xT);
                end
            else
                xT(:,end+1) = xT(:,end);
            end
        case 'rcp' %applying robot continuous perturbation
            if iSim >= round(options.perturbation.t0/options.dt)+1 && iSim <= round(options.perturbation.tf/options.dt) && length(options.perturbation.dx)==d
                x(:,iSim+1,:) = x(:,iSim+1,:) + repmat(options.perturbation.dx,[1 1 nbSPoints])*options.dt;
            end
    end

    % plotting the result
    if options.plot
        if d>1
            plot_results('u',sp,x,xT);
        else
            plot_results('u',sp,[x(1:end-1);xd],[xT;0]);
        end
    end
    axis equal;

    %%%%%%%%%%%%%%%%% Check for collision %%%%%%%%%%%%%%%%%%%%%%%
    for ii = 1:size(x,3)
        if obs_check_collision(obs,x(:,end,ii)) %Observe collision
            % TODO: check tis
            metrics.penetrationNum(ii) = metrics.penetrationNum(ii) +1 ;
            plot(x(1,end,ii) , x(2,end,ii) , 'kx', 'Linewidth', 10);
            fprintf('Collision detected at [%3.2f,%3.3f] \n',x(:,end,ii) ); 
            %pause;
        end
    end
    
    %%%%%%%%%%%%%%%% Create Animation %%%%%%%%%%%%%%%%%%%%%%%
    if(movieSave) % write to file
%         frame = getframe(sp.fig); 
%         im = frame2im(frame); 
%         [imind,cm] = rgb2ind(im,256); 
%         % Write to the GIF File 
%         if i == 1 
%           imwrite(imind,cm,filename,'gif', 'Loopcount',Inf,'DelayTime',1); 
%         else 
%           imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.05);        
%         end 
        %# create movie
       %surf(sin(2*pi*k/20)*Z, Z)
        writeVideo(vidObj, getframe(gca));
        %close(gcf)
        %# save as AVI file, and open it using system video player
    end

    xd_3last = xd(:,max([1 iSim-3]):iSim,:);
    xd_3last(isnan(xd_3last)) = 0;

    if and(dataAnalysis, iSim > 1)
       sumSqr = sum((squeeze(xd(:,end,:)) - xd_init).^2,1);
       metrics.relativeChangeSqr = metrics.relativeChangeSqr + sumSqr;
        for ii = 1:size(x,3)
            if metrics.convergeTime(ii) == 0 % not converged yet
                lastDistSqrd =  sum((x(:,end,ii)-x(:,end-1,ii)).^2);
                metrics.convergeDist(ii) = metrics.convergeDist(ii) + sqrt(lastDistSqrd);  % Discrete integration of the distance
                metrics.convergeEnergy = metrics.convergeEnergy +  sqrt(sum((xd(:,end,ii)-xd(:,end-1,ii)).^2)*lastDistSqrd); % Discrete integration of energy

                if and(iSim>3, all(all(abs(xd_3last(:,:,ii))))<options.tol) % is converged now
                    xd_3last_part = xd_3last(:,:,ii)
                    metrics.convergeTime(ii) = t(iSim+1);
                    fprintf('Traj %d converged at %d s \n', ii, metrics.convergeTime(ii))
                end
            end
        end
    end

    if(maxVal) % limit to maximum value, in case of locally stable system
        xlim([-5,6])
        ylim([-5,6])
        value_max = 1000;
        x(:,iSim,:) = min(value_max, x(:,iSim,:));
    end
    % Save simulation pictures
    if and(figSave, (options.i_max-2)/N_pigSave*picNum < iSim)
        print(strcat('fig/',simulationName,'_fig',num2str(picNum)),'-depsc')
        picNum = picNum +1;
    end
    
    %Checking the convergence
    if all(all(all(abs(xd_3last)<options.tol))) || iSim>options.i_max-2
        if options.plot
            %plot_results('f',sp,x,xT);
        end
        iSim=iSim+1;

        % xd(:,i,:)=reshape(fn_handle(squeeze(x(:,i,:))-XT),[d 1 nbSPoint]);
        x(:,end,:) = [];
        t(end) = [];
        fprintf('Number of Iterations: %1.0f\n',iSim)
        tmp='';
        for j=1:d
            tmp=[tmp ' %1.4f ;'];
        end
        tmp=tmp(2:end-2);
        fprintf('Final Time: %1.2f (sec)\n',t(1,end,1))
        fprintf(['Final Point: [' tmp ']\n'],squeeze(x(:,end,:)))
        fprintf(['Target Position: [' tmp ']\n'],xT(:,end))
        fprintf('## #####################################################\n\n\n')
        
        if iSim>options.i_max-2
            fprintf('Simulation stopped since it reaches the maximum number of allowed iterations i_max = %1.0f\n',iSim)
            fprintf('Exiting without convergence!!! Increase the parameter ''options.i_max'' to handle this error.\n')
        end
        
        if figSave % Save the last picture
            print(strcat('fig/',simulationName,'_fig',num2str(picNum)),'-depsc')
        end
        
        break
    end
    

       
    iSim=iSim+1; % increment time
end

%%%%%%%% End simulation loop %%%%%%%%
if(movieSave); close(vidObj); end % Close video obj

for ii = 1:nbSPoints % if not converged
    if metrics.convergeTime(ii) == 0
        metrics.convergeTime(ii) = t(end);            
    end
end

metrics.compTime = metrics.compTime/iSim; % Get average incerement time
metrics.badTrajectoriesNum = sum(metrics.penetrationNum>0);
if metrics.badTrajectoriesNum== 0
    metrics.penetrationNum = 0;
else
    metrics.penetrationNum = sum(metrics.penetrationNum)/metrics.badTrajectoriesNum;
end

if(dataAnalysis)
    fileID = fopen(strcat('simulationResults/table_',simulationName,'.txt'),'w');
    fprintf(fileID,'Relative Change [m^2] & Numer of Trajecctories with Penetration [] & Penetration Steps [] & Convergence Time [s] & Convergence Distance [m] & Normaliced Energy [J/kg]  & Computational Time [ms]\\\\ \\hline \n');
    fprintf(fileID,'%3.4f & %3.4f & %3.4f & %3.4f & %3.4f  & %3.4f\\\\ \\hline \n', ....
            sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum, metrics.penetrationNum, ...
            sum(metrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints), sum(metrics.compTime)/nbSPoints*1000;
    fclose(fileID);
end

end