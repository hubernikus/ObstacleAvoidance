function [] = multipleSimulations(x0, opt_sim, simulationName, fn_handle)
funcHandle = {};
funcHandle{1} = @(x,xd,obs,b_contour,varargin) obs_modulation_ellipsoid(x,xd,obs,b_contour, varargin);
funcHandle{2} = @(x,xd,obs,b_contour,varargin) obs_modulation_fluidMechanics(x,xd,obs,b_contour, varargin);
funcHandle{3} = @(x,xd,obs,b_contour,varargin) obs_modulation_rotation(x,xd,obs,b_contour, varargin);
funcHandle{4} = @(x,xd,obs,b_contour,varargin) obs_modulation_ellipsoid(x,xd,obs,b_contour, varargin);
avoidanceType = {'DMM','IFD','LRS','-'};

nbSPoints = size(x0,2);


for ii = 1:3
    opt_sim.obstacleAvoidanceFunction = funcHandle{ii};
    fig(ii) = figure('name',strcat(simulationName,'_',avoidanceType{ii}),'position',[200 100 700 700]);
    opt_sim.figure = fig(ii);
    [~,xd,~,~,~,metrics{ii}] = Simulation(x0,[],fn_handle,opt_sim); 
    
end


ii = 4;
opt_simInit = opt_sim;
opt_simInit.obstacle = [];
fig(ii) = figure('name',strcat(simulationName,'_none'),'position',[200 100 700 700]);
opt_simInit.figure = fig(ii);
[~,~,~,~,~,metrics{ii}] = Simulation(x0,[],fn_handle,opt_simInit); 


N_simuSteps = length(xd);

% Post treatement metric 
% TODO: include this in SImulatin file; do graphs comparison
for ii = 1:4 % Post modification of all metric varaiables
    metrics{ii}.avoidanceType = strcat('\multicolumn{1}{c|}{',avoidanceType{ii},'}');
    
    RCS = 0;
    for pp =1:nbSPoints
        N_simu = metrics{ii}.convergeTime(pp)/opt_sim.dt;
        RCS = RCS + sum(metrics{ii}.relativeChangeSqr/N_simu);
    end
    metrics{ii}.relativeChangeSqr =  RCS/nbSPoints;
    metrics{ii}.badTrajectoriesNum =  metrics{ii}.badTrajectoriesNum/nbSPoints*100;
    % Penetration number - no adaptation
    metrics{ii}.convergeTime =  sum(metrics{ii}.convergeTime,1)/nbSPoints;
    metrics{ii}.convergeDist =  sum(metrics{ii}.convergeDist,1)/nbSPoints;
    metrics{ii}.convergeEnergy = sum(metrics{ii}.convergeEnergy,1)/nbSPoints;
    metrics{ii}.compTime = 1000*sum(metrics{ii}.compTime)/nbSPoints;
end

% Print to latex table
metricsNames = {'avoidanceType', 'relativeChangeSqr', 'badTrajectoriesNum', 'penetrationNum', ...
                'convergeTime', 'convergeDist', 'convergeEnergy', 'compTime'};
%labels = {'Object Avoidance Type', 'Relative Change [m2/s2]', 'Number of Trajectories with Penetration [%%]', 'Penetration Steps [] ', ...
%          'Convergence Time [s]', 'Convergence Distance [m]', 'Normalized Energy [J/kg]', 'Computational Time [ms]'};
labels={'', 'SRC $[m2/s2]$' ,'$N_{pen} \, [\%]$' ,'$T_{pen} [-]$' ,'$T_{conv} [s]$' ,'$D_{conv} [m]$' ,'$\hat E_{conv} [J/kg]$' ,'$T_{comp} [ms]$'};
dataType = {'%s' ,'%4.1f' ,'%2.1f', '%3.1f' ,'%3.3f' ,'%3.3f' ,'%3.2f' ,'%3.4f', };      
      
      
fileID = fopen(strcat('simulationResults/transposed/table_',simulationName,'.txt'),'w');
for ii = 1:length(metricsNames)
   fprintf(fileID, strcat('%s', repmat(strcat(' & ', dataType{ii}),1,4), '\\\\ \\hline \n'), ...
            labels{ii}, metrics{4}.(metricsNames{ii}), metrics{1}.(metricsNames{ii}), metrics{2}.(metricsNames{ii}), metrics{3}.(metricsNames{ii} ));
end

%fprintf(fileID,'Object Avoidance Type & Relative Change [m2/s2] & Number of Trajectories with Penetration [%%] & Penetration Steps [] & Convergence Time [s] & Convergence Distance [m] & Normalized Energy [J/kg] & Computational Time [ms] \\\\ \\hline \n');
%fprintf(fileID,'%s & %3.4f & %2.1f& %3.4f & %3.4f & %3.4f & %3.4f & %3.4f \\\\ \\hline \n', ....
%            avoidanceType{ii}, sum(metrics.relativeChangeSqr), metrics.badTrajectoriesNum/nbSPoints*100, metrics.penetrationNum, ...
%            sum(rmetrics.convergeTime,1)/nbSPoints, sum(metrics.convergeDist,1)/nbSPoints, sum(metrics.convergeEnergy,1)/nbSPoints, 1000*sum(metrics.compTime)/nbSPoints);

fclose(fileID);

end