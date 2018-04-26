% Visualization_divergence
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

%% Zeros Determinant
% clear all;
% 
% 
% obs = [];
% n=1; % obs number
% obs{n}.a = [2;2];
% obs{n}.p = [1;1];
% obs{n}.x0 = [8;0.0];
% obs{n}.sf = [1.0];
% obs{n}.th_r = 0*pi/180;
% obs{n}.x_center = [0.0;0.0];
% 
% w = 0;
% xd = 0;
% 
% simulationName = 'circleAt8';
% saveFig = true;

%

% Place obstacles
obs = [];
n=1;
obs{n}.a = [4;1];
obs{n}.p = [1;1];
obs{n}.x0 = [8;0.0];
obs{n}.sf = [1.0];
obs{n}.th_r = 30*pi/180;
obs{n}.x_center = [0.0;0.0];

w = 0;
xd = 0;

simulationName = 'ellipseDiag';
saveFig = true;

% 
% % %%
% 
% % Place obstacles 
% obs = [];
% n=1;
% obs{n}.a = [1;4];
% obs{n}.p = [1;1];
% obs{n}.x0 = [8;0.0];
% obs{n}.sf = [1.0];
% obs{n}.th_r = 0*pi/180;
% obs{n}.x_center = [0.0;0.0];
% 
% w = 0;
% xd = 0;
% 
% simulationName = 'ellipseVertical';
% saveFig = false;

%%
tic;
N = 201; % Size grid
N_x = N; 
N_y = N_x; 

d = 2; % Dimensions

rho = 1;


fn_handle_objAvoidance = @(x,xd,obs,varargin) ...
                          obs_modulation_convergence(x,xd,obs, varargin);

x_range = [-2,15]; y_range = [-8.8,8.8];
    
rotMat = compute_R(d,obs{1}.th_r);

xValues = linspace(x_range(1),x_range(2),N_x);
yValues = linspace(y_range(1),y_range(2),N_y);

d1 = obs{1}.x0(1);
if obs{1}.x0(2)
    warning('Attentioni - x2 \neq 0 ')
end

tol = 5*1e-3; % Tolerance for numerical zero valuation

% Initialize variables
poles = [];
zerosDet = [];
zerosTrace = []; 

% Value matrices
denominator = zeros(N_x,N_y);
determinantNominator = zeros(N_x,N_y);
traceNominator = zeros(N_x,N_y);
trace = zeros(N_x,N_y);
determinant = zeros(N_x,N_y);
eigValues = zeros(2,N_x, N_y);


for ix = 1:N_x
    x1 = xValues(ix);
    for iy = 1:N_y
        x2 = yValues(iy);
        x_t = rotMat'*([x1;x2]-obs{n}.x0);

        Gamma = sum((x_t./obs{n}.a).^(2*obs{n}.p));
        nv = (2*obs{n}.p./obs{n}.a.*(x_t./obs{n}.a).^(2*obs{n}.p - 1)); %normal vector of the tangential hyper-plane
%         E(1,2:d) = nv(2:d)';
%         E(2:d,2:d) = -eye(d-1)*nv(1);

%         t1 = E(1,2)
%         t2 = E(2,2);
        nv_hat = rotMat*nv;
        
        t1 = nv_hat(2);
        t2 = -nv_hat(1);
        
        D = eye(d)+diag([-1,1]*1/abs(Gamma)^(1/rho) );
        l_n = D(1,1);
        l_t = D(2,2);
        
%         l_n = 1-1/(Gamma);
%         l_t = 1+1/(Gamma);

        % Poles of trace and determinant
        denominator(ix,iy) = t1*x2-t2*x1+t2*d1;
%         determinantNominator(ix,iy) = lambda0/2*(lambda0*t1*x2 - lambda0*t2*x1 + lambda1*t2*d1);
%         traceNominator(ix,iy) = -((lambda0+lambda1)*t1*x2 + 2*(lambda0*t1*x2-lambda0*t2*x1));
%         denominator(ix,iy) = t2*x1+t2*d1;
        determinantNominator(ix,iy) = l_n/2*(l_n*t1*x2 - l_n*t2*x1 + l_t*t2*d1);
        traceNominator(ix,iy) = -((l_n+l_t)*t2*d1 + 2*(l_n*t1*x2-l_n*t2*x1));
        
        determinant(ix,iy) = determinantNominator(ix,iy)/denominator(ix,iy);
        trace(ix,iy) = traceNominator(ix,iy)/denominator(ix,iy);
        
        sqrtDet = sqrt(trace(ix,iy)^2/4-determinant(ix,iy));
        
        eigValues(1,ix,iy) = trace(ix,iy)/2 - real(sqrtDet);
        eigValues(2,ix,iy) = trace(ix,iy)/2 + real(sqrtDet);
        
%         if and(eigValues(1,ix,iy) > 0, x_t(1)<0) 
%             eigValues(1,ix,iy);
%         end
%         if and(eigValues(2,ix,iy) > 0, x_t(1)<0) 
%             eigValues(2,ix,iy);
%         end
%         val1_temp(ix,iy) = x1;
%         val2_temp(ix,iy) = x2;
        val1_temp(ix,iy) = x_t(1);
        val2_temp(ix,iy) = x_t(2);
        val3_temp(ix,iy) = Gamma;

%         if imag(sqrtDet)
%             sqrtDet
%         end
        
%         sqrt = (
%         traceValues(ix,iy) = 
%         traceValues(ix,iy) = (lambda0+lambda1)*t1*x2 + 2*(lambda1*t2*d1-lambda0*t2*x1);
        
        
%         if abs(denominatorValues(ix,iy)) < tol
%             poles = [poles, [x1;x2]];
%         else
            % Evaluate zeros of the determinante
%             if ~lambda0 
%                 zerosDet = [zerosDet, [x1;x2]];
%             else
                
%                 if abs(determinantValues(ix,iy)) < tol
%                     zerosDet = [zerosDet, [x1;x2]];
%                 end
%             end
            
            % Zeros of trace
            
%             if abs(traceValues(ix,iy)) < tol
%                 zerosTrace = [zerosTrace, [x1;x2]];
%             end 
%         end
    end
end

% Create colormap
% blue = [-lambda0*t2*x1;
%         -lambda0*t2*x1];
%green = [0,128,0;
%        144,238,144]/255;  
green = [200,255,200;
        0,155,0]/255;
    
red = [155,0,0;
       255,200,200]/255;
N=100;

map_4cols = four_colors(N, green, red); % Create desired coloring
   
%
[X, Y] = meshgrid(xValues,yValues);
collisionMatrix = obs_check_collision(obs, X, Y);
U = zeros(size(X));
V = zeros(size(X));


for ix = 1:N_x
    for iy = 1:N_y
%         if collisionMatrix(ix,iy)
%             U(ix,iy) = 0; V(ix,iy) = 0;
%         else
            x_dot = linearStableDS([X(ix,iy); Y(ix,iy)] );
%             if obsExist
                x_dot = fn_handle_objAvoidance([X(ix,iy);Y(ix,iy)], x_dot, obs, w, xd );
%             end
            U(ix,iy) = x_dot(1); 
            V(ix,iy) = x_dot(2);
%         end
    end
end


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

U = U.*not(collisionMatrix);
V = V.*not(collisionMatrix);

% Vectorfield

close all
for ii=1:5
    figure;
    switch ii
%         case 1
%             imagesc(xValues, yValues, val1_temp'); hold on; % Create plot    
%         case 2
%             imagesc(xValues, yValues, val2_temp'); hold on; % Create plot
%         case 3
%             imagesc(xValues, yValues, val3_temp'); hold on; % Create plot
        case 1
            imagesc(xValues, yValues, denominator'); hold on; % Create plot
            caxis(2*[-1,1]) % set axis range
            measureName = 'denominator';
%         case 2 
%             imagesc(xValues, yValues, determinantNominator'); hold on; % Create plot
%             caxis(2*[-1,1]) % set axis range
%         case 3
%             imagesc(xValues, yValues, traceNominator'); hold on; % Create plot
%             caxis(2*[-1,1]) % set axis range
        case 2
            imagesc(xValues, yValues, trace'); hold on; % Create plot
            caxis(2*[-1,1]) % set axis range
            measureName = 'trace';
        case 3 
            imagesc(xValues, yValues, determinant'); hold on; % Create plot
            caxis(2*[-1,1]) % set axis range
            measureName = 'determinant';
        case 4
            imagesc(xValues, yValues, squeeze(eigValues(1,:,:))' ); hold on; % Create plot
            caxis(2*[-1,1]) % set axis range
            measureName = 'eigValue1';
        case 5
            imagesc(xValues, yValues, squeeze( eigValues(2,:,:))' ); hold on; % Create plot
            caxis(2*[-1,1]) % set axis range
            measureName = 'eigValue2';
    end
    map_4cols = four_colors(N, green, red); % Create desired coloring
    colormap(map_4cols);
    colorbar; % show bar

    % Draw vector field
    stream = streamslice(X, Y, U, V,'k'); hold on;
    set(stream, 'Color', [0.2 0.2 0.2] );
    
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
    if saveFig
        print(strcat('fig_vector/analysisContraction_',simulationName, '_',measureName ), '-depsc')
    end
end
t_numericalZero = toc;

%fprintf('Numerical evaluation took %d seconds. \n', t_numericalZero)

