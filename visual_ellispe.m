%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%       Visualize differences between DMM - IFD
%           
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear all;

saveFig = true;
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



%%
% Ellipse of the form
% x^2/a^2 + y^2/b^b = 1
a_elli = 0.8;
b_elli = 1.5;

xAttractor = [1;0];
xAttractor_unit = xAttractor.*1./[a_elli,b_elli]';

l_normal = b_elli*0.25;

lw = 100;

%% Circle ellipse with IFD

N_grid = 6;
xLim = (l_normal+1)*[-1,1]; yLim = (l_normal+1)*[-1,1];
[X,Y] = meshgrid(xLim(1) : (xLim(2)-xLim(1))/N_grid : xLim(2), ...
                yLim(1) : (yLim(2)-yLim(1))/N_grid : yLim(2));

xLim_plot = [X(1,1)+(X(1,1)-X(1,2))*0.2,  X(1,end)+(X(1,end)-X(1,end-1))*0.2];
yLim_plot = [Y(1,1)+(Y(1,1)-Y(2,1))*0.2,  Y(end,1)+(Y(end,1)-Y(end-1,1))*0.2];
N_test = 12;

i=1;
obs{i}.a = [1;1];
obs{i}.p = [1;1];
obs{i}.x0 = [0;0];
obs{i}.sf = [1];

r_circ = 1;
phi_circ = linspace(0,2*pi,48);

x_circ = cos(phi_circ)*r_circ;
y_circ = sin(phi_circ)*r_circ;


x_ellipse = a_elli*x_circ;
y_ellipse = b_elli*y_circ;


Jac_circ= @(x,y) ([2*x/(1)^2, 2*y/(1)]);
it_test = floor(length(phi_circ)/N_test*(1:N_test));
l_normal = r_circ/2;

figure('Position',[100,100,400,350]);
%subplot(1,2,1)
[x_obs_boundary, x_obs_sf] = obs_draw_ellipsoid(obs,50);
plot(x_circ,y_circ,'k'); hold on;
grid on;
axis equal;
set(groot,'DefaultAxesFontSize',12)
set(groot,'DefaultLineLineWidth',0.8)
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')

xlim(xLim_plot); ylim(yLim_plot)
[Nx,Ny] = size(X);
u = zeros(Nx,Ny); v = zeros(Nx,Ny);u_noCol = zeros(Nx,Ny); v_noCol= zeros(Nx,Ny);
u_flat = zeros(Nx,Ny); v_flat = zeros(Nx,Ny);
u_circ = zeros(Nx,Ny); v_circ = zeros(Nx,Ny);
[collisionMatrix,X_noCollision, Y_noCollision] = obs_check_collision(obs,X,Y); 

for ix = 1:Nx
    for iy = 1:Ny
        dx = stableDS([X(ix,iy),; Y(ix,iy)],xAttractor);
        u(ix,iy) = dx(1); v(ix,iy) =dx(2);
        if(not(collisionMatrix(ix,iy)))
            u_noCol(ix,iy) = dx(1); v_noCol(ix,iy) =dx(2);
            %[dx_bar,~,~,~] = obs_modulation_ellipsoid([X(ix,iy);Y(ix,iy)],dx, obs, 0, 0,0);
            [dx_bar,~,~,~] = obs_modulation_fluidMechanics([X(ix,iy);Y(ix,iy)],dx, obs, 0, [0;0],0);
            u_circ(ix,iy) = dx_bar(1); v_circ(ix,iy) =dx_bar(2);
        end
        dx = stableDS([X(ix,iy)*a_elli,; Y(ix,iy)*b_elli],xAttractor);
        u(ix,iy) = dx(1); v(ix,iy) =dx(2);
        dx = [a_elli;b_elli].*dx;
        u_flat(ix,iy) = dx(1); v_flat(ix,iy) =dx(2);
        
    end
end
quiver(X(:,:), Y(:,:), u_circ, v_circ, 'b')
quiver(X(:,:), Y(:,:), u_noCol, v_noCol, 'Color', [0.2,0.2,0.2])


for i = 1:N_test
    n_dir = Jac_circ(x_circ(it_test(i)), y_circ(it_test(i)));
    n_dir = n_dir/norm(n_dir);

    plot([x_circ(it_test(i))],[y_circ(it_test(i))],'ko')        
    drawArrow_vel([x_circ(it_test(i)); y_circ(it_test(i))], n_dir*l_normal*0.8,{'color', [1 0 0.], 'LineWidth',2})
    %plot([x_circ(it_test(i)),x_circ(it_test(i))+n_dir(1)*l_normal],...
    %     [y_circ(it_test(i)),y_circ(it_test(i))+n_dir(2)*l_normal],...
    %     'Color',[1 0 0.])
     
    drawArrow_vel([x_circ(it_test(i)); y_circ(it_test(i))], [-n_dir(2);n_dir(1)]*l_normal*0.8,{'color', [0 0.6 0.], 'LineWidth',2})
    %plot([x_circ(it_test(i))-n_dir(2)*l_normal*0.,x_circ(it_test(i))+n_dir(2)*l_normal*1],...
    %     [y_circ(it_test(i))+n_dir(1)*l_normal*0.,y_circ(it_test(i))-n_dir(1)*l_normal*1],...
    %     'Color',[0. 0.6 0])
end


plot(xAttractor_unit(1),xAttractor_unit(2),'kh','LineWidth', lw)
patchs = patch(x_obs_boundary(1,:),x_obs_boundary(2,:),[0.6 1 0.6], 'FaceAlpha',0.2); hold on;

if saveFig
    print(strcat('fig_vector/','avoidingEllipse_IFD_CRF'),'-depsc')
end

%%

% figure('Position',[100,100,1400,800]);
% %subplot(1,2,1)
% quiver(X(:,:), Y(:,:), u, v, 'b'); hold on;
% quiver(X(:,:), Y(:,:), u_flat, v_flat, 'r'); 
% plot(xAttractor_unit(1),xAttractor_unit(2),'k*','LineWidth', lw)
% grid on;
% axis equal;
% xlim(xLim); ylim(yLim)

%% Aligned Ellipse with IFD and DMM
i=1;
obs{i}.a = [a_elli;b_elli].*[1;1];
%obs{i}.p = [1;1];
%obs{i}.x0 = [0;0];

l_normal = a_elli;
xLim = (l_normal+a_elli)*[-1,1]; yLim = (l_normal+b_elli)*[-1,1];
[X,Y] = meshgrid(xLim(1) : (xLim(2)-xLim(1))/N_grid : xLim(2), ...
                yLim(1) : (yLim(2)-yLim(1))/N_grid : yLim(2));
            
% Plot limits            
xLim_plot = [X(1,1)+(X(1,1)-X(1,2))*0.2,  X(1,end)+(X(1,end)-X(1,end-1))*0.2];
yLim_plot = [Y(1,1)+(Y(1,1)-Y(2,1))*0.2,  Y(end,1)+(Y(end,1)-Y(end-1,1))*0.2];            
            
            
[Nx,Ny] = size(X);
u = zeros(Nx,Ny); v = zeros(Nx,Ny);
u_safe = zeros(Nx,Ny); v_safe = zeros(Nx,Ny);
u_ifd = zeros(Nx,Ny); v_ifd = zeros(Nx,Ny);
u_elli = zeros(Nx,Ny); v_elli = zeros(Nx,Ny);
[collisionMatrix,X_noCollision, Y_noCollision] = obs_check_collision(obs,X,Y); 
[x_obs_boundary, x_obs_sf] = obs_draw_ellipsoid(obs,50);

for ix = 1:Nx
    for iy = 1:Ny
        dx = stableDS([X(ix,iy),; Y(ix,iy)],xAttractor);
        u(ix,iy) = dx(1); v(ix,iy) =dx(2);
                
        if(not(collisionMatrix(ix,iy)))
            u_safe(ix,iy) = dx(1); v_safe(ix,iy) =dx(2);
            [dx_bar,~,~,~] = obs_modulation_fluidMechanics([X(ix,iy);Y(ix,iy)],dx, obs,0,[0;0],0);
            u_ifd(ix,iy) = dx_bar(1); v_ifd(ix,iy) = dx_bar(2);
            
            contour = -1;
            [dx_bar,~,~,~] = obs_modulation_ellipsoid([X(ix,iy);Y(ix,iy)],dx, obs,contour,[0;0],0);
            u_elli(ix,iy) = dx_bar(1); v_elli(ix,iy) =dx_bar(2);
        end
    end
end


figure('Position',[100,100,300,350]);
plot(x_ellipse, y_ellipse,'k'); hold on;
plot(xAttractor(1),xAttractor(2),'kh','LineWidth', lw)
quiver(X(:,:), Y(:,:), u_ifd, v_ifd, 'b')
quiver(X(:,:), Y(:,:), u_safe, v_safe, 'Color',[0.3,0.3,0.3])
set(groot,'DefaultAxesFontSize',12)
set(groot,'DefaultLineLineWidth',0.8)
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')
patchs = patch(x_obs_boundary(1,:),x_obs_boundary(2,:),[0.6 1 0.6], 'FaceAlpha',0.2); hold on;


grid on;
axis equal;
xlim(xLim_plot); ylim(yLim_plot);


if saveFig
    print(strcat('fig_vector/','avoidingEllipse_IFD_noEig'),'-depsc')
end

% Evaulate Ellipse avoidance with IFD
for i = 1:N_test
    %n_dir = Jac_circ(x_circ(it_test(i)), y_circ(it_test(i)));
    n_dir = [a_elli*x_circ(it_test(i)); b_elli*y_circ(it_test(i))];
    n_dir = n_dir/norm(n_dir);
    
    t_dir = [-a_elli^2*n_dir(2);b_elli^2*n_dir(1)];
    t_dir = t_dir/norm(t_dir);
    
    plot(a_elli^1*[x_circ(it_test(i))],b_elli*[y_circ(it_test(i))],'ko')        
    
    drawArrow_vel([a_elli*x_circ(it_test(i)); b_elli*y_circ(it_test(i))], n_dir*l_normal*0.8,{'color', [1 0 0.], 'LineWidth',2})
%     plot(a_elli*[x_circ(it_test(i)),x_circ(it_test(i))+n_dir(1)*l_normal],...
%          b_elli*[y_circ(it_test(i)),y_circ(it_test(i))+n_dir(2)*l_normal],...
%          'Color',[1,0,0])
    drawArrow_vel([a_elli*x_circ(it_test(i)); b_elli*y_circ(it_test(i))], t_dir*l_normal*0.8,{'color', [0 0.6 0.], 'LineWidth',2})
%     plot(a_elli*[x_circ(it_test(i))-n_dir(2)*l_normal*0.0,x_circ(it_test(i))+n_dir(2)*l_normal*1.],...
%         b_elli*[y_circ(it_test(i))+n_dir(1)*l_normal*0.0,y_circ(it_test(i))-n_dir(1)*l_normal*1.],...
%         'Color',[0,0.6,0])
end

if saveFig
    print(strcat('fig_vector/','avoidingEllipse_IFD_ERF'),'-depsc')
end

%  DMM
Jac_ellipse = @(x,y) ([2*x/(a_elli)^2, 2*y/(b_elli)^2]);

figure('Position',[100,100,300,350]);
plot(x_ellipse, y_ellipse,'k'); hold on;
plot(xAttractor(1),xAttractor(2),'kh','LineWidth', lw)
patchs = patch(x_obs_boundary(1,:),x_obs_boundary(2,:),[0.6 1 0.6], 'FaceAlpha',0.2); hold on;
quiver(X(:,:), Y(:,:), u_elli, v_elli, 'b')
quiver(X(:,:), Y(:,:), u_safe, v_safe, 'Color',[0.3,0.3,0.3])
set(groot,'DefaultAxesFontSize',12)
set(groot,'DefaultLineLineWidth',0.8)
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')
grid on;
axis equal;
xlim(xLim_plot); ylim(yLim_plot);



for i = 1:N_test
    n_dir = Jac_ellipse(x_ellipse(it_test(i)), y_ellipse(it_test(i)));
    n_dir = n_dir/norm(n_dir);

    plot([x_ellipse(it_test(i))],[y_ellipse(it_test(i))],'ko')        
    
    drawArrow_vel([x_ellipse(it_test(i)); y_ellipse(it_test(i))], n_dir*l_normal*0.8,{'color', [1 0 0.], 'LineWidth',2})
%     plot([x_ellipse(it_test(i)),x_ellipse(it_test(i))+n_dir(1)*l_normal],...
%          [y_ellipse(it_test(i)),y_ellipse(it_test(i))+n_dir(2)*l_normal],...
%          'Color',[1,0,0.0])
      
    drawArrow_vel([x_ellipse(it_test(i)); y_ellipse(it_test(i))], [-n_dir(2);n_dir(1)]*l_normal*0.8,{'color', [0 0.6 0.], 'LineWidth',2})
%     plot([x_ellipse(it_test(i))-n_dir(2)*l_normal*0.,x_ellipse(it_test(i))+n_dir(2)*l_normal*1.],...
%          [y_ellipse(it_test(i))+n_dir(1)*l_normal*0.,y_ellipse(it_test(i))-n_dir(1)*l_normal*1.],...
%           'Color',[0,0.6,0])
end

%subplot(1,3,3)

grid on;
axis equal;
xlim(xLim_plot); ylim(yLim_plot);

if saveFig
    print(strcat('fig_vector/','avoidingEllipse_DMM'),'-depsc')
end

%% Rotated Ellipe with IFD 
i=1;
obs{1}.th_r = 30*pi/180;
obs{1}.x0 = [0;0];

phi = obs{1}.th_r;
R = [cos(phi), -sin(phi);
     sin(phi), cos(phi)];
 
l_normal = a_elli;
xLim = (l_normal+a_elli)*[-1,1]; yLim = (l_normal+b_elli)*[-1,1];
[X,Y] = meshgrid(xLim(1) : (xLim(2)-xLim(1))/N_grid : xLim(2), ...
                yLim(1) : (yLim(2)-yLim(1))/N_grid : yLim(2));
            
% Plot limits            
xLim_plot = [X(1,1)+(X(1,1)-X(1,2))*0.2,  X(1,end)+(X(1,end)-X(1,end-1))*0.2];
yLim_plot = [Y(1,1)+(Y(1,1)-Y(2,1))*0.2,  Y(end,1)+(Y(end,1)-Y(end-1,1))*0.2];            
            
            
[Nx,Ny] = size(X);
u = zeros(Nx,Ny); v = zeros(Nx,Ny);
u_safe = zeros(Nx,Ny); v_safe = zeros(Nx,Ny);
u_ifd = zeros(Nx,Ny); v_ifd = zeros(Nx,Ny);
u_elli = zeros(Nx,Ny); v_elli = zeros(Nx,Ny);
[collisionMatrix,X_noCollision, Y_noCollision] = obs_check_collision(obs,X,Y); 
[x_obs_boundary, x_obs_sf] = obs_draw_ellipsoid(obs,50);

% Rotate attractor
xAttractor_rot = R*xAttractor;

for ix = 1:Nx
    for iy = 1:Ny
        dx = stableDS([X(ix,iy),; Y(ix,iy)],xAttractor_rot);
        u(ix,iy) = dx(1); v(ix,iy) =dx(2);
                
        if(not(collisionMatrix(ix,iy)))
            u_safe(ix,iy) = dx(1); v_safe(ix,iy) =dx(2);
            [dx_bar,~,~,~] = obs_modulation_fluidMechanics([X(ix,iy);Y(ix,iy)],dx, obs,0,[0;0],0);
            u_ifd(ix,iy) = dx_bar(1); v_ifd(ix,iy) = dx_bar(2);
            
        end
    end
end
i=1;
obs{1}.th_r = 30*pi/180;

phi = obs{1}.th_r;
R = [cos(phi), -sin(phi);
     sin(phi), cos(phi)];
 
% Vusualization
 figure('Position',[100,100,300,350]);
%plot(x_ellipse, y_ellipse,'k'); hold on;  % TODO: REMOVE
plot(xAttractor_rot(1),xAttractor_rot(2),'kh','LineWidth', lw); hold on;
quiver(X(:,:), Y(:,:), u_ifd, v_ifd, 'b')
quiver(X(:,:), Y(:,:), u_safe, v_safe, 'Color',[0.3,0.3,0.3])
set(groot,'DefaultAxesFontSize',12)
set(groot,'DefaultLineLineWidth',0.8)
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')
patchs = patch(x_obs_boundary(1,:),x_obs_boundary(2,:),[0.6 1 0.6], 'FaceAlpha',0.2); hold on;


x_ifd = x_circ;
y_ifd = y_circ;

% for i = 1:N_test
%     n_dir = Jac_circ(x_circ(it_test(i)), y_circ(it_test(i)));
%     n_dir = n_dir/norm(n_dir);
%     t_dir = R*(diag([a_elli,b_elli])*[n_dir(2);-n_dir(1)]);
%     n_dir = R*(diag([a_elli,b_elli])*n_dir');
%     
%     r_circ = R*[a_elli*x_circ(it_test(i));b_elli*y_circ(it_test(i))];
%     
%     x_ifd(it_test(i)) = r_circ(1);
%     y_ifd(it_test(i)) = r_circ(2);
%     
%     %plot(a_elli*[x_circ(it_test(i))],b_elli*[y_circ(it_test(i))],'ko')        
%     plot([x_ifd(it_test(i))],[y_ifd(it_test(i))],'ko')
%     
%     plot([x_ifd(it_test(i)),x_ifd(it_test(i))+n_dir(1)*l_normal],...
%          [y_ifd(it_test(i)),y_ifd(it_test(i))+n_dir(2)*l_normal],...
%          'Color',[1,0,0])
%       
%     plot([x_ifd(it_test(i))+t_dir(1)*l_normal*0.0,x_ifd(it_test(i))+t_dir(1)*l_normal*1.],...
%          [y_ifd(it_test(i))+t_dir(2)*l_normal*0.0,y_ifd(it_test(i))+t_dir(2)*l_normal*1.],...
%         'Color',[0,0.6,0])
% end

grid on;
axis equal;
xlim(xLim_plot); ylim(yLim_plot);

if saveFig
    print(strcat('fig_vector/','avoidingEllipse_IFD_ERF_rotated'),'-depsc')
end

%% Rotated and Transposed Ellipe with IFD 
i=1;
obs{1}.th_r = 30*pi/180;
obs{1}.x0 = [2;1];
x0 = obs{1}.x0;

phi = obs{1}.th_r;
R = [cos(phi), -sin(phi);
     sin(phi), cos(phi)];
 
l_normal = a_elli;
xLim = (l_normal+a_elli)*[-1,1]; yLim = (l_normal+b_elli)*[-1,1];
xLim(1) = xLim(1)+x0(1)*0.9;
yLim(1) = yLim(1)+x0(2)*0.9;

xLim(2) = xLim(2)+x0(1);
yLim(2) = yLim(2)+x0(2);

[X,Y] = meshgrid(xLim(1) : (xLim(2)-xLim(1))/(N_grid*1.13) : xLim(2), ...
                yLim(1) : (yLim(2)-yLim(1))/(N_grid*1.13    ) : yLim(2) );

% Plot limits            
xLim_plot = [X(1,1)+(X(1,1)-X(1,2))*0.2,  X(1,end)+(X(1,end)-X(1,end-1))*0.2];
yLim_plot = [Y(1,1)+(Y(1,1)-Y(2,1))*0.2,  Y(end,1)+(Y(end,1)-Y(end-1,1))*0.2];            
            
[Nx,Ny] = size(X);
u = zeros(Nx,Ny); v = zeros(Nx,Ny);
u_safe = zeros(Nx,Ny); v_safe = zeros(Nx,Ny);
u_ifd = zeros(Nx,Ny); v_ifd = zeros(Nx,Ny);
u_elli = zeros(Nx,Ny); v_elli = zeros(Nx,Ny);
[collisionMatrix,X_noCollision, Y_noCollision] = obs_check_collision(obs,X,Y); 
[x_obs_boundary, x_obs_sf] = obs_draw_ellipsoid(obs,50);

% Rotate attractor
xAttractor_rot = R*xAttractor+x0;

for ix = 1:Nx
    for iy = 1:Ny
        dx = stableDS([X(ix,iy),; Y(ix,iy)],xAttractor_rot);
        u(ix,iy) = dx(1); v(ix,iy) =dx(2);
                
        if(not(collisionMatrix(ix,iy)))
            u_safe(ix,iy) = dx(1); v_safe(ix,iy) =dx(2);
            [dx_bar,~,~,~] = obs_modulation_fluidMechanics([X(ix,iy);Y(ix,iy)],dx, obs,0,[0;0],0);
            u_ifd(ix,iy) = dx_bar(1); v_ifd(ix,iy) = dx_bar(2);
            
        end
    end
end

% Visualize
figure('Position',[100,100,300,350]);
%plot(x_ellipse, y_ellipse,'k'); hold on;  % TODO: REMOVE
plot(xAttractor_rot(1),xAttractor_rot(2),'kh','LineWidth', lw); hold on;
quiver(X(:,:), Y(:,:), u_ifd, v_ifd, 'b')
quiver(X(:,:), Y(:,:), u_safe, v_safe, 'Color',[0.3,0.3,0.3])
set(groot,'DefaultAxesFontSize',12)
set(groot,'DefaultLineLineWidth',0.8)
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')
patchs = patch(x_obs_boundary(1,:),x_obs_boundary(2,:),[0.6 1 0.6], 'FaceAlpha',0.2); hold on;


x_ifd = x_circ;
y_ifd = y_circ;

% for i = 1:N_test
%     n_dir = Jac_circ(x_circ(it_test(i)), y_circ(it_test(i)));
%     n_dir = n_dir/norm(n_dir);
%     t_dir = R*(diag([a_elli,b_elli])*[n_dir(2);-n_dir(1)]);
%     n_dir = R*(diag([a_elli,b_elli])*n_dir');
%     
%     r_circ = R*[a_elli*x_circ(it_test(i));b_elli*y_circ(it_test(i))];
%     
%     x_ifd(it_test(i)) = r_circ(1)+x0(1);
%     y_ifd(it_test(i)) = r_circ(2)+x0(2);
%     
%     %plot(a_elli*[x_circ(it_test(i))],b_elli*[y_circ(it_test(i))],'ko')        
%     plot([x_ifd(it_test(i))],[y_ifd(it_test(i))],'ko')
%     
%     plot([x_ifd(it_test(i)),x_ifd(it_test(i))+n_dir(1)*l_normal],...
%          [y_ifd(it_test(i)),y_ifd(it_test(i))+n_dir(2)*l_normal],...
%          'Color',[1,0,0])
%       
%     plot([x_ifd(it_test(i))+t_dir(1)*l_normal*0.0,x_ifd(it_test(i))+t_dir(1)*l_normal*1.],...
%          [y_ifd(it_test(i))+t_dir(2)*l_normal*0.0,y_ifd(it_test(i))+t_dir(2)*l_normal*1.],...
%         'Color',[0,0.6,0])
% end

grid on;
axis equal;
xlim(xLim_plot); ylim(yLim_plot);

if saveFig
    print(strcat('fig_vector/','avoidingEllipse_IFD_ERF_rotTrans'),'-depsc')
end
 
%% Rotated and Transposed Ellipe with IFD 
i=1;
obs{1}.th_r = 30*pi/180;
obs{1}.x0 = [2;1];
x0 = obs{1}.x0;

phi = obs{1}.th_r;
R = [cos(phi), -sin(phi);
     sin(phi), cos(phi)];
 
l_normal = a_elli;
xLim = (l_normal+a_elli)*[-1,1]; yLim = (l_normal+b_elli)*[-1,1];
xLim(1) = xLim(1)+x0(1)*0.9;
yLim(1) = yLim(1)+x0(2)*0.9;

xLim(2) = xLim(2)+x0(1);
yLim(2) = yLim(2)+x0(2);

[X,Y] = meshgrid(xLim(1) : (xLim(2)-xLim(1))/(N_grid*1.13) : xLim(2), ...
                yLim(1) : (yLim(2)-yLim(1))/(N_grid*1.13    ) : yLim(2) );

% Plot limits            
xLim_plot = [X(1,1)+(X(1,1)-X(1,2))*0.2,  X(1,end)+(X(1,end)-X(1,end-1))*0.2];
yLim_plot = [Y(1,1)+(Y(1,1)-Y(2,1))*0.2,  Y(end,1)+(Y(end,1)-Y(end-1,1))*0.2];            
            
[Nx,Ny] = size(X);
u = zeros(Nx,Ny); v = zeros(Nx,Ny);
u_safe = zeros(Nx,Ny); v_safe = zeros(Nx,Ny);
u_dmm = zeros(Nx,Ny); v_ifd = zeros(Nx,Ny);
u_elli = zeros(Nx,Ny); v_dmm = zeros(Nx,Ny);
[collisionMatrix,X_noCollision, Y_noCollision] = obs_check_collision(obs,X,Y); 
[x_obs_boundary, x_obs_sf] = obs_draw_ellipsoid(obs,50);

% Rotate attractor
xAttractor_rot = R*xAttractor+x0;

for ix = 1:Nx
    for iy = 1:Ny
        dx = stableDS([X(ix,iy),; Y(ix,iy)],xAttractor_rot);
        u(ix,iy) = dx(1); v(ix,iy) =dx(2);
                
        if(not(collisionMatrix(ix,iy)))
            u_safe(ix,iy) = dx(1); v_safe(ix,iy) =dx(2);
            [dx_bar,~,~,~] = obs_modulation_ellipsoid([X(ix,iy);Y(ix,iy)],dx, obs,0,[0;0],0);
            u_dmm(ix,iy) = dx_bar(1); v_dmm(ix,iy) = dx_bar(2);
            
        end
    end
end

% Visualize
figure('Position',[100,100,300,350]);
%plot(x_ellipse, y_ellipse,'k'); hold on;  % TODO: REMOVE
plot(xAttractor_rot(1),xAttractor_rot(2),'kh','LineWidth', lw); hold on;
quiver(X(:,:), Y(:,:), u_dmm, v_dmm, 'b')
quiver(X(:,:), Y(:,:), u_safe, v_safe, 'Color',[0.3,0.3,0.3])
set(groot,'DefaultAxesFontSize',12)
set(groot,'DefaultLineLineWidth',0.8)
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')
patchs = patch(x_obs_boundary(1,:),x_obs_boundary(2,:),[0.6 1 0.6], 'FaceAlpha',0.2); hold on;

x_DMM = x_ellipse;
y_DMM = y_ellipse;

for i = 1:N_test
    n_dir = Jac_ellipse(x_ellipse(it_test(i)), y_ellipse(it_test(i)));
    
    n_dir = n_dir/norm(n_dir);
    
    n_dir = R*n_dir';
    
    r_DMM = x0 + R*[x_ellipse(it_test(i));y_ellipse(it_test(i))];
    
    x_DMM(it_test(i)) = r_DMM(1);
    y_DMM(it_test(i)) = r_DMM(2);
    
    plot([x_DMM(it_test(i))],[y_DMM(it_test(i))],'ko')        
    
    drawArrow_vel([x_DMM(it_test(i)); y_DMM(it_test(i))], n_dir*l_normal*0.8,{'color', [1.0 0. 0.], 'LineWidth',2})
    
    
    drawArrow_vel([x_DMM(it_test(i)); y_DMM(it_test(i))], [-n_dir(2);n_dir(1)]*l_normal*0.8,{'color', [0 0.6 0.], 'LineWidth',2})
    
%     plot([x_DMM(it_test(i)),x_DMM(it_test(i))+n_dir(1)*l_normal],...
%          [y_DMM(it_test(i)),y_DMM(it_test(i))+n_dir(2)*l_normal],...
%          'Color',[1,0,0.0])
%      
%     plot([x_DMM(it_test(i))-n_dir(2)*l_normal*0.,x_DMM(it_test(i))+n_dir(2)*l_normal*1.],...
%          [y_DMM(it_test(i))+n_dir(1)*l_normal*0.,y_DMM(it_test(i))-n_dir(1)*l_normal*1.],...
%           'Color',[0,0.6,0])
end

grid on;
axis equal;
xlim(xLim_plot); ylim(yLim_plot);

if saveFig
    print(strcat('fig_vector/','avoidingEllipse_DMM_ERF_rotTrans'),'-depsc')
end
 