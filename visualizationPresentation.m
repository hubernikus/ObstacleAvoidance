%function []=visualizationPresentation()
clc; close all; clear all;

figPosition = [100,100, 800,600];

dim = 2;

%% Avoidance cylinder
DS_handle = @(x) (1*[1;0])

N_circ = 50;
r = 1;
phi = 2*pi/N_circ*(1:N_circ);

x_circ = r*cos(phi);
y_circ = r*sin(phi);

xLimit = [-5,5];        
yLimit = [-4,4];        


N_x = 20;
N_y = N_x;

xSeq = linspace(xLimit(1),xLimit(2), N_x);
ySeq = linspace(yLimit(1),yLimit(2), N_y);

[X, Y] = meshgrid(xSeq, ySeq);

xd_bar = zeros(dim, N_x, N_y);
for ix = 1:N_x
    for iy = 1:N_y
        x_temp = [X(ix,iy);Y(ix,iy)];
        xd_bar(:,ix,iy) = DS_handle(x_temp);
        xd_bar(:,ix,iy) = modulationUnitCircle(x_temp,xd_bar(:,ix,iy));
    end
end

%%

fig_circle = figure('Position', figPosition);
set(groot,'DefaultAxesFontSize',12)
set(groot,'DefaultLineLineWidth',0.8)
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')
set(gca,'xtick',[],'ytick',[]); box on;
%axis off;
xlim(xLimit);ylim(yLimit);
axis equal;
patch(x_circ, y_circ,[0.6 1 0.6], 'FaceAlpha',1); hold on;
quiver(X,Y,squeeze(xd_bar(1,:,:)),squeeze(xd_bar(2,:,:)))
print(strcat('fig_vector/','quiver_IFD'), '-depsc','-r300');

fig_circle_streamslice= figure('Position', figPosition);
set(groot,'DefaultAxesFontSize',12)
set(groot,'DefaultLineLineWidth',0.8)
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')
set(gca,'xtick',[],'ytick',[]); box on;
%axis off;
xlim(xLimit*0.9);ylim(yLimit*0.9);
axis equal;
patch(x_circ, y_circ,[0.6 1 0.6], 'FaceAlpha',1); hold on;
streamslice(X,Y,squeeze(xd_bar(1,:,:)),squeeze(xd_bar(2,:,:)))
print(strcat('fig_vector/','streamslice_IFD'), '-depsc','-r300');     
%% Avoidance cylinder

N_circ = 50;
r = 1;
phi = 2*pi/N_circ*(1:N_circ);

x_circ = r*cos(phi);
y_circ = r*sin(phi);

xLimit = [-5,5];        
yLimit = [-4,4];        


N_x = 20;
N_y = N_x;

xSeq = linspace(xLimit(1),xLimit(2), N_x);
ySeq = linspace(yLimit(1),yLimit(2), N_y);

[X, Y] = meshgrid(xSeq, ySeq);

xd_bar = zeros(dim, N_x, N_y);
for ix = 1:N_x
    for iy = 1:N_y
        x_temp = [X(ix,iy);Y(ix,iy)];
        xd_bar(:,ix,iy) = DS_handle(x_temp);
        xd_bar(:,ix,iy) = modulationUnitCircle(x_temp,xd_bar(:,ix,iy));
    end
end

fig_circle = figure('Position', figPosition);
set(groot,'DefaultAxesFontSize',12)
set(groot,'DefaultLineLineWidth',0.8)
xlabel('$\xi_1$','interpreter','latex')
ylabel('$\xi_2$','interpreter','latex')
set(gca,'xtick',[],'ytick',[]); box on;
%axis off;
xlim(xLimit);ylim(yLimit);
axis equal;
patch(x_circ, y_circ,[0.6 1 0.6], 'FaceAlpha',1); hold on;
quiver(X,Y,squeeze(xd_bar(1,:,:)),squeeze(xd_bar(2,:,:)))



%%
%return;

%end

function [dx] = modulationUnitCircle(x,dx)

r = sqrt(sum(x.^2));
R_theta = rotMatrix(x,2);

% Deflection Matrix
M = diag([(1 - 1/r^2),(1 +1/r^2)]);

dx = R_theta'*dx;
if(r<1)
    dx = [0;0];
else
    dx = M*dx;
end

dx = R_theta*dx;
end

function [R] = rotMatrix(angleInp, dim)
if(nargin<2)
    dim = 2; % Default dimension
end


if(length(angleInp) == 1)
    theta = angleInp; % angle is given
    % Trigonemetric function evaluation
    cosTheta = cos(theta);
    sinTheta = sin(theta);
elseif(length(angleInp) == 2) % Angle input is 2D vector
    x = angleInp;
    dist = norm(x);
    if (dist) % bigger than 0
        cosSinTheta = x/dist;
        cosTheta = cosSinTheta(1);
        sinTheta = cosSinTheta(2);
    else
        % Unit matrix
        cosTheta = 1;
        sinTheta = 0;
    end
else % Define 3D .. - quaternions?
    fprintf('not defined \n')
end

% Create Roation matrix
if (dim == 2)
    R = [cosTheta, -sinTheta;
         sinTheta, cosTheta];
else
    fprintf('rotMat of dimension=%d no implemented \n', dim);
    R = eye(dim);
end
end