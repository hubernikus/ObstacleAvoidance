function [ obs ] = calculate_dynamic_center( obs, x_obs_sf, intersection_obs )
% Calculate the dynamic center for the applicaiton of the modulation
% matrix. A smooth transition is desired between objects far away (center
% in middle) and to convex obstacles intersecting. 

% This might not always be the completely optimal position of the
% dynamic_center of the obstacles, but it results in a continous movement
% of it and keeps computational cost low

% TODO --- 
% Increase sampling, when less thant 1/9 of original points are still 
% available; decrease inital number of sampling points to 27 initially
% do 3x -> sampling accuracy of 3^3*3^2*3^2 = 3^7 = 2187

% only implemented for 2d
dim = size(obs{1}.x0,1);

if(dim>2)
    warning('Not implemented for higher order than d=2! \n')
end

if nargin<10
    % Margin where it starts to influence the dynamic center
    marg_dynCenter = 1;
end

intersection_obs = unique(intersection_obs);

N_obs = size(obs,2);

% Center normalized vector to surface points
x_obs_norm = zeros(dim,size(x_obs_sf,2),N_obs);
x_obs_rad = zeros(size(x_obs_sf,2),N_obs);
rotMatrices = zeros(dim,dim,N_obs);
for ii = 1:N_obs
    x_obs_sf(:,:,ii) = x_obs_sf(:,:,ii)-obs{ii}.x0;
    x_obs_rad(:,ii) = sqrt(sum(x_obs_sf(:,:,ii).^2,1));
    x_obs_norm(:,:,ii) = x_obs_sf(:,:,ii)./repmat(x_obs_rad(:,ii)',dim,1);
    rotMatrices(:,:,ii) = compute_R(dim,obs{ii}.th_r);
end

% Calculate distance between obstacles
weight_obs_temp = zeros(N_obs,N_obs);
%x_middle = zeros(dim, N_obs, N_obs);
x_cenDyn_temp = zeros(dim, N_obs, N_obs);

% axis equal;
% %close all;
% figure;
% x_obs = drawEllipse_bound(obs{1}.a,obs{1}.th_r,obs{1}.p);
% plot(x_obs(1,:), x_obs(2,:), 'k'); 
% hold on; axis equal;
% x_obs = drawEllipse_bound(obs{2}.a,obs{2}.th_r,obs{2}.p,obs{2}.x0-obs{1}.x0);
% plot(x_obs(1,:), x_obs(2,:), 'k'); 

% Reference Distance
% TODO: only obstacles which are not intersection_obs
for ii = 1:N_obs
    % Rotation Matrix
    rotMat = rotMatrices(:,:,ii);
    
    % 2nd loop over all obstacles, but itself
%     obs2_list = 1:N_obs;
%     obs2_list(ii) = [];
    for jj = 1+ii:N_obs % only iterate over half the obstacles
        if and(ismember(jj, intersection_obs), ismember(ii,intersection_obs) )
            continue; % Don't reasign dynamic center if intersection exists
        end
        
        % Radius is used -> 2x for diameter
        ref_dist = marg_dynCenter*(max(obs{ii}.a)+max(obs{jj}.a));
        %ref_dist(ii,jj) = ref_dist(jj,ii); % symmetric

        % Inside consideration region
        % TODO - second power. Does another one work to?! it should...
        ind = sum(x_obs_sf(:,:,jj).^2,1) < (ref_dist+max(obs{ii}.a)+max(obs{jj}.a) )^2;

        if sum(ind) == 0
            continue; % Obstacle too far away
        end
        
        %x_obs_temp = x_obs_temp(:,ind);
        %N_inter = size(x_obs_temp,3);

        % Change increment step
        N_gamma = 3;
        dist_step = (ref_dist)/(N_gamma);
        dist_0 = 0;

        % Default value greater than 2 to enter loop
        n_intersection=3;

        % Iteration counter
        itCount = 0;
        itMax = 100;
        
        % Tries to find the distance to ellipse
        while(n_intersection>2)  % Continious while not a sharp intersection
            it_gamma0 = 1; % Iteration starting value
            for it_gamma = it_gamma0:N_gamma
                delta_d = dist_step*it_gamma+dist_0;

                % Search the intersection nummerically
                x_obs_temp = obs{jj}.x0 - obs{ii}.x0 + x_obs_norm(:,:,jj).* ...
                             (repmat(x_obs_rad(:,jj)'+ delta_d, dim, 1) );
                Gamma = sum( ( 1/obs{ii}.sf*rotMat'*(x_obs_temp )./ ...
                        repmat( (obs{ii}.a+delta_d) , 1, length(ind)) ).^(2*obs{ii}.p), 1);
                n_intersection = sum(Gamma<1);

%                 x_obs = drawEllipse_bound(obs{1}.a+delta_d,obs{1}.th_r,obs{1}.p);
%                     plot(x_obs(1,:), x_obs(2,:), '--'); 
%                     
%                     plot(x_obs_temp(1,:), x_obs_temp(2,:), '--'); 

                % Increment iteratoin counter
                itCount = itCount + 1;

                % Intersection found
                if n_intersection
                    dist_0 = dist_0 + (it_gamma-1)*dist_step;
                    dist_step = dist_step/(N_gamma);
                    
                    if n_intersection < 5
                        % Increse resolution of obstacle
                        intersection_ind = Gamma<1;
                        if n_intersction == 1
                            drawEllipse_bound(obs, 
                                x_obs_temp(:,intersection_ind-1),x_obs_temp(intersection_ind+1) );
                        else
                            drawEllipse_bound(a,theta,p,x0, N_resol, dim, x_start, x_end)
                            [(n_intersection_ind(1), n_intersection_ind(end)], );
                        end
                        break;
                    end
                    
                    if it_gamma == 0
                        % The increasing resolution caused new points,
                        % lower value of dist_0 is not bounding anymore
                        dist_0 = dist_0 - dist_step;
                        dist_step = dist_step + dist_step/N_gamma; %
                        n_intersection = 3;
                    end
                    
                    break;
                end

            end
            
            
            if (n_intersection == 0)
                break; % No intersection between the obstacles
            end
            
            % TODO -- Remove or change contidion
            if and(itCount > itMax*0.2, n_intersection < 4)
                break;% Loosen condition
            end

            if itCount > itMax
                warning('No close intersection found ...\n');
                break; % Emergency exiting -- in case of slow convergence
            end
        end

        % Negative step
        if(delta_d == 0)
            weight_obs_temp(ii,jj) = realmax;
        else
            weight_obs_temp(ii,jj) = max(1/delta_d -1/ref_dist,0);
        end
        weight_obs_temp(jj,ii) = weight_obs_temp(ii,jj);

        % Position the middle of shortest line connecting both obstacles
        x_middle = mean(x_obs_temp(:,Gamma<1),2);

%             x_close(:,it_obs2,it_obs1) = -x_close(:,it_obs1,it_obs2);
%             plot(x_close(1,ii,jj),x_close(2,ii,jj),'ok')

        % Desired Gamma in (0,1) to be on obstacle
        Gamma_dynCenter = max(1-delta_d/ref_dist,realmin);

        % Desired position of dynamic_center if only one obstacle existed
        Gamma_intersec = sum( ( (rotMat'*x_middle)./ ...
                                (obs{ii}.sf*obs{ii}.a)  ).^(2*obs{ii}.p),1);

        x_cenDyn_temp(:,ii,jj) = rotMat*(rotMat'*x_middle.*(ones(dim,1)*Gamma_dynCenter/Gamma_intersec).^(1./(2*obs{ii}.p))) ;

        % Desired position if only one obstacle exists 
        x_middle = x_middle + obs{ii}.x0 - obs{jj}.x0; % center around obs{jj}.x0
%             Gamma_intersec = sum( ( (x_middle) ./ ...
%                                     (obs{jj}.sf*obs{jj}.a)  ).^(2*obs{jj}.p), 1);
        Gamma_intersec = sum( ( (rotMatrices(:,:,jj)'*x_middle) ./ ...
                                (obs{jj}.sf*obs{jj}.a)  ).^(2*obs{jj}.p), 1);
        x_cenDyn_temp(:,jj,ii) = x_middle.*(ones(dim,1)*Gamma_dynCenter/Gamma_intersec).^(1./(2*obs{jj}.p)) ;

        % TODO remove after succesfull debugging
        Gamma_act = sum( ( 1/obs{ii}.sf*rotMat'*(x_cenDyn_temp(:,ii,jj) )./ ...
                          (obs{ii}.a)  ).^(2*obs{ii}.p), 1);
%             Gamma_dynCenter
%             dGamma = round((Gamma_act-Gamma_dynCenter)/sum([Gamma_act,Gamma_dynCenter])*2,4)
%             
%             if  dGamma 
%                 
%                 warning('Gamma not equal to Gamma desired. deltaGamma: %2.3f', dGamma)
%             end

    end
end

for ii =1:N_obs % Assign dynamic center 
    if ismember(ii,intersection_obs) 
        continue; % Don't reasign dynamic center if intersection exists
    end
    if sum(weight_obs_temp(ii,:))
        % Linear interpolation if at least one close obstacle --- MAYBE
        % change to nonlinear
        weight_obs = weight_obs_temp(:,ii)/ sum(weight_obs_temp(:,ii) );
        x_centDyn = squeeze(x_cenDyn_temp(:,ii,:));

        obs{ii}.x_center_dyn = sum(x_centDyn.*repmat(weight_obs',dim,1), 2) ...
                                    +obs{ii}.x0;
    else % default center otherwise
        obs{ii}.x_center_dyn = obs{ii}.x0;
    end
end

end

% function ang = angleSum(ang1, ang2)
% ang = ang1+ang2;
% if(ang>pi)
%     ang = ang -2*pi;
%     return;
% end
% if(ang<pi)
%     ang = ang +2*pi;
%     return;
% end
% 
% end

function x_obs = drawEllipse_bound(obs, x_bound, N_resol,  x0)
a = obs.a;
theta = obs.theta;
p = obs.p;

if nargin<3; N_resol = 27; end % 2^3
if nargin<4; x0 = obs.x0; end

dim = size(x0,1); % Dimension

R = compute_R(dim, theta);

if nargin<2
    dTheta = 2*pi/(N_resol);
    x_start = [1;0];
    x_end = [cos(dTehta); -sin(dTheta)];
    
    theta_min = 0;
else
    x_start = R'*x_bound(:,1);
    x_end = R'*x_bound(:,2);
    theta_min = atan2(x_start(2),x_start(1));
    theta_max= atan2(x_end(2),x_end(1));

    % increase if to low
    theta_max = (theta_max<theta_min)*theta_max+2*pi;
    
    dTheta = (theta_max-theta_min)/(N_resol-1);
end

x_obs = zeros(dim, N_resol);
for it_x = 1:N_resol
    theta = it_x*dTheta+theta_min;
    theta = theta-(theta>pi)*2*pi; % Range of [-pi, pi]
    
    x_obs(1,it_x) = a(1,:).*cos(theta);
    x_obs(2,it_x) = a(2,:).*sign(theta).*(1 - cos(theta).^(2.*p(1,:))).^(1./(2.*p(2,:)));
    x_obs(:,it_x) = R*x_obs(:,it_x);
end
x_obs = x_obs+x0;
end