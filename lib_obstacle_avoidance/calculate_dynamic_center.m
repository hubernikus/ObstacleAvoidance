function [ obs ] = calculate_dynamic_center( obs, x_obs_sf, intersection_obs )
% Calculate the dynamic center for the applicaiton of the modulation
% matrix. A smooth transition is desired between objects far away (center
% in middle) and to convex obstacles intersecting. 
%
% This might not always be the completely optimal position of the
% dynamic_center of the obstacles, but it results in a continous movement
% of it and keeps computational cost low
%
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

% Resolution of outside plot
% MAYBE - Change to user fixed size -- replot first one
[~, x_obs_sf] = obs_draw_ellipsoid(obs,16); % Resolution # 16
N_resol = size(x_obs_sf,2); 

% Center normalized vector to surface points
x_obs_norm = zeros(dim,size(x_obs_sf,2),N_obs); % TODO - init the right size
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

axis equal;
%close all;
figure(12)%, 'Position', [300,600,400,400]);
x_obs = drawEllipse_bound(obs{1});
plot(x_obs(1,:), x_obs(2,:), 'k'); 
hold on; axis equal;
x_obs = drawEllipse_bound(obs{2});
plot(x_obs(1,:), x_obs(2,:), 'k'); 

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
        
        resolution = size(x_obs_sf,2)-1; % usually first point is double..
        %N_resol = 27; % 2^3
        resol_max = 3^7;
        
        % Default value greater than 2 to enter loop
        intersection_min = 2;
        n_intersection=intersection_min+1;
        intersection_ind = true(size(x_obs_norm,2),1);
        
%         if sum(abs(x_obs_temp(:,end) - x_obs_temp(:,1)))==0
%             % TODO - remove if statement -- only at beginning
%             x_obs_temp(:,end) =[]; % Remove repetition
%         end
        
        % Draw new obstacle
        obs_temp = obs{jj};
        obs_temp.a = obs_temp.a + dist_step+dist_0;
        obs_temp.x0 = obs_temp.x0 - obs{ii}.x0;
        x_obs_temp = drawEllipse_bound(obs_temp);

        itCount = 0; % Iteration counter
        itMax = 100; % Maximum number of iterations
        
        % Tries to find the distance to ellipse
        while(n_intersection>intersection_min)  % Continious while not a sharp intersection
            it_gamma0 = 1; % Iteration starting value
            for it_gamma = it_gamma0:N_gamma
                delta_d = dist_step*it_gamma+dist_0

                % Search the intersection nummerically
                %x_obs_temp = obs{jj}.x0 - obs{ii}.x0 + x_obs_norm(:,intersection_ind,jj).* ...
                %                (repmat(x_obs_rad(intersection_ind,jj)' + delta_d, dim,1) );
                
                %x_obs_temp = obs{jj}.x0 + x_obs_norm(:,intersection_ind,jj).* ...
                %                 (repmat(x_obs_rad(intersection_ind,jj)' + delta_d, dim,1) );
                             
%                 x_obs_temp = obs{jj}.x0 + x_obs_norm(:,intersection_ind,jj)*5;                                 (repmat(x_obs_rad(intersection_ind,jj)' + delta_d, dim,1) );

                %x_obs_temp = x_obs_sf(:,:,jj) + obs{ii}.x0;
                
                for kk = 1:size(x_obs_temp,2) % can loop be removed? one-liner -- remove rotation matrix
%                     Gamma(ii) = sum( ( 1/obs{ii}.sf*rotMat'*(x_obs_temp(:,ii) )./ ...
%                         repmat( (obs{ii}.a+delta_d) , 1, length(ind)) ).^(2*obs{ii}.p), 1);
                    Gamma(kk) = sum( ( 1/obs{ii}.sf*rotMat'*(x_obs_temp(:,kk) )./ ...
                         (obs{ii}.a+delta_d)).^(2*obs{ii}.p), 1);
                end
                intersection_ind_temp = Gamma<1;
                n_intersection = sum(intersection_ind_temp) %, x_range, N_resol);

                % Remove
                obs_temp = obs{ii};
                obs_temp.a = obs_temp.a + delta_d;
                obs_temp.x0 = [0;0];
                x_obs = drawEllipse_bound(obs_temp);
                plot(x_obs(1,:), x_obs(2,:), 'g.-'); hold on;
                    
                plot(x_obs_temp(1,:), x_obs_temp(2,:), 'r.-'); 

                % Increment iteratoin counter
                itCount = itCount + 1

                
                if n_intersection  % Intersection found
                    dist_0 = dist_0 + (it_gamma-1)*dist_step;
                    dist_step = dist_step/(N_gamma);
                    
                        if n_intersection == 1
                            % Increse resolution of obstacle


                            indLow = find(intersection_ind_temp,1)-1;
                            if indLow == 0;  indLow = size(x_obs_temp,2); end
                            indHigh = find(intersection_ind_temp,1,'last')+1;
                            if indHigh > size(x_obs_temp,2); indHigh = 1;  end

                            xRange = [x_obs_temp(:,indLow),x_obs_temp(:,indHigh)]-obs{ii}.x0;
                            % [x_obs_temp, x_obs_rad(:,jj), x_obs_norm(:,jj)] = drawEllipse_bound(obs{jj}, xRange, N_resol);
                            [x_obs_temp] = drawEllipse_bound(obs{jj}, xRange, N_resol);
                        elseif n_intersection < length(intersection_ind_temp)
                            % all intersection in middle -  [ 0 0 1 1 1 0 ] 
                            % extrema 1 - [ 0 0 0 1 1 1 ]
                            % extrema 2 - [ 1 1 1 0 0 0 ]
                            indLow = x_obs_temp(:,find(intersection_ind_temp,1, 'first') );
                            indHigh = x_obs_temp(:,find(intersection_ind_temp,1,'last') ) ;
                            
                            if ~ or(indHigh < length(intersection_ind_temp), indLow > 1)
                                % split at border - [ 1 1 0 0 0 1 ]
                                indHigh = x_obs_temp(:,find(~intersection_ind_temp,1, 'first') ) - 1;
                                if indHigh == 0;  indHigh = size(x_obs_temp,2); end
                                
                                indLow = x_obs_temp(:,find(~intersection_ind_temp,1,'last') ) + 1;
                                if indLow > size(x_obs_temp,2); indLow = 1;  end
                            end
                            xRange = [indLow, indHigh];
                            [x_obs_temp] = drawEllipse_bound(obs{jj}, xRange, N_resol);
                        end

                        % Resolution of the surface mapping - 2D
                        resolution = resolution/(n_intersection-1)*N_resol;

%                              %Reset intersection paramters
%                             n_intersection = intersection_min+1; % 
%                             intersection_ind = ones(size(intersection_ind));
%                             break;
%                         else 
%                             TODO more compact - transfer which index of
%                             intersection is kept 
%                             intersection_ind_new = (1:size(intersection_ind,1))';
%                             intersection_ind_new = intersection_ind_new(intersection_ind);
%                             intersection_ind = zeros(size(intersection_ind));
%                             intersection_ind(intersection_ind_new(intersection_ind_temp)) = 1;
%                         end
%                      end
                    
                    if it_gamma == 0
                        % The increasing resolution caused new points,
                        % lower value of dist_0 is not bounding anymore
                        dist_0 = dist_0 - dist_step;
                        dist_step = dist_step + dist_step/N_gamma; %
                        
                        n_intersection = intersection_min+1; % 
                    end
                    
                    %break;
                end
                if resolution > resol_max 
                    break; % Numerical resolution of boundary is high enough
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

function [x_obs, x_obs_rad, x_obs_norm] = drawEllipse_bound(obs, x_bound, N_resol,  x0)
a = obs.a;
th_r = obs.th_r;
p = obs.p;

if nargin<3; N_resol = 16; end % 2^3
if nargin<4; x0 = obs.x0; end

dim = size(x0,1); % Dimension

R = compute_R(dim, th_r);

if nargin<2
    theta_min = 0;
    dTheta = 2*pi/N_resol;
else
    x_start = R'*x_bound(:,1)-x0;
    x_end = R'*x_bound(:,2)-x0;
    theta_min = atan2(x_start(2),x_start(1));
    theta_max= atan2(x_end(2),x_end(1));

    % increase if to low
    theta_max = (theta_max<theta_min)*2*pi + theta_max;
    
    dTheta = (theta_max-theta_min)/(N_resol-1);
end

theta_list = []; % TODO -- delete this temporary list (only debugging)

x_obs = zeros(dim, N_resol);
for it_x = 1:N_resol
    
    theta = it_x*dTheta+theta_min;
    theta = theta-(theta>pi)*2*pi; % Range of [-pi, pi]
    theta_list = [theta_list, round(theta*180/pi)];
    
    x_obs(1,it_x) = a(1,:).*cos(theta);
    x_obs(2,it_x) = a(2,:).*sign(theta).*(1 - cos(theta).^(2.*p(1,:))).^(1./(2.*p(2,:)));
    x_obs(:,it_x) = R*x_obs(:,it_x);
end
x_obs = x_obs+x0;

theta_list

% Caclulate cylindric representation -- TODO - DELETE
x_obs_sf = x_obs - x0;
x_obs_rad= sqrt(sum(x_obs_sf.^2,1));
x_obs_norm = x_obs_sf./repmat(x_obs_rad,dim,1);

end