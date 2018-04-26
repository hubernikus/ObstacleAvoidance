function [obs, intersection_obs ] = obs_common_section( obs, x_obs_sf )
%OBS_COMMON_SECTION finds common section of two ore more obstacles 
% at the moment only solution in two d is implemented

% Intersction surface
intersection_sf = []; x_center_dyn = []; intersection_obs = [];
intersectionExists = false;

% No intersection region 
if(size(obs,2) == 1)
    return;
end

if nargin < 2 % draw obstacles
    [~, x_obs_sf] = obs_draw_ellipsoid(obs,50);
end

% Extend for more dimensions
d = 2;

N_points = size(x_obs_sf,2);
Gamma_steps = 15;

% figure;
% for ii = 1:size(x_obs_sf,3)
%     plot(x_obs_sf(1,:,ii),x_obs_sf(2,:,ii),'b.'); hold on;
% end

for it_obs1 = 1:size(x_obs_sf,3)
    for it_obs2 = it_obs1+1:size(x_obs_sf,3)
        
        if intersectionExists % Modify intersecition part
            obsCloseBy = true;
            
            % Check dimensions before starting expensive calculation
            for ii = intersection_obs
                if norm(obs{ii}.x0 - obs{it_obs2}.x0) > max(obs{ii}.a) + max(obs{it_obs2}.a)
                    % Obstacles to far apart
                    obsCloseBy = false;     
                    break;
                end
            end
            
            if obsCloseBy
                N_inter = size(intersection_sf,2); % Number of intersection points

                R = compute_R(d,obs{it_obs2}.th_r);
                Gamma = sum( ( 1/obs{it_obs2}.sf*R'*(intersection_sf(:,:)-repmat(obs{it_obs2}.x0,1,N_inter) )./repmat(obs{it_obs2}.a, 1, N_inter) ).^(2*obs{it_obs2}.p),1);

                ind = Gamma<1;
                if sum(ind) 
                    intersection_sf = intersection_sf(:,ind);
                    intersection_obs = [intersection_obs, it_obs2];
                end
            end
            
        else
            % Check dimensions before starting expensive calculation
            if norm(obs{it_obs1}.x0 - obs{it_obs2}.x0) < max(obs{it_obs1}.a) + max(obs{it_obs2}.a)
                % Obstacles are close enough
                
                % get all points of obs2 in obs1
                
                R = compute_R(d,obs{it_obs1}.th_r);
                % \Gamma = \sum_{i=1}^d (xt_i/a_i)^(2p_i) = 1
                Gamma = sum( ( 1/obs{it_obs1}.sf*R'*(x_obs_sf(:,:,it_obs2)-repmat(obs{it_obs1}.x0,1,N_points) )./repmat(obs{it_obs1}.a, 1, N_points) ).^(2*obs{it_obs1}.p),1);
                
                intersection_sf = [intersection_sf,x_obs_sf(:,Gamma<1,it_obs2)];
                
                % Get all poinst of obs1 in obs2
                R = compute_R(d,obs{it_obs2}.th_r);
                Gamma = sum( ( 1/obs{it_obs2}.sf*R'*(x_obs_sf(:,:,it_obs1)-repmat(obs{it_obs2}.x0,1,N_points) )./repmat(obs{it_obs2}.a, 1, N_points) ).^(2*obs{it_obs2}.p),1);

                intersection_sf = [intersection_sf,x_obs_sf(:,Gamma<1,it_obs1)];

                if size(intersection_sf)>0
                    intersectionExists = true;
                    intersection_obs = [it_obs1,it_obs2];
                    
                    % Create more intersection points
                    obs_interior = [];
                    obs_interior{1} = obs{it_obs1};
                    
                    % Increase resolution by sampling points within
                    % obstacle, too
                    for ii = 1:Gamma_steps-1
                        N_points_interior = ceil(N_points/Gamma_steps*ii);
                        obs_interior{it_obs1}.a = obs{it_obs1}.a/Gamma_steps*ii;
                        
                        [x_obs_sf_interior, ~] = obs_draw_ellipsoid(obs_interior, N_points_interior);
                        
                        Gamma = sum( ( 1/obs{it_obs2}.sf*R'*(x_obs_sf_interior(:,:,1)-repmat(obs{it_obs2}.x0,1,N_points_interior) )./repmat(obs{it_obs2}.a, 1, N_points_interior) ).^(2*obs{it_obs2}.p),1);
    
                        intersection_sf = [intersection_sf,x_obs_sf_interior(:,Gamma<1,1)];
                    end
                    % Check center piont too
                    Gamma = sum( ( 1/obs{it_obs2}.sf*R'*(obs_interior{it_obs1}.x0-obs{it_obs2}.x0 )./obs{it_obs2}.a).^(2*obs{it_obs2}.p),1);
                    
                    if Gamma<1
                        intersection_sf = [intersection_sf,obs_interior{it_obs1}.x0];
                    end
                    
                end
            end
        end        

    end
    if intersectionExists; break; end
end

if size(intersection_sf,2) == 0 
    return;
end

%plot(intersection_sf(1,:), intersection_sf(2,:));
% Remove doubles
intersection_sf = unique(intersection_sf','rows')';
%plot(intersection_sf(1,:), intersection_sf(2,:));

% Get numerical mean
x_center_dyn= mean(intersection_sf,2);
for it_obs = intersection_obs
    obs{it_obs}.x_center_dyn = x_center_dyn;
end

% sort points according to angle
intersec_sf_cent = intersection_sf - repmat(x_center_dyn,1,size(intersection_sf,2));


% TODO - replace atan2 for speed
[~, ind] = sort( atan2(intersec_sf_cent(2,:), intersec_sf_cent(1,:)));

intersection_sf = intersection_sf(:, ind);
intersection_sf = [intersection_sf, intersection_sf(:,1)];

intersection_obs = [1:size(obs,2)];
end