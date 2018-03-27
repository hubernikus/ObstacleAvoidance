function [ intersection_sf, x_center_dyn, intersection_obs ] = obs_common_section( x_obs_sf )
%OBS_COMMON_SECTION finds common section of two ore more obstacles 
% at the moment only solution in two d is implemented

% Intersction surface
intersection_sf = [];
intersectionExists = false;
intersection_obs = [];


%[~, x_obs_sf] = obs_draw_ellipsoid(obs_list,50);
for it_obs1 = 1:size(x_obs_sf,3)
    for it_obs2 = it_obs1+1:size(x_obs_sf,3)
        
        if intersectionExists % Intersection already found
            n_iterx = size(intersection_sf,2)
        else
            n_iterx = size(x_obs_sf,2)
        end
        
        for ii = 1:n_iterx  % Over all points of an obstacle 1
            if size(intersection_sf,1) % Intersection already found
                x0 = intersection_sf(:,ii);
            else
                x0 = x_obs_sf(:,ii,it_obs1);
            end

            inside =  inside_convex_region(x0, x_obs_sf(:,:,it_obs2));
            
            if(inside)
%                 fig1 = figure('Position', [100,100,800,700]);
%                 plot(x_obs_sf(1,:,it_obs1), x_obs_sf(2,:,it_obs1), 'b.'); hold on;
%                 plot(x_obs_sf(1,:,it_obs2), x_obs_sf(2,:,it_obs2), 'k.');
                
                if(intersectionExists)
                    intersection_sf = create_convex_intersection(x0,{intersection_sf,x_obs_sf(:,:,it_obs2)},ii);
                    intersection_obs = [intersection_obs, it_obs2];
                else
                    intersection_sf = create_convex_intersection(x0,{x_obs_sf(:,:,it_obs1),x_obs_sf(:,:,it_obs2)}, ii);
                    intersection_obs = [it_obs1, it_obs2];
                end
                intersectionExists = true;
                break;
            end

            if false
                figure(10);
                plot([x_obs_sf(1,i0,obs),x_obs_sf(1,i1,obs)], ...
                     [x_obs_sf(2,i0,obs),x_obs_sf(2,i1,obs)], ...    
                      'r--')
                plot([x_obs_sf(1,i1,obs),x_obs_sf(1,i2,obs)], ...
                     [x_obs_sf(2,i1,obs),x_obs_sf(2,i2,obs)], ...    
                      'r--')
                plot(x0(1), x0(2), 'kx', 'Linewidth', 10);

                plot([x_obs_sf(1,i0,obs)],[x_obs_sf(2,i0,obs)],'gx'); hold on;
                plot([x_obs_sf(1,i0,obs)+vec1(1),x_obs_sf(1,i0,obs)],...
                     [x_obs_sf(2,i0,obs)+vec1(2),x_obs_sf(2,i0,obs)],'k--')
                plot([x_obs_sf(1,i0,obs)+perpDir1(1),x_obs_sf(1,i0,obs)],...
                     [x_obs_sf(2,i0,obs)+perpDir1(2),x_obs_sf(2,i0,obs)],'kx--')
                 plot([x_obs_sf(1,i0,obs)+testVec(1),x_obs_sf(1,i0,obs)],...
                     [x_obs_sf(2,i0,obs)+testVec(2),x_obs_sf(2,i0,obs)],'r')
                axis equal;
        %             xlim([-10,-4]); ylim([-3,3])
                pause()
                close(10);
            end

        end
    end
end

x_center_dyn = mean(intersection_sf,2);
end

function [concave_sf] = create_convex_intersection(x_temp, obs, concave_iter_new)
concave_sf = [x_temp];

% List of obstacles the concave_sf points belongs to 0 or 1
concave_obs_temp = 1; outside_obs_temp = 2;
concave_obs = [concave_obs_temp]; 
N_points = size(obs{concave_obs_temp},2); % Number of points 

% Remebers the point interator (of the origininal sf list)
concave_iterator = [concave_iter_new]; 

iterCount = 1;
iterMax = 100;
% Variable which decides the direction of the steps
stepDir = 1; % positve: +1, negative: -1, not decided: 0
% Variable which decides the direction search steps
searchDir = 0;% positve: +1, negative: -1, not decided: 0



while(true)
    % Next iteration step
    concave_iter_old = concave_iter_new;
    concave_iter_new  = mod(concave_iter_old + stepDir + searchDir -1 ,N_points) +1;
    
    
    
    % Check whether last point is reached
    while(norm(obs{concave_obs_temp}(:,concave_iter_new)-obs{concave_obs(end)}(:,concave_iterator(end)))==0 )
        %concave_iter_new  = concave_iter_new + stepDir + searchDir;
        %i0 = mod(N_points+i1-2,N_points)+1;
        if concave_iter_new == concave_iter_old
            break;
        end
        concave_iter_new  = mod(concave_iter_new + stepDir + searchDir -1 ,N_points) +1;

        iterCount = iterCount +1;
        if iterCount > iterMax
            warning('Maxiter reached \n');
            break;
        end
    end
    x_temp = obs{concave_obs_temp}(:,concave_iter_new);
%     plot(x_temp(1),x_temp(2),'go')
    
    if(iterCount>1 & concave_obs(1)==concave_obs_temp & ...
                concave_iterator(1)==concave_iter_new)
        % We made a full loop circle --- exit
        concave_sf = [concave_sf, x_temp];
        break;
    end
    
    
    % Check if another object is in that direction
    if(inside_convex_region(x_temp, obs{outside_obs_temp}))
        % Assign value to smalles surface list
        concave_sf = [concave_sf, x_temp];
        concave_obs = [concave_obs, concave_obs_temp];
        concave_iterator = [concave_iterator, concave_iter_new];
         
%         plot(x_temp(1),x_temp(2),'rx'); hold on;

        if searchDir
            % If it was in direction searching mode, 
            % set stepping direction fix.
            stepDir = searchDir;
            searchDir = 0;

        elseif not(stepDir)
            % Directionless searching found a point in concave region, 
            % but the right direction still needs to be found
            
            searchDir = 1;
            % stepDir = 0;
        end
    else
        
        if(stepDir) 
            % There is no point inside the concave region,
            % change obstacle and change direction search mode
            stepDir = 0;
            searchDir = 0;
            
            % Change obstacle
            outside_obs_temp = concave_obs_temp;
            concave_obs_temp = (concave_obs_temp==1) + 1;
            N_points = size(obs{concave_obs_temp},2);
            
            % Find closest point
            allDistances = sum((obs{concave_obs_temp}-repmat(x_temp,1,N_points)).^2,1);  
            [~, concave_iter_new] = min(allDistances);
           
        elseif searchDir == 0
            % The search mode was wrong
            concave_iter_temp = concave_iter_new;
            searchDir = 1;
            
        elseif searchDir == 1
            % One step in the wrong direction
            concave_iter_new = concave_iter_temp;

            searchDir = 0;
        elseif searchDir == -1
            warning('algorithm failed \n');
        end
    end
        
    iterCount = iterCount +1;
    if iterCount > iterMax
        warning('Maxiter reached \n')
        break;
    end

end


end


function [inside] =  inside_convex_region(x0, x_obs_sf)
% TODO: Change this intersection region finder 
% - only allow positive search direction
% - use convexitiy constraint to change obstacles.
% - add edges to convex hull

inside = false;

N_obs = size(x_obs_sf,2);
            
% all points of obstacle 2
allDistances = sum((x_obs_sf(:,:)-repmat(x0,1,N_obs)).^2,1);  

[~, i1] = min(allDistances);

            
% Indexes for the closest triangle
i0 = mod(N_obs+i1-2,N_obs)+1;
i2 = mod(N_obs+i1,N_obs)+1;

% Find direction of convexity
vec1 = x_obs_sf(:,i1)-x_obs_sf(:,i0);
while(norm(vec1) == 0) % twice the point in loop (last&first)
    i0 = i0 -1;
    vec1 = x_obs_sf(:,i1)-x_obs_sf(:,i0);
end

vec1 = vec1/norm(vec1);

perpDir1 = x_obs_sf(:,i2)-x_obs_sf(:,i0);
perpDir1 = perpDir1-(perpDir1'*vec1)*vec1;
perpDir1 = perpDir1/norm(perpDir1);

% Include other class
testVec = x0 -x_obs_sf(:,i0);

if(testVec'*perpDir1 > 0) % if on the convex side           
    vec2 = x_obs_sf(:,i2)-x_obs_sf(:,i1);
    while(norm(vec2) == 0) % twice the point in loop (last&first)
        i2 = i2 +1;
        vec2 = x_obs_sf(:,i2)-x_obs_sf(:,i1);
    end
    vec2 = vec2/norm(vec2);
    perpDir2 =  x_obs_sf(:,i0)-x_obs_sf(:,i1);
    perpDir2 = perpDir2-(perpDir2'*vec2)*vec2;
    perpDir2 = perpDir2/norm(perpDir2);

    testVec = x0 -x_obs_sf(:,i1);
    if(testVec'*perpDir2 > 0) % is in the convex region
        inside = true;
        return;
    end
end

end

