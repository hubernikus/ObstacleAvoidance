function [collision] = obs_check_collision(obs_list, x0)
% For the moment only implemented in 2D

%merge_obs = [];

collision = false; % ONLY for 2 objects

% figure; 
% plot(x_obs_sf(1,:,1),x_obs_sf(2,:,1),'k--'); hold on;
% plot(x_obs_sf(1,:,2),x_obs_sf(2,:,2),'k--')
[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs_list,50);

% Check wheter objects intersect -- ONLY 2D
for obs = 1:size(obs_list,3)
    %x_obs_sf(:,:,obs) =x_obs_sf_sing;
    
    
    N_obs = size(x_obs_sf,2);
    %for jj = 1:size(x_obs_sf(:,:,obs),2) % triplets of data
    %x_obs_sf(:,end,obs) = []; % remove last point, cause same as first
    
    allDistances = sum((x_obs_sf(:,:,obs)-repmat(x0,1,N_obs)).^2,1);    

    [~, i1] = min(allDistances);

    % Indexes for the closest triangle
    i0 = mod(N_obs+i1-2,N_obs)+1;
    i2 = mod(N_obs+i1,N_obs)+1;

    % Find direction of convexity
    vec1 = x_obs_sf(:,i1,obs)-x_obs_sf(:,i0,obs);
    while(norm(vec1) == 0) % twice the point in loop (last&first)
        i0 = i0 -1;
        vec1 = x_obs_sf(:,i1,obs)-x_obs_sf(:,i0,obs);
    end

    vec1 = vec1/norm(vec1);

    perpDir1 = x_obs_sf(:,i2,obs)-x_obs_sf(:,i0,obs);
    perpDir1 = perpDir1-(perpDir1'*vec1)*vec1;
    perpDir1 = perpDir1/norm(perpDir1);

    % Include other class
    testVec = x0 -x_obs_sf(:,i0,obs);

    if(testVec'*perpDir1 > 0) % if on the convex side           
        vec2 = x_obs_sf(:,i2,obs)-x_obs_sf(:,i1,obs);
        while(norm(vec2) == 0) % twice the point in loop (last&first)
            i2 = i2 +1;
            vec2 = x_obs_sf(:,i2,obs)-x_obs_sf(:,i1,obs);
        end
        vec2 = vec2/norm(vec2);
        perpDir2 =  x_obs_sf(:,i0,obs)-x_obs_sf(:,i1,obs);
        perpDir2 = perpDir2-(perpDir2'*vec2)*vec2;
        perpDir2 = perpDir2/norm(perpDir2);

        testVec = x0 -x_obs_sf(:,i1,obs);
        if(testVec'*perpDir2 > 0) % is in the convex region
            collision = true;
            
            
            if false
                plot([x_obs_sf(1,i0,obs),x_obs_sf(1,i1,obs)], ...
                     [x_obs_sf(2,i0,obs),x_obs_sf(2,i1,obs)], ...    
                      'r--')
                plot([x_obs_sf(1,i1,obs),x_obs_sf(1,i2,obs)], ...
                     [x_obs_sf(2,i1,obs),x_obs_sf(2,i2,obs)], ...    
                      'r--')
                plot(x0(1), x0(2), 'kx', 'Linewidth', 10);
                %plot(x_obs_sf(1,i1,obs),x_obs_sf(2,i1,obs),'xr')
                %plot(x_obs_sf(1,i2,obs),x_obs_sf(2,i2,obs),'xr')

                plot([x_obs_sf(1,i0,obs)],[x_obs_sf(2,i0,obs)],'gx'); hold on;
                %plot([x_obs_sf(1,i1,obs),x_obs_sf(1,i0,obs)],[x_obs_sf(2,i1,obs),x_obs_sf(2,i0,obs)],'b')
                %plot([x_obs_sf(1,i2,obs),x_obs_sf(1,i1,obs)],[x_obs_sf(2,i2,obs),x_obs_sf(2,i1,obs)],'b')
                plot([x_obs_sf(1,i0,obs)+vec1(1),x_obs_sf(1,i0,obs)],...
                     [x_obs_sf(2,i0,obs)+vec1(2),x_obs_sf(2,i0,obs)],'k--')
                plot([x_obs_sf(1,i0,obs)+perpDir1(1),x_obs_sf(1,i0,obs)],...
                     [x_obs_sf(2,i0,obs)+perpDir1(2),x_obs_sf(2,i0,obs)],'kx--')
                 plot([x_obs_sf(1,i0,obs)+testVec(1),x_obs_sf(1,i0,obs)],...
                     [x_obs_sf(2,i0,obs)+testVec(2),x_obs_sf(2,i0,obs)],'r')
                %axis equal;
        %             xlim([-10,-4]); ylim([-3,3])
            end
            
            break;
        end
    end

    %end
end

%close all;

%figure;
%subplot(1,2,1)
%plot(x_obs_sf(1,:,1),x_obs_sf(2,:,1),'bo-'); hold on;
%plot(x_obs_sf(1,1,1),x_obs_sf(2,1,1),'kx')
%plot(x_obs_sf(1,:,2),x_obs_sf(2,:,2),'ro-')
%plot(x_obs_sf(1,1,2),x_obs_sf(2,1,2),'kx-')



end