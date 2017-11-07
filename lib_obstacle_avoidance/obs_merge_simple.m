function [merge_obs] = obs_merge_simple(x_obs_sf)
% For the moment only implemented in 2D

fprintf('obs_merge_simple entered \n')
%merge_obs = [];

objMerge = false; % ONLY for 2 objects

testVecCount = zeros(size(x_obs_sf,2),1);

% figure; 
% plot(x_obs_sf(1,:,1),x_obs_sf(2,:,1),'k--'); hold on;
% plot(x_obs_sf(1,:,2),x_obs_sf(2,:,2),'k--')

% Check wheter objects intersect -- ONLY 2D
for obs = 1:size(x_obs_sf,3)-1
    for obsRest = obs+1:size(x_obs_sf,3)
        N_obs = size(x_obs_sf,2);
        for jj = 1:size(x_obs_sf(:,:,obs),2) % triplets of data
            allDistances = sum((x_obs_sf(:,:,obs)-repmat(x_obs_sf(:,jj,obsRest),1,N_obs)).^2,1);    
            
            [~, i1] = min(allDistances);
            
            % Indexes for the closest triangle
            i0 = mod(N_obs+i1-2,N_obs)+1;
            i2 = mod(N_obs+i1,N_obs)+1;
            
            % Find direction of convexity
            vec1 = x_obs_sf(:,i1,obs)-x_obs_sf(:,i0,obs);
            if(norm(vec1) == 0) % twice the point in loop (last&first)
                i0 = i0 -1;
                vec1 = x_obs_sf(:,i1,obs)-x_obs_sf(:,i0,obs);
            end
            
            vec1 = vec1/norm(vec1);
  
            perpDir1 = x_obs_sf(:,i2,obs)-x_obs_sf(:,i0,obs);
            perpDir1 = perpDir1-perpDir1'*vec1*vec1;
            perpDir1 = perpDir1/norm(perpDir1);
            
            % Include other class
            testVec = x_obs_sf(:,jj,obsRest) -x_obs_sf(:,i0,obs);
            
%             plot([x_obs_sf(1,i0,obs)],[x_obs_sf(2,i0,obs)],'gx'); hold on;
%             plot([x_obs_sf(1,i1,obs),x_obs_sf(1,i0,obs)],[x_obs_sf(2,i1,obs),x_obs_sf(2,i0,obs)],'b')
%             plot([x_obs_sf(1,i2,obs),x_obs_sf(1,i1,obs)],[x_obs_sf(2,i2,obs),x_obs_sf(2,i1,obs)],'b')
%             plot([x_obs_sf(1,i0,obs)+vec1(1),x_obs_sf(1,i0,obs)],...
%                  [x_obs_sf(2,i0,obs)+vec1(2),x_obs_sf(2,i0,obs)],'k--')
%             plot([x_obs_sf(1,i0,obs)+perpDir1(1),x_obs_sf(1,i0,obs)],...
%                  [x_obs_sf(2,i0,obs)+perpDir1(2),x_obs_sf(2,i0,obs)],'kx--')
%              plot([x_obs_sf(1,i0,obs)+testVec(1),x_obs_sf(1,i0,obs)],...
%                  [x_obs_sf(2,i0,obs)+testVec(2),x_obs_sf(2,i0,obs)],'r')
%             axis equal;
% %             xlim([-10,-4]); ylim([-3,3])
%             a=1
%             
            
            if(testVec'*perpDir1 > 0) % if on the convex side            
                vec2 = x_obs_sf(:,i2,obs)-x_obs_sf(:,i1,obs);
                if(norm(vec2) == 0) % twice the point in loop (last&first)
                    i2 = i2 +1;
                    vec2 = x_obs_sf(:,i2,obs)-x_obs_sf(:,i1,obs);
                end
                vec2 = vec2/norm(vec2);
                perpDir2 =  x_obs_sf(:,i0,obs)-x_obs_sf(:,i1,obs);
                perpDir2 = perpDir2-perpDir2'*vec2;
                perpDir2 = perpDir2/norm(perpDir2);
                
                testVec = -x_obs_sf(:,jj,obsRest) -x_obs_sf(:,i1,obs);
                if(testVec'*perpDir2 > 0) % is in the convex region
                    objMerge = true;
                    break;
                end
            end
            
        end
    end
end

merge_obs = []

%close all;

%figure;
%subplot(1,2,1)
%plot(x_obs_sf(1,:,1),x_obs_sf(2,:,1),'bo-'); hold on;
%plot(x_obs_sf(1,1,1),x_obs_sf(2,1,1),'kx')
%plot(x_obs_sf(1,:,2),x_obs_sf(2,:,2),'ro-')
%plot(x_obs_sf(1,1,2),x_obs_sf(2,1,2),'kx-')

if objMerge 
    fprintf('Mergin objects \n'); 
  
    obs = 1;
    obsRest = 2;
    merge_obs1 = convexityEnforcement(x_obs_sf, obs, obsRest);
    
    
    obs = 2;
    obsRest = 1;
    merge_obs2 = convexityEnforcement(x_obs_sf, obs, obsRest);

    merge_obs = [merge_obs1,merge_obs2];

    %subplot(1,2,2)
    %plot(merge_obs1(1,:),merge_obs1(2,:),'o-'); hold on;
    %plot(merge_obs2(1,:),merge_obs2(2,:),'o-')
    
    N_merge = size(merge_obs,2);
    % Calculate geometric center
    midPoint = sum(merge_obs,2)/N_merge;

    % Calculate the angle of each object
    centerDir = merge_obs - repmat(midPoint,1,N_merge);

    % Get angle
    angleMerge = atan2(centerDir(2,:),centerDir(1,:));

    % Sort
    [~, indMerge] = sort(angleMerge);

    merge_obs = merge_obs(:,[indMerge,1]);
    %plot(merge_obs(1,:),merge_obs(2,:),'k--')

end


end

function [merge_obs]= convexityEnforcement(x_obs_sf, obs, obsRest)
% Function removes all points of obs, which are concave in repsect to obsRest
    merge_obs = []; 
    
    N_boundary = size(x_obs_sf,2);
    
    % Check whether current point is in convex configuration
    firstConvCheck = true;
    convexConsCheck = true;
    convexPrevCheck = true;
    
    % Backwards comparison of first point
    vec1 = x_obs_sf(:,1,obs)-x_obs_sf(:,N_boundary-1,obs);  
    vec1 = vec1/norm(vec1);
    
    % Perpendicular vector (decide direction of convexity)
    perpDir1 = x_obs_sf(:,2,obs)-x_obs_sf(:,N_boundary-1,obs);
    perpDir1 = perpDir1-(perpDir1'*vec1)*vec1;
    perpDir1 = perpDir1/norm(perpDir1);
    
    % Check wheter all points of the other cluster lie in the convexity
    % direction. Otherwise notify the point (false).
    for jj = 1:size(x_obs_sf(:,:,obsRest),2) 
        testVec = x_obs_sf(:,jj,obsRest) -x_obs_sf(:,N_boundary-1,obs);
        if(testVec'*perpDir1 < 0)
            convexPrevCheck = false;
            firstConvCheck = false;
            break;
        end
    end
    
    for i1 = 2:size(x_obs_sf(:,:,obs),2)-1 % triplets of data
        % Indexes for the closest triangle
        i0 = i1-1;
        i2 = i1+1;

        % Find direction of convexity
        vec1 = x_obs_sf(:,i1,obs)-x_obs_sf(:,i0,obs);
        vec1 = vec1/norm(vec1);
        
        perpDir1 = x_obs_sf(:,i2,obs)-x_obs_sf(:,i0,obs);
        perpDir1 = perpDir1-(perpDir1'*vec1)*vec1;
        perpDir1 = perpDir1/norm(perpDir1);
    

        for jj = 1:size(x_obs_sf(:,:,obsRest),2)
            testVec = x_obs_sf(:,jj,obsRest) -x_obs_sf(:,i0,obs);
            if(testVec'*perpDir1 < 0)
                convexConsCheck = false;
                break;
            end
        end
        
        if(or(convexPrevCheck, convexConsCheck)) % keep value if its part of outer most tangents
            merge_obs(:,end+1) = [x_obs_sf(:,i0,obs)];
            %plot(x_obs_sf(1,i0,obs),x_obs_sf(2,i0,obs),'gx')
        end
        convexPrevCheck = convexConsCheck;
        convexConsCheck = true;
    end
    if(or(firstConvCheck, convexPrevCheck)) % Evaluate 1st/last point (same)
        merge_obs(:,end+1) = [x_obs_sf(:,i1,obs)];
        %plot(x_obs_sf(1,i1,obs),x_obs_sf(2,i1,obs),'rx')
    end
end