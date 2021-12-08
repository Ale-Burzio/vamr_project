function [R_C_W, t_C_W] ...
    = ransacLocalization(matched_query_keypoints, corresponding_landmarks, K)
% query_keypoints should be 2x1000
% all_matches should be 1x1000 and correspond to the output from the
%   matchDescriptors() function from exercise 3.
% best_inlier_mask should be 1xnum_matched (!!!) and contain, only for the
%   matched keypoints (!!!), 0 if the match is an outlier, 1 otherwise.
   
    s=3; %we need 6 points for P3P
    max_dist=10; %threshold is 10 pixels
    num_iteration_history = [0];
    num_matched=length(corresponding_landmarks);
    
    max_num_inliers_history=zeros(1, 2000); %initialize inliers
    i=1;
    while i<=max(1, num_iteration_history(i))
        if i~=1
            max_num_inliers_history(i)=max_num_inliers_history(i-1);
        end
            
        matched_query_keypoints = flip(matched_query_keypoints);
        [points, idx] = datasample(matched_query_keypoints, s, 2, 'Replace', false); %select random points
        
        points=K \ [points; ones(1, s)];
        points=normc(points);
        
        %find M, R, T from the 6 random points with DLT
        M_stacked = real(p3p(corresponding_landmarks(:, idx), points));
        for j=1:4
            M_tilde=M_stacked(:, (4*j-3):(4*j));
            R=M_tilde(:, 2:4);
            T=M_tilde(:, 1);
            M_tilde=[R, T; 0 0 0 1]^(-1);
            M_tilde=M_tilde(1:3, :);
        
            %find the reprojection of 3D points to the image plane
            projected_points = reprojectPoints(corresponding_landmarks', M_tilde, K)';
        
            %compute the diference from the matched points to the reprojections
            diff=sqrt(sum((projected_points - matched_query_keypoints).^ 2));


            %distance from inliers to reproj. has to be smaller than 10 pixels
            num_inliers = length(find(diff<=max_dist));
        
            if  num_inliers > max_num_inliers_history(i)
                max_num_inliers_history(i)=num_inliers;
                best_inlier_mask = diff <= max_dist;
                R_C_W=R;
                t_C_W=T;
            end
        end
        
        epsilon=1-max_num_inliers_history(i)/num_matched;
        num_iteration_history=[num_iteration_history, log(1-0.99)/log(1-(1-epsilon)^s)];
        i=i+1;
        
    end
end
    
    
