function [S_i,T_wc_i] = CO_processFrame(img_i, img_i_prev, S_i_prev)

    % RANSAC vision.PointTracker
    [points,point_validity] = pointTracker(img_i);
    setPoints(pointTracker,points,point_validity) ;
    
    % RANSAC P3P
    T_wc_i = ;
    
    % keypoints (keypoints = 2xK) in img_i and 
    % 3D landmarks (P = 3xK) corrisponding to keypoints
    S_i = struct(keypoints, keypoints, landmarks, P);
    
end

