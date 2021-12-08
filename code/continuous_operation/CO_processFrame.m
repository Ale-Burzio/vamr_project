function [S_i,T_wc_i,tracker_out] = CO_processFrame(img_i, img_i_prev, S_i_prev, tracker,K)
    

%% check to find new keypoints
    % if keyfranme
    % initializate point.Trck
    % initialize with fast/orb find new keypoints
    % Bidir_err = 1;
    % tracker = vision.PointTracker('MaxBidirectionalError',Bidir_err);
    % initialize(tracker,P2D_to_track,img_i_prev);
    % RANSAC vision.PointTracker
    % else:
    
    P2D_to_track = S_i_prev.keypoints';
    setPoints(tracker,P2D_to_track) ;
    [P2D_tracked,point_validity] = tracker(img_i);
    keys = P2D_tracked(point_validity,:)';
    tracker_out = tracker;
    P3D = S_i_prev.landmarks(:,point_validity);
    
    % RANSAC P3P
    [R_C_W, t_C_W] = ransacLocalization(keys, P3D, K);
    T_wc_i = [R_C_W, t_C_W];
    
    % keypoints (keypoints = 2xK) in img_i and 
    % 3D landmarks (P = 3xK) corrisponding to keypoints
    
    P3D = S_i_prev.landmarks(:,point_validity);
    S_i = struct('keypoints', keys, 'landmarks', P3D);
   
    
end

