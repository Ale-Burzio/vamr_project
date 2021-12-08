function [S_i_prev,T_wc_i] = CO_processFrame(img_i, img_i_prev, S_i_prev, K)
    
%% check to find new keypoints
    
    % status prec
    P2D_to_track = S_i_prev.keypoints'; % Kx2
    P3D_prev = S_i_prev.landmarks'; 
    C_prev = S_i_prev.candidates';
    first_obser_prev = S_i_prev.first_obser'; % F
    
    % FIND NEW FEATURES
    corners1 = detectSURFFeatures(img_i_prev);
    N = 800;
    features = selectStrongest(corners1,N).Location;  
    
    % check if FEATURES are P or C = find new possible C
    PC = [P2D_to_track; C_prev];
    distances_features_PC = min(pdist2(features, PC),[],2);
    tresh_dis = 3;
    possible_C = features(distances_features_PC > tresh_dis, :);
    
    % track possible C
    pointTracker = vision.PointTracker('MaxBidirectionalError', 1);
    initialize(pointTracker,possible_C,img_i_prev);
    [new_C, validity_poss_C] = pointTracker(img_i);
    new_C = new_C(validity_poss_C,:);
    first_obser_new = possible_C(validity_poss_C,:);
    
    % track C
    if not(isempty(C_prev))
        pointTracker = vision.PointTracker('MaxBidirectionalError', 1);
        initialize(pointTracker,C_prev,img_i_prev);
        [C_prev_good, validity_C_prev] = pointTracker(img_i);
        C_prev_good = C_prev_good(validity_C_prev,:);
        first_obser_prev_good = first_obser_prev(validity_C_prev,:);
    else
        C_prev_good = [];
        first_obser_prev_good = [];
    end
    
    % find new P from C
    
    % track new P
    
    % track P
    pointTracker = vision.PointTracker('MaxBidirectionalError', 1);
    initialize(pointTracker,P2D_to_track,img_i_prev);
    [P2D_tracked, validity_P] = pointTracker(img_i);
    P2D_tracked = P2D_tracked(validity_P,:);
    P3D_prev_good = P3D_prev(validity_P,:);
    
    % RANSAC P3P
    [R_C_W, t_C_W, inliers_mask] = ransacLocalization(P2D_tracked', P3D_prev_good', K);
    T_wc_i = [R_C_W, t_C_W];
    P2D_tracked = P2D_tracked(inliers_mask, :);
    P3D_prev_good = P3D_prev_good(inliers_mask, :);
    
    % set new status
    S_i_prev.candidates = [C_prev_good; new_C]';
    S_i_prev.first_obser = [first_obser_prev_good; first_obser_new]';
    S_i_prev.keypoints = P2D_tracked';
    S_i_prev.landmarks = P3D_prev_good';
    
end

