function [S_i,T_i_wc] = Copy_of_CO_processFrame(img_i, img_i_prev, S_i_prev, K)

    %% status prev
    P2D_to_track = S_i_prev.keypoints'; % P = Kx2
    P3D_to_track = S_i_prev.landmarks'; % X = Kx3
    C_prev = S_i_prev.candidates'; % C = Mx2
    first_obser_prev = S_i_prev.first_obser'; % F = Mx2
    cam_pos_first_obser_prev = S_i_prev.cam_pos_first_obser'; % T = Mx12
    
    %% find new FEATURES
    corners1 = detectHarrisFeatures(img_i, 'MinQuality', 0.000001);
    N = 800;
    features = selectStrongest(corners1,N).Location;
    
    %% track possible C
    pointTrackerC = vision.PointTracker('MaxBidirectionalError', 1);
    initialize(pointTrackerC,features,img_i);
    [features_matched, validity_poss_C] = pointTrackerC(img_i_prev);
    features_matched = features_matched(validity_poss_C,:);
    release(pointTrackerC);

    %% check if FEATURES are P or C = find new possible C
    PC = [P2D_to_track; C_prev];
    distances_features_PC = min(pdist2(features_matched, PC),[],2);
    tresh_dis = 5;
    first_obser_new = features_matched(distances_features_PC > tresh_dis, :);
    new_C = features(distances_features_PC > tresh_dis, :);
    % cam_pos_first_obser_new now is initialized and assigned later (after p3p)
    cam_pos_first_obser_new = ones(size(first_obser_new,1),1);
    
    %% track old C
    if not(isempty(C_prev))
        pointTrackerCold = vision.PointTracker('MaxBidirectionalError', 1);
        initialize(pointTrackerCold,C_prev,img_i_prev);
        [C_prev_good, validity_C_prev] = pointTrackerCold(img_i);
        
        C_prev_good = C_prev_good(validity_C_prev,:);
        first_obser_prev_good = first_obser_prev(validity_C_prev,:);
        cam_pos_first_obser_prev = cam_pos_first_obser_prev(validity_C_prev,:);
        release(pointTrackerCold);
    else
        C_prev_good = [];
        first_obser_prev_good = [];
        cam_pos_first_obser_prev = [];
    end
    
    %% track P
    pointTrackerP = vision.PointTracker('MaxBidirectionalError', 1);
    initialize(pointTrackerP,P2D_to_track,img_i_prev);
    [P2D_tracked, validity_P] = pointTrackerP(img_i);
    release(pointTrackerP);
    
    P2D_tracked = P2D_tracked(validity_P,:);
    P2D_to_track = P2D_to_track(validity_P,:);
    P3D_prev_good = P3D_to_track(validity_P,:);
    
    %figure(11);
    %showMatchedFeatures(img_i_prev, img_i, P2D_to_track, P2D_tracked)
    
    %% RANSAC P3P
    [R_C_W, t_C_W, inliers_mask, ~, ~] = ransacLocalization(P2D_tracked', P3D_prev_good', K);
    T_i_wc = [R_C_W, t_C_W];
    P2D_tracked = P2D_tracked(inliers_mask, :);
    P2D_to_track = P2D_to_track(inliers_mask,:);
    P3D_prev_good = P3D_prev_good(inliers_mask, :);
    % cam_pos_first_obser_new now is setted
    cam_pos_first_obser_new = cam_pos_first_obser_new .* reshape(T_i_wc, [1, 12]);
    
    %% PLOT 
    figure(3)
    imshow(img_i);
    p1 = P2D_tracked';
    p2 = P2D_to_track';
    hold on;
    plot(p1(1,:), p1(2,:), 'bs');
    hold on;
    plot(p2(1,:), p2(2,:), 'rs');
    hold on;
    plot([p1(1,:);p2(1,:)], [p1(2,:);p2(2,:)], 'g-', 'Linewidth', 2);
    pause()
    close(3)
    
    %% find new P from C
    C_tot = [C_prev_good; new_C];
    first_obser_tot = [first_obser_prev_good; first_obser_new];
    cam_pos_first_obser_tot = [cam_pos_first_obser_prev; cam_pos_first_obser_new];
    angles = C_F_angle(C_tot, first_obser_tot, T_i_wc, cam_pos_first_obser_tot, K);
    tresh_angle = 10 * pi / 180;
    validity_angle = angles > tresh_angle;
    
    %% ad new P to old P (P2D_tracked) 
    P2D_new = C_tot(validity_angle,:);
    P2D_tot = [P2D_tracked; P2D_new];
    
    %% triangulate new P (find X)
    first_obser_P2D_new = first_obser_tot(validity_angle,:);
    cam_pos_first_obser_P2D_new = cam_pos_first_obser_tot(validity_angle,:); 
    cameraParams = cameraParameters("IntrinsicMatrix",K');
    P3D_new = zeros(size(P2D_new,1),3);
    
    for k = 1:size(P2D_new,1)
        %%%%%%%%%%%%%%%%%%%%%%%%%% CHECK ERROR ON ROTO-TRASLATION BETWEEN
        %%%%%%%%%%%%%%%%%%%%%%%%%% P1 and P2 TO TRIANGULATE %%%%%%%%%%%%%
        T_first_wc = reshape(cam_pos_first_obser_P2D_new(k,:),[4,3])';
        Rot_C_first = R_C_W * T_first_wc(:,1:3)';
        Trasl_C_first = T_first_wc(:,4) - t_C_W;
        stereoParams = stereoParameters(cameraParams, cameraParams, Rot_C_first, Trasl_C_first); 
        p1 = P2D_new(k,:);
        p2 = first_obser_P2D_new(k,:);
        P3D_new(k,:) = triangulate(p1,p2,stereoParams);
    end
    P3D_tot = [P3D_prev_good; P3D_new]; 
    
    %% remove new P from C, F, T
    C_tot = C_tot(not(validity_angle),:);
    first_obser_tot = first_obser_tot(not(validity_angle),:);
    cam_pos_first_obser_tot = cam_pos_first_obser_tot(not(validity_angle),:);
    
    %% set new status
    S_i = S_i_prev;
    S_i.keypoints = P2D_tot'; % P = 2xK
    S_i.landmarks = P3D_tot'; % X = 3xK
    S_i.candidates = C_tot'; % C = 2xM
    S_i.first_obser = first_obser_tot'; % F = 2xM
    S_i.cam_pos_first_obser = cam_pos_first_obser_tot'; % T = 12xM
    
end