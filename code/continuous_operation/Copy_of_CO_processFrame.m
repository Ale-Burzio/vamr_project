function [S_i,T_i_wc] = Copy_of_CO_processFrame(img_i, img_i_prev, S_i_prev, K, i)

    %% status prev
    P2D_to_track = S_i_prev.keypoints'; % P = Kx2
    P3D_to_track = S_i_prev.landmarks'; % X = Kx3
    C_prev = S_i_prev.candidates'; % C = Mx2
    first_obser_prev = S_i_prev.first_obser'; % F = Mx2
    cam_pos_first_obser_prev = S_i_prev.cam_pos_first_obser'; % T = Mx12
    
    %% find new FEATURES
    corners1 = detectFASTFeatures(img_i);
    N = 400;
    features = selectStrongest(corners1,N).Location;
    
    %% track possible C
    pointTrackerC = vision.PointTracker('MaxBidirectionalError', 0.1);
    initialize(pointTrackerC,features,img_i);
    [features_matched, validity_poss_C] = pointTrackerC(img_i_prev);
    features_matched = round(features_matched(validity_poss_C,:));
    features = features(validity_poss_C, :);
    release(pointTrackerC);
    
    distances = vecnorm(features-features_matched,2,2);
    features = features(distances > 3,:);
    features_matched = features_matched(distances > 3,:);

    %% check if FEATURES are P or C = find new possible C
    PC = [P2D_to_track; C_prev];
    distances_features_PC = min(pdist2(features_matched, PC),[],2);
    tresh_dis = 4;
    first_obser_new = features_matched(distances_features_PC > tresh_dis, :);
    new_C = features(distances_features_PC > tresh_dis, :);
    first_obser_new = new_C;
    
    %% track old C
    if not(isempty(C_prev))
        pointTrackerCold = vision.PointTracker('MaxBidirectionalError', 0.1);
        initialize(pointTrackerCold,C_prev,img_i_prev);
        [C_prev_good, validity_C_prev] = pointTrackerCold(img_i);
        
        C_prev_good = round(C_prev_good(validity_C_prev,:));
        first_obser_prev_good = first_obser_prev(validity_C_prev,:);
        cam_pos_first_obser_prev = cam_pos_first_obser_prev(validity_C_prev,:);
        release(pointTrackerCold);
    else
        C_prev_good = [];
        first_obser_prev_good = [];
        cam_pos_first_obser_prev = [];
    end
    
    %% track P
    pointTrackerP = vision.PointTracker('MaxBidirectionalError', 0.1);
    initialize(pointTrackerP,P2D_to_track,img_i_prev);
    [P2D_tracked, validity_P] = pointTrackerP(img_i);
    release(pointTrackerP);
    
    P2D_tracked = round(P2D_tracked(validity_P,:));
    P2D_to_track = P2D_to_track(validity_P,:);
    P3D_prev_good = P3D_to_track(validity_P,:);
    
    %figure(11);
    %showMatchedFeatures(img_i_prev, img_i, P2D_to_track, P2D_tracked)
    
    %% RANSAC P3P
%     [R_C_W, t_C_W, inliers_mask, ~, ~] = ransacLocalization(P2D_tracked', P3D_prev_good', K);
%      R_C_W = eye(3);
%      t_C_W = [-1;0;0]*i;

    % test with matlab function
    cameraParams = cameraParameters("IntrinsicMatrix",K');
    [R_C_W,t_C_W, inliers_mask] = estimateWorldCameraPose(P2D_tracked,P3D_prev_good,cameraParams, 'Confidence', 99, 'MaxNumTrials', 3000);
    R_C_W = R_C_W';
    t_C_W = - R_C_W'* t_C_W';
    
    T_i_wc = [R_C_W, t_C_W];
    P2D_tracked = P2D_tracked(inliers_mask, :);
    P2D_to_track = P2D_to_track(inliers_mask,:);
    P3D_prev_good = P3D_prev_good(inliers_mask, :);
    % cam_pos_first_obser_new now is setted
    cam_pos_first_obser_new = repmat(reshape(T_i_wc, [1, 12]),[size(new_C,1),1]);
    
    
    
    %% find new P from C
    C_tot = [C_prev_good; new_C];
    first_obser_tot = [first_obser_prev_good; first_obser_new];
    cam_pos_first_obser_tot = [cam_pos_first_obser_prev; cam_pos_first_obser_new];
    angles = C_F_angle(C_tot, first_obser_tot, T_i_wc, cam_pos_first_obser_tot, K);
    tresh_angle = 5 * pi / 180;
    validity_angle = abs(angles) > tresh_angle;
    
    %% ad new P to old P (P2D_tracked) 
    P2D_new = C_tot(validity_angle,:);
    
    %% triangulate new P (find X)
    first_obser_P2D_new = first_obser_tot(validity_angle,:);
    cam_pos_first_obser_P2D_new = cam_pos_first_obser_tot(validity_angle,:); 
    cameraParams = cameraParameters("IntrinsicMatrix",K');
    P3D_new = zeros(size(P2D_new,1),3);
    
    for k = 1:size(P2D_new,1)
        %%%%%%%%%%%%%%%%%%%%%%%%%% CHECK ERROR ON ROTO-TRASLATION BETWEEN
        %%%%%%%%%%%%%%%%%%%%%%%%%% P1 and P2 TO TRIANGULATE %%%%%%%%%%%%%
        T_first_w = reshape(cam_pos_first_obser_P2D_new(k,:),[3,4]);
        R_F = T_first_w(:,1:3);
        t_F = T_first_w(:,4);

        M1 = cameraMatrix(cameraParams, R_F, t_F);
        M2 = cameraMatrix(cameraParams, R_C_W, t_C_W);
        
        p1 = first_obser_P2D_new(k,:);
        p2 = P2D_new(k,:);
        
      
        P3D_new(k,:) = triangulate(p1,p2,M1,M2);
    end
    
%     [nme,~] = size(P3D_new);
%     v = ones(nme,1);
%     for i = 1:nme
%        if P3D_new(i,3) <=0 || P3D_new(i,3) > 200
%            v(i)=0;
%        end
%     end
%     
%     P2D_new = P2D_new(v>0,:);
%     P3D_new = P3D_new(v>0,:);
    
    P2D_tot = [P2D_tracked; P2D_new];
    P3D_tot = [P3D_prev_good; P3D_new]; 
    
    %% remove new P from C, F, T
    C_tot = C_tot(not(validity_angle),:);
    first_obser_tot = first_obser_tot(not(validity_angle),:);
    cam_pos_first_obser_tot = cam_pos_first_obser_tot(not(validity_angle),:);
    
    %% PLOT 
    figure(3),
    subplot(1,2,1),
    imshow(img_i);
    p1 = P2D_tracked';
    p2 = P2D_to_track';
    p3 = P2D_new';
    p4 = first_obser_P2D_new';
    p5 = new_C';
    hold on;
    plot(p1(1,:), p1(2,:), 'bs');
    plot(p2(1,:), p2(2,:), 'bs');
    plot(p3(1,:), p3(2,:), 'rs');
    plot(p4(1,:), p4(2,:), 'rs');
    plot([p1(1,:);p2(1,:)], [p1(2,:);p2(2,:)], 'g-', 'Linewidth', 2);
    plot([p3(1,:);p4(1,:)], [p3(2,:);p4(2,:)], 'y-', 'Linewidth', 2);
    plot(p5(1,:), p5(2,:), 'mo');
    hold off;
    
    subplot(1,2,2),
    plot3(P3D_prev_good(:,1), P3D_prev_good(:,2), P3D_prev_good(:,3), 'bo');
    hold on
    plot3(P3D_new(:,1), P3D_new(:,2), P3D_new(:,3), 'ro');
    plotCoordinateFrame(eye(3),[0 0 0]', 0.8);
    text(-0.1,-0.1,-0.1,'Camera 1','fontsize',10,'color','k','FontWeight','bold');
    center_cam2_W = - R_C_W'* t_C_W;
    plotCoordinateFrame(R_C_W',center_cam2_W, 0.8);
    text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam','fontsize',10,'color','k','FontWeight','bold');
    axis equal
    rotate3d on;
    grid
    title('Cameras poses')
    
    hold off;
    
    %pause()
    
    %% set new status
    S_i = S_i_prev;
    S_i.keypoints = P2D_tot'; % P = 2xK
    S_i.landmarks = P3D_tot'; % X = 3xK
    S_i.candidates = C_tot'; % C = 2xM
    S_i.first_obser = first_obser_tot'; % F = 2xM
    S_i.cam_pos_first_obser = cam_pos_first_obser_tot'; % T = 12xM
    
end