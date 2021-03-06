function [S_i,T_i_wc] = CO_processFrame(img_i, img_i_prev, S_i_prev, cameraParams, cfgp, R_prev, T_prev)
    %% status prev
    P = S_i_prev.keypoints'; % P = Kx2
    X = S_i_prev.landmarks'; % X = Kx3
    C_prev = S_i_prev.candidates'; % C = Mx2
    C_count = S_i_prev.C_count; % C_count = Mx1
    first_obser = S_i_prev.first_obser'; % F = Mx2
    cam_pos_first_obser = S_i_prev.cam_pos_first_obser'; % T = Mx12
    
    pt1 = vision.PointTracker('MaxBidirectionalError', cfgp.max_bidirectional_error);
    pt2 = vision.PointTracker('MaxBidirectionalError', cfgp.max_bidirectional_error);
    P_new = [];
    X_new = [];
    first_obser_P_new = [];
    C_next = [];
    
    %% track old P

    initialize(pt1,P,img_i_prev);
    [P_next, val_P_next] = pt1(img_i);
    release(pt1);
    P_next = P_next(val_P_next,:);

    if cfgp.ds==2
        P_next = round(P_next);
    end

    X_prev_good = X(val_P_next,:);

    %% RANSAC P3P + NL pose optimization

    [R_W_C, t_W_C, inliers_mask] = estimateWorldCameraPose(P_next, X_prev_good, cameraParams, ...
                                    'Confidence', cfgp.ransac_confidence, 'MaxNumTrials', cfgp.ransac_numtrials, ...
                                    'MaxReprojectionError', cfgp.error_threshold);

    P_next = P_next(inliers_mask, :);
    X_prev_good = X_prev_good(inliers_mask, :);

    T_rigid = rigid3d(R_W_C, t_W_C);
    T_rigid_refined = bundleAdjustmentMotion(X_prev_good, P_next, T_rigid, cameraParams, 'PointsUndistorted', true);
    R_C_W = T_rigid_refined.Rotation';
    t_C_W = -R_C_W*T_rigid_refined.Translation';
    T_i_wc = [R_C_W, t_C_W];
     
    %% track old C

    if not(isempty(C_prev))
        initialize(pt2,C_prev,img_i_prev);
        [C_next, val_C_next] = pt2(img_i);
        release(pt2);
        C_next = C_next(val_C_next,:);
        if cfgp.ds==2
            C_next = round(C_next);
        end
        first_obser = first_obser(val_C_next,:);
        cam_pos_first_obser = cam_pos_first_obser(val_C_next,:);
        C_count = C_count(val_C_next)+1;
    end

    %% find new FEATURES
    corners = detectHarrisFeatures(img_i, 'MinQuality', cfgp.harris_minquality);
    features = selectStrongest(corners,cfgp.strongest_co).Location;

    %% find new candidates from features

    PC = [C_next; P_next]; % in current image
    distances_features_PC = min(pdist2(features, PC),[],2);
    tresh_dis = cfgp.min_displacement_match;
    C_new = round(features(distances_features_PC > tresh_dis, :));
    C_count_new = ones(1,size(C_new,1));
    cam_pos_first_obser_C_new = repmat(reshape(T_i_wc, [1, 12]),[size(C_new,1),1]);

    %% find new P from C and triangulate
    if not(isempty(C_next))
        angles = C_F_angle(C_next, first_obser, T_i_wc, cam_pos_first_obser, cameraParams.IntrinsicMatrix');
        tresh_angle = cfgp.min_angle_deg * pi / 180;

        validity_angle = abs(angles) > tresh_angle;
        validity_count = C_count > cfgp.min_consecutive_frames;
        validity = (validity_count'+validity_angle)>0;

        P_new = C_next(validity,:);
        P_count = C_count(validity);
        first_obser_P_new = first_obser(validity,:);
        cam_pos_first_obser_P_new = cam_pos_first_obser(validity,:);

        X_new = zeros(size(P_new,1),3);
        repro = [];
        val = [];
        for k = 1:size(P_new,1)
            T_first_w = reshape(cam_pos_first_obser_P_new(k,:),[3,4]);
            R_F = T_first_w(:,1:3);
            t_F = T_first_w(:,4);
    
            M1 = cameraMatrix(cameraParams, R_F, t_F);
            M2 = cameraMatrix(cameraParams, R_C_W, t_C_W);
            
            p1 = first_obser_P_new(k,:);
            p2 = P_new(k,:);
          
            [X_new(k,:), repro(k), val(k)] = triangulate(p1,p2,M1,M2); %triangulates in world frame
            
            % check if triangulated point too far or repro error too high
            if repro(k) > cfgp.max_repro || norm(abs(X_new(k,:)')-abs(t_C_W)) > cfgp.furthest_triangulate
                val(k) = 0;
            end
            
        end

        val = val>0;
        
        if not(isempty(P_new))
            % remove new P from C, F, T
            C_next = [P_new(not(val),:); C_next(not(validity),:)];
            C_count = [P_count(not(val)),C_count(not(validity))];
            first_obser = [first_obser_P_new(not(val),:); ...
                           first_obser(not(validity),:)];
            cam_pos_first_obser = [cam_pos_first_obser_P_new(not(val),:); ...
                                   cam_pos_first_obser(not(validity),:)];

            % remove incorrect triangulations
            P_new = P_new(val,:);
            first_obser_P_new = first_obser_P_new(val,:);
            X_new = X_new(val,:);
        end

        if not(isempty(P_new))
            u = P_new(:,1);
            v = P_new(:,2);
            pt_array = [pointTrack(1,[u(1),v(1)])];
            AbsolutePose = rigid3d(R_C_W',(-R_C_W'*t_C_W)');
            ViewId = uint32(1);
            tab = table(ViewId,AbsolutePose);
            parfor k = 2:size(P_new,1)
                pt_array(k) = pointTrack(1,[u(k),v(k)]);
            end
            X_new = bundleAdjustmentStructure(X_new,pt_array,tab,cameraParams, 'PointsUndistorted', true);
        end
    end

    %% add new P to old P (P2D_tracked), new X to old X 
    P_tot = [P_next; P_new];
    X_tot = [X_prev_good; X_new];

    C_tot = [C_next; C_new];
    C_count_tot = [C_count, C_count_new];
    first_obser_tot = [first_obser; C_new];
    cam_pos_first_obser_tot = [cam_pos_first_obser; cam_pos_first_obser_C_new];

    %% set new status
    S_i = S_i_prev;
    S_i.keypoints = P_tot'; % P = 2xK
    S_i.landmarks = X_tot'; % X = 3xK
    S_i.candidates = C_tot'; % C = 2xM
    S_i.C_count = C_count_tot;
    S_i.first_obser = first_obser_tot'; % F = 2xM
    S_i.cam_pos_first_obser = cam_pos_first_obser_tot'; % T = 12xM

    %% PLOT
    if cfgp.plot_correspond
        figure(42)
        subplot(2,2,2)
        imshow(img_i);
        hold on;
        plot(P_next(:,1), P_next(:,2), '.g');
        if not(isempty(C_next))
            plot(C_next(:,1), C_next(:,2), '.r');
        end
        if not(isempty(C_new))
            plot(C_new(:,1), C_new(:,2), '.m')
        end
        
        if not(isempty(P_new))
            plot(P_new(:,1), P_new(:,2), 'cs');
            plot(first_obser_P_new(:,1), first_obser_P_new(:,2), 'bs');
            plot([P_new(:,1)'; first_obser_P_new(:,1)'], [P_new(:,2)'; first_obser_P_new(:,2)'], 'y-', 'Linewidth', 1);
        end
        title("keypoints (green), candidates (red), new candidates (magenta)");
        hold off;
    end
    if cfgp.plot_cameras
        figure(42)
    
        subplot(2,2,3)
        if cfgp.plot_new
            plot3(X_prev_good(:,1), X_prev_good(:,2), X_prev_good(:,3), 'bo');
            hold on
            if not(isempty(X_new))
                plot3(X_new(:,1), X_new(:,2), X_new(:,3), 'ro');
            end
        end
        center_cam2_W = -R_C_W'*t_C_W;
        plot3(center_cam2_W(1), center_cam2_W(2), center_cam2_W(3))
        plotCoordinateFrame(R_prev,-R_prev'*T_prev, 0.8);
        plotCoordinateFrame(R_C_W,center_cam2_W, 0.8);
        text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam','fontsize',10,'color','k','FontWeight','bold');
        axis equal
        rotate3d on;
        grid
        title('Cameras poses')
        view(0,0)
        hold off;
    end
end