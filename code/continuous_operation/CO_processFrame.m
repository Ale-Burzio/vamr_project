function [S_i,T_i_wc] = CO_processFrame(img_i, img_i_prev, S_i_prev, K, ds)
    %% status prev
    P = S_i_prev.keypoints'; % P = Kx2
    X = S_i_prev.landmarks'; % X = Kx3
    C_prev = S_i_prev.candidates'; % C = Mx2
    first_obser = S_i_prev.first_obser'; % F = Mx2
    cam_pos_first_obser = S_i_prev.cam_pos_first_obser'; % T = Mx12
   
    pt1 = vision.PointTracker('MaxBidirectionalError', 0.5);
    pt2 = vision.PointTracker('MaxBidirectionalError', 0.5);
    P_new = [];
    X_new = [];
    first_obser_P_new = [];
    cam_pos_first_obser_P_new = [];
    C_next = [];
    
    %% track old P

    initialize(pt1,P,img_i_prev);
    [P_next, val_P_next] = pt1(img_i);
    release(pt1);
    P_prev_good = P(val_P_next,:);
    P_next = P_next(val_P_next,:);
    if ds==2
        P_next = round(P_next);
    end
    X_prev_good = X(val_P_next,:);

    figure(4)
    showMatchedFeatures(img_i_prev, img_i, P_prev_good, P_next);

    %% RANSAC P3P

    [R_C_W, t_C_W, inliers_mask, ~, ~] = ransacLocalization(P_next', X_prev_good', K);
    
    T_i_wc = [R_C_W, t_C_W]
    P_next = P_next(inliers_mask, :);
    P_prev_good = P_prev_good(inliers_mask,:);
    X_prev_good = X_prev_good(inliers_mask, :);
    
    %% track old C
    if not(isempty(C_prev))
        initialize(pt2,C_prev,img_i_prev);
        [C_next, val_C_next] = pt2(img_i);
        release(pt2);
        C_next = C_next(val_C_next,:);
        if ds==2
            C_next = round(C_next);
        end
        first_obser = first_obser(val_C_next,:);
        cam_pos_first_obser = cam_pos_first_obser(val_C_next,:);
    end

    %% find new FEATURES
    corners = detectHarrisFeatures(img_i, 'MinQuality', 1e-8);
    N = 1000;
    features = selectStrongest(corners,N).Location;

    %% find new candidates from features

    PC = [C_next; P_next]; % in current image
    distances_features_PC = min(pdist2(features, PC),[],2);
    tresh_dis = 4;
    C_new = features(distances_features_PC > tresh_dis, :);
    cam_pos_first_obser_C_new = repmat(reshape(T_i_wc, [1, 12]),[size(C_new,1),1]);

    %% find new P from C and triangulate
    if not(isempty(C_next))
        angles = C_F_angle(C_next, first_obser, T_i_wc, cam_pos_first_obser, K);
        tresh_angle = 10 * pi / 180;
        validity_angle = abs(angles) > tresh_angle;

        P_new = C_next(validity_angle,:);
        first_obser_P_new = first_obser(validity_angle,:);
        cam_pos_first_obser_P_new = cam_pos_first_obser(validity_angle,:);

        cameraParams = cameraParameters("IntrinsicMatrix",K');

        X_new = zeros(size(P_new,1),3);
        val = [];
        repro = [];
        parfor k = 1:size(P_new,1)
            T_first_w = reshape(cam_pos_first_obser_P_new(k,:),[3,4]);
            R_F = T_first_w(:,1:3);
            t_F = T_first_w(:,4);
    
            M1 = cameraMatrix(cameraParams, R_F', t_F);
            M2 = cameraMatrix(cameraParams, R_C_W', t_C_W);
            
            p1 = first_obser_P_new(k,:);
            p2 = P_new(k,:);
          
            [X_new(k,:), repro(k), val(k)] = triangulate(p1,p2,M1,M2); %triangulates in world frame
            
            % check if triangulated point repro error too high
            if repro(k) > 0.3
                val(k) = 0;
            end
        end

        val = val>0;
        
        if not(isempty(P_new))
            % remove new P from C, F, T
            C_next = [C_next(not(validity_angle),:); P_new(not(val),:)];
            first_obser = [first_obser(not(validity_angle),:); ...
                           first_obser_P_new(not(val),:)];
            cam_pos_first_obser = [cam_pos_first_obser(not(validity_angle),:); ...
                                    cam_pos_first_obser_P_new(not(val),:)];

            % remove incorrect triangulations
            P_new = P_new(val,:);
            first_obser_P_new = first_obser_P_new(val,:);
            X_new = X_new(val,:);
        end
    end

    %% add new P to old P (P2D_tracked), new X to old X 
    P_tot = [P_next; P_new];
    X_tot = [X_prev_good; X_new];

    C_tot = [C_next; C_new];
    first_obser_tot = [first_obser; C_new];
    cam_pos_first_obser_tot = [cam_pos_first_obser; cam_pos_first_obser_C_new];

    %% set new status
    S_i = S_i_prev;
    S_i.keypoints = P_tot'; % P = 2xK
    S_i.landmarks = X_tot'; % X = 3xK
    S_i.candidates = C_tot'; % C = 2xM
    S_i.first_obser = first_obser_tot'; % F = 2xM
    S_i.cam_pos_first_obser = cam_pos_first_obser_tot'; % T = 12xM

    %% PLOT
    if false
        figure(3),
        subplot(1,2,1),
        hold on;
        imshow(img_i);
        p3 = P_new';
        p5 = first_obser_P_new';
        
        if not(isempty(p3))
            plot(p3(1,:), p3(2,:), 'bs');
            plot(p5(1,:), p5(2,:), 'bs');
            plot([p3(1,:);p5(1,:)], [p3(2,:);p5(2,:)], 'y-', 'Linewidth', 1);
        end
    
        hold off;
        
        subplot(1,2,2),
        plot3(X_prev_good(:,1), X_prev_good(:,2), X_prev_good(:,3), 'bo');
        hold on
        if not(isempty(X_new))
            plot3(X_new(:,1), X_new(:,2), X_new(:,3), 'ro');
        end
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
        
        pause(0.05)
    end
end