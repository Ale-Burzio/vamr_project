function [R_C2_W, T_C2_W, keys_init, P3D_init] = initialization_KLT(img0,img1,img2, cameraParams, cfgp)
%% MATCHED POINTS ----------------------------------------------------------

% detect harris corners
corners1 = detectHarrisFeatures(img0, 'MinQuality', cfgp.harris_minquality);

% select strongest
p1 = selectStrongest(corners1,cfgp.strongest_init).Location;

% track keypoints 
pointTracker = vision.PointTracker('MaxBidirectionalError', cfgp.max_bidirectional_error);

initialize(pointTracker,p1,img0);

[p_intermediate, valid_intermediate] = pointTracker(img1);
p_intermediate = p_intermediate(valid_intermediate,:);
release(pointTracker);
initialize(pointTracker,p_intermediate,img1);
[p2, valid_final] = pointTracker(img2);

p1 = p1(valid_intermediate,:);
p1 = p1(valid_final,:)';
p2 = p2(valid_final,:)';

if cfgp.ds == 2
    p1 = round(p1);
    p2 = round(p2);
end

% delete matches with small displacement
distances = vecnorm(p1-p2);
threshold = cfgp.min_displacement_match;
p1 = p1(:,distances>threshold);
p2 = p2(:,distances>threshold);


%% RELATIVE POSE -----------------------------------------------------------

% estimate fondamental Matrix
K = cameraParams.IntrinsicMatrix';
[F, inliers] = estimateFundamentalMatrix(p1(1:2, :)', p2(1:2, :)', ...
    'Method','RANSAC', 'DistanceThreshold', cfgp.error_threshold, ...
    'NumTrials', cfgp.ransac_numtrials, 'Confidence', cfgp.ransac_confidence);
inp1 = p1(:,inliers);
inp2 = p2(:,inliers);
E=K.'*F*K;

[R_C2_W,T_C2_W] = relativeCameraPose(E, cameraParams, inp1',inp2');
T_C2_W = T_C2_W';
R_C2_W = R_C2_W';

% triangulate
inp1 = inp1(1:2,:)';
inp2 = inp2(1:2,:)';

M1 = cameraMatrix(cameraParams, eye(3), [0,0,0]);
M2 = cameraMatrix(cameraParams, R_C2_W, -R_C2_W'*T_C2_W);

[P,~,val] = triangulate(inp1,inp2,M1,M2);
ft = cfgp.furthest_triangulate;
parfor k = 1: size(P,1)
    if norm(P(k,:)) > ft
        val(k) = false;
    end
end

P = P(val,:)';
inp2 = inp2(val,:)';

keys_init = inp2;
P3D_init = P;

%% PLOT INITIALIZATION ----------------------------------------------------
if cfgp.plot_initialization
    figure(1),
    
    subplot(3,2,1),
    imshow(img0);
    hold on
    plot(p1(1,:), p1(2,:), 'ys');
    title('Features detected and matched in img0')
    
    subplot(3,2,2),
    imshow(img1);
    hold on
    plot(p2(1,:), p2(2,:), 'ys');
    title('Image 3')
    title('Features detected and matched in img1')
    
    subplot(3,2,3:4),
    imshow(img1);
    hold on;
    plot(p1(1,:), p1(2,:), 'bs');
    hold on;
    plot(p2(1,:), p2(2,:), 'rs');
    hold on;
    plot([p1(1,:);p2(1,:)], [p1(2,:);p2(2,:)], 'g-', 'Linewidth', 2);
    title('Output from Feature Detector, Descriptor and Matcher')
    
    subplot(3,2,5:6),
    showMatchedFeatures(img0, img1, inp1', inp2');
    title('Matching kept after RANSAC applied in estimateFondamentalMatrix')

    figure(2)
    close(2)
    figure(2),
    
    subplot(1,2,1)
    plot3(P(1,:), P(2,:), P(3,:), 'o');
    plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
    text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
    center_cam2_W = T_C2_W;
    plotCoordinateFrame(R_C2_W,center_cam2_W, 0.8);
    text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
    axis equal
    rotate3d on;
    grid
    title('3d point cloud and cameras')
    
    subplot(1,2,2)
    hold on;
    plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
    text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
    center_cam2_W = T_C2_W;
    plotCoordinateFrame(R_C2_W,center_cam2_W, 0.8);
    text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
    axis equal
    rotate3d on;
    grid
    title('Cameras relative poses')

    hold off;
    

    pause()
end
end