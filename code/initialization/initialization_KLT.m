function [R_C2_W, T_C2_W, keys_init, P3D_init] = initialization_KLT(img0,img1,img2, K)
%% MATCHED POINTS ----------------------------------------------------------

% detect harris corners
corners1 = detectHarrisFeatures(img0, 'MinQuality', 0.0001);

% select strongest
N = 800;
p1 = round(selectStrongest(corners1,N).Location);

% track keypoints 
pointTracker = vision.PointTracker('MaxBidirectionalError', 1);

initialize(pointTracker,p1,img0);

[p_intermediate, valid_intermediate] = pointTracker(img1);
%p_intermediate = p_intermediate(valid_intermediate ~= 0,:);
% validity_pos = min(p_intermediate>=0,[], 2)
setPoints(pointTracker,abs(p_intermediate), valid_intermediate);
release(pointTracker);
initialize(pointTracker,abs(p_intermediate),img1);
[p2, valid_final] = pointTracker(img2);

p1 = p1(valid_final,:)';
p2 = p2(valid_final,:)';

% eliminate matches with small displacement
distances = vecnorm(p1-p2);
threshold = 3;
p1 = round(p1(:,distances>threshold));
p2 = round(p2(:,distances>threshold));


%% RELATIVE POSE -----------------------------------------------------------

% estimate fondamental Matrix
[F, inliers] = estimateFundamentalMatrix(p1(1:2, :)', p2(1:2, :)', ...
    'Method','RANSAC', 'DistanceThreshold', 0.01, 'NumTrials', 3000, 'Confidence', 99.99);
inp1 = p1(:,inliers);
inp2 = p2(:,inliers);
E=K.'*F*K;

%decompose essential matrix
% [R_C2_W,T_C2_W] = fromEtoPos(E, inp1, inp2, K);

intrinsics = cameraParameters('IntrinsicMatrix',  K');
[R_C2_W,T_C2_W] = relativeCameraPose(E,intrinsics,inp1',inp2');
R_C2_W = R_C2_W';
T_C2_W = - R_C2_W * T_C2_W';

% triangulate
inp1 = inp1(1:2,:)';
inp2 = inp2(1:2,:)';
cameraParams = cameraParameters("IntrinsicMatrix",K');

M1 = cameraMatrix(cameraParams, eye(3), [0,0,0]);
M2 = cameraMatrix(cameraParams, R_C2_W, T_C2_W);

P = triangulate(inp1,inp2,M1, M2)';

% bundle adjustement
% intrinsics = cameraParameters('IntrinsicMatrix',  K');
% IDs = [1, 1, 1, 2, 2, 2]';
% Rotations = [eye(3); R_C2_W'];
% Traslatins = [0; 0;0 ;T_C2_W];
% cameraPoses = table(IDs, Rotations, Traslatins);
% image1Points = inp1;
% image2Points = inp2;
% [~, num_matches] = size(P);
% ViewIds = [ones(1,num_matches), 2 * ones(1,num_matches)];
% Points = [image1Points;image2Points];
% pointTracks = pointTrack(ViewIds,Points);
% xyzPoints = [P(1:3,:)';P(1:3,:)'];
% size(ViewIds)
% size(Points)
% size(xyzPoints)
% [xyzRefinedPoints,refinedPoses] = bundleAdjustment(xyzPoints,pointTracks,cameraPoses,intrinsics, ' PointsUndistorted', true);
% R_C2_W = refinedPoses.Orientation;
% R_C2_W = R_C2_W(4:6,1:3)';
% T_C2_W = refinedPoses. Location;
% T_C2_W = T_C2_W(4:6);
% P = xyzRefinedPoints';

% eliminate negative and further points
[~, nme] = size(P);
V = ones(nme,1);
for i = 1:nme
   if P(3,i) <=0 || P(3,i) > 200
       V(i) = 0;
   end
end
P = P(:,V>0);
inp2 = inp2(V>0,:);
inp1 = inp1(V>0,:);
keys_init = inp2';
P3D_init = P;

%% PLOT INITIALIZATION ----------------------------------------------------

figure(1),

subplot(3,2,1),
imshow(img0,[]);
hold on
plot(p1(1,:), p1(2,:), 'ys');
title('Features detected and matched in img0')

subplot(3,2,2),
imshow(img1,[]);
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
showMatchedFeatures(img0, img1, inp1, inp2);
title('Matching kept after RANSAC applied in estimateFondamentalMatrix')

figure(2),

subplot(1,2,1)
plot3(P(1,:), P(2,:), P(3,:), 'o');
plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
center_cam2_W = -R_C2_W'*T_C2_W;
plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
axis equal
rotate3d on;
grid
title('3d point cloud and cameras')

subplot(1,2,2)
plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
center_cam2_W = -R_C2_W'*T_C2_W;
plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
axis equal
rotate3d on;
grid
title('Cameras relative poses')

end