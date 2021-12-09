function [R_C2_W, T_C2_W, keys_init, P3D_init] = initialization(img0,img1, K)
%% MATCHED POINTS ----------------------------------------------------------

% detect harris corners
corners1 = detectHarrisFeatures(img0, 'FilterSize', 5, 'MinQuality', 0.0001);
corners2 = detectHarrisFeatures(img1, 'FilterSize', 5, 'MinQuality', 0.0001);

% select strongest
N = 4000;
points1 = selectStrongest(corners1,N);
points2 = selectStrongest(corners2,N);

% estract freak features
descriptor_radius = 19;
[features1,valid_points1] = extractFeatures(img0, points1, ...
    'BlockSize', descriptor_radius, 'Method', 'SURF');
[features2,valid_points2] = extractFeatures(img1, points2, ...
    'BlockSize', descriptor_radius, 'Method', 'SURF');

% match keypoints 
matches = matchFeatures(features1,features2);

% estract location of matched keypoints
p1 = valid_points1.Location(matches(:,1),:)';
p2 = valid_points2.Location(matches(:,2),:)';
p1 = round(p1);
p2 = round(p2);
lenp1 =length(p1);
lenp2 = length(p1);
p1 = [p1;ones(1,lenp1)];
p2 = [p2;ones(1,lenp2)];

% eliminate matches with small displacement
distances = vecnorm(p1-p2);
threshold = 5;
p1 = p1(:,distances>threshold);
p2 = p2(:,distances>threshold);

%% RELATIVE POSE -----------------------------------------------------------

% estimate fondamental Matrix
[F, inliers] =estimateFundamentalMatrix(p1(1:2, :)', p2(1:2, :)', ...
    'Method','RANSAC', 'DistanceThreshold', 0.01, 'NumTrials', 8000, 'Confidence', 99.99);
inp1 = p1(:,inliers);
inp2 = p2(:,inliers);
E=K.'*F*K;

%decompose essential matrix
[R_C2_W,T_C2_W] = fromEtoPos(E, inp1, inp2, K);

% triangulate
inp1 = inp1(1:2,:)';
inp2 = inp2(1:2,:)';
cameraParams = cameraParameters("IntrinsicMatrix",K');
stereoParams = stereoParameters(cameraParams, cameraParams, R_C2_W', T_C2_W'); 
P = triangulate(inp1,inp2,stereoParams)';

% bundle adjustement
% intrinsics = cameraParameters('IntrinsicMatrix',  K');
% % ViewId = [1, 1, 1, 2, 2, 2]';
% % Orientation = [eye(3); R_C2_W'];
% % Location = [0; 0;0 ;T_C2_W];
% %cameraPoses = table(ViewId, Orientation, Location)
% ViewId = uint32([1, 2]');
% AbsolutePose = [rigid3d(eye(3), zeros(1,3)); rigid3d(R_C2_W', T_C2_W')];
% cameraPoses = table(ViewId, AbsolutePose);
% image1Points = inp1;
% image2Points = inp2;
% [~, num_matches] = size(P);
% ViewIds = [1, 2];
% for i = 1:num_matches 
%     Points = [image1Points(i,:);image2Points(i,:)];
%     pointTracks(i) = pointTrack(ViewIds, Points);
% end
% xyzPoints = P(1:3,:)';
% [xyzRefinedPoints,refinedPoses] = bundleAdjustment(xyzPoints,pointTracks,cameraPoses,intrinsics, 'PointsUndistorted', true);
% R_C2_W = refinedPoses.AbsolutePose(1).Rotation;
% R_C2_W = R_C2_W';
% T_C2_W = refinedPoses.AbsolutePose(1).Translation;
% T_C2_W = T_C2_W';
% P = xyzRefinedPoints';

% eliminate negative and further points
[~, nme] = size(P);
for i = 1:nme
   if P(3,i) <=0 || P(3,i) > 200
       P(3,i) = 0;
       P(2,i) = 0;
       P(1,i) = 0;
   end
end

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