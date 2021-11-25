function [R_C2_W, T_C2_W] = initialization(img0,img1, K)
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
    'BlockSize', descriptor_radius, 'Method', 'FREAK');
[features2,valid_points2] = extractFeatures(img1, points2, ...
    'BlockSize', descriptor_radius, 'Method', 'FREAK');

% match keypoints 
matches = matchFeatures(features1,features2);

% estract location of matched keypoints
p1 = valid_points1.Location(matches(:,1),:)';
p2 = valid_points2.Location(matches(:,2),:)';

lenp1 =length(p1);
lenp2 = length(p1);
p1 = [p1;ones(1,lenp1)];
p2 = [p2;ones(1,lenp2)];

%eliminate matches with small displacement
threshold = 3;
distances = vecnorm(p1-p2);

p1 = p1(:,distances>threshold);
p2 = p2(:,distances>threshold);

%% PLOT MATCH SOLUTION ----------------------------------------------------
figure(1),
imshow(img1);
hold on;
plot(p1(1,:), p1(2,:), 'bs');
hold on;
plot(p2(1,:), p2(2,:), 'rs');
hold on;
plot([p1(1,:);p2(1,:)], [p1(2,:);p2(2,:)], 'g-', 'Linewidth', 2);

figure(2),
subplot(1,2,1)
imshow(img0,[]);
hold on
plot(p1(1,:), p1(2,:), 'ys');
title('Image 1')
subplot(1,2,2)
imshow(img1,[]);
hold on
plot(p2(1,:), p2(2,:), 'ys');
title('Image 3')

%% RELATIVE POSE -----------------------------------------------------------

% estimate fondamental Matrix
[F, inliers] = estimateFundamentalMatrix(p1(1:2, :)', p2(1:2, :)', ...
    'Method','RANSAC', 'DistanceThreshold', 0.01, 'NumTrials', 80000, 'Confidence', 99.99);
inp1 = p1(:,inliers);
inp2 = p2(:,inliers);
figure(4),
showMatchedFeatures(img0, img1, inp1(1:2, :)', inp2(1:2, :)');
E=K.'*F*K;

%decompose essential matrix
[R_C2_W,T_C2_W] = fromEtoPos(E, inp1, inp2, K);     % function file
M1 = K * eye(3,4);
M2 = K * [R_C2_W, T_C2_W];
P = linearTriangulation(p1,p2,M1,M2);               % function file
% tform = 
% P = triangulate(p1,p2,)

% eliminate negative and further points
[~, nme] = size(P);
for i = 1:nme
   if P(3,i) <=0 || P(3,i) > 200
       P(3,i) = 0;
       P(2,i) = 0;
       P(1,i) = 0;
   end
end

%% PLOT POSE SOLUTION -----------------------------------------------------

figure(3),
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

subplot(1,2,2)
plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
center_cam2_W = -R_C2_W'*T_C2_W;
plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
axis equal
rotate3d on;
grid
end