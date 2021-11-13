function [p1, p2, n_matches] = MatchedPoints(img0,img1)
%% parameters
corner_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 400;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

%% harris img 1, 3

harris_scores_0 = harris(img0, corner_patch_size, harris_kappa);
assert(min(size(harris_scores_0) == size(harris_scores_0)));

harris_scores_1 = harris(img1, corner_patch_size, harris_kappa);
assert(min(size(harris_scores_1) == size(harris_scores_1)));

%{
figure('Color', 'w');
subplot(2, 2, 1);
imshow(img0);
subplot(2, 2, 2);
imshow(img1);

subplot(2, 2, 3);
imagesc(harris_scores_0);
title('Harris 0');
daspect([1 1 1]);
axis off;

subplot(2, 2, 4);
imagesc(harris_scores_1);
title('Harris 1');
daspect([1 1 1]);
axis off;
%}

%% select keypoints

keypoints_0 = selectKeypoints(...
    harris_scores_0, num_keypoints, nonmaximum_supression_radius);

keypoints_1 = selectKeypoints(...
    harris_scores_1, num_keypoints, nonmaximum_supression_radius);

%{
figure(2);
imshow(img0);
hold on;
plot(keypoints_0(2, :), keypoints_0(1, :), 'rx', 'Linewidth', 2);

figure(3);
imshow(img1);
hold on;
plot(keypoints_1(2, :), keypoints_1(1, :), 'rx', 'Linewidth', 2);
%}
    
%% describe

descriptors_0 = describeKeypoints(img0, keypoints_0, descriptor_radius);
descriptors_1 = describeKeypoints(img1, keypoints_1, descriptor_radius);

%% matching

matches = matchDescriptors(descriptors_1, descriptors_0, match_lambda)

% figure(4);
% imshow(img1);
% hold on;
% plot(keypoints_1(2, :), keypoints_1(1, :), 'rx', 'Linewidth', 2);
% plotMatches(matches, keypoints_1, keypoints_0);

n_matches = length(matches(matches>0))
p1=zeros(2,n_matches);
p2=zeros(2,n_matches);
c=1;

for i = 1:length(matches)
    if matches(i) ~= 0
        p1(2:-1:1, c) = keypoints_0(:,matches(i));
        p2(2:-1:1, c) = keypoints_1(:,i);
        c=c+1;
    end
end
