clear all
close all
% clc
% addpath('C:\Users\edoar\Desktop\UNI\VAMR_Vision_Algorithms_for_Mobile_Robotics\project_new\')
%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    kitti_path = 'kitti';
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    malaga_path = 'malaga/malaga-urban-dataset-extract-07';
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    parking_path='parking';
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
bootstrap_frames=[1,3];
if ds == 0
    img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

%% Initialization

[R_C2_W, T_C2_W] = initialization(img0,img1, K);
T = [R_C2_W, T_C2_W; 0, 0, 0, 1]

% check real solution
if ds ==0
    poses = load('kitti/poses/00.txt');  
    R = zeros(3,3,4541);
    T = zeros(3,1,4541);
elseif ds == 1
    poses = load('kitti/poses/00.txt');
elseif ds == 2
    poses = load('parking/poses.txt');
    R = zeros(3,3,599);
    T = zeros(3,1,599);
end

R(1,1,:) = poses(:,1);
R(1,2,:) = poses(:,2);
R(1,3,:) = poses(:,3);
R(2,1,:) = poses(:,5);
R(2,2,:) = poses(:,6);
R(2,3,:) = poses(:,7);
R(3,1,:) = poses(:,9);
R(3,2,:) = poses(:,10);
R(3,3,:) = poses(:,11);
T(1,1,:) = poses(:,4);
T(2,1,:) = poses(:,8);
T(3,1,:) = poses(:,12);
%figure(8)
%plot3(X,Y,Z)

Rot = R(:,:,1) * R(:,:,3)';
Trasl = T(:,:,1) - T(:,:,3);
T_real = [Rot, Trasl; 0, 0, 0, 1]

Rot_err = (rotm2eul(Rot)  - rotm2eul(R_C2_W)) * 57.2958
Trasl_err = (Trasl - T_C2_W)'
return 

%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    
    % [outputArg1,outputArg2] = continuousOperation(inputArg1,inputArg2)
    
    % Makes sure that plots refresh.    
    pause(0.01);
    
    prev_img = image;
end