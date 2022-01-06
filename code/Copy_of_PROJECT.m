clear all
close all
clc

%% Setup ==================================================================
ds = 3; % 0: KITTI, 1: Malaga, 2: parking

addpath("continuous_operation\");
addpath("initialization\");

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    kitti_path = '..\datasets\kitti';
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    malaga_path = '..\datasets\malaga-urban-dataset-extract-07\malaga-urban-dataset-extract-07_rectified_800x600_Images';
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir(malaga_path);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    kitti_path = '..\datasets\kitti';
    % Path containing images, depths and all...
    parking_path='..\datasets\parking';
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '\K.txt']);
     
    ground_truth = load([kitti_path '\poses\00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
elseif ds == 3
    parking_path='..\datasets\parking';
    % Path containing images, depths and all...
    custom_path ='..\datasets\custom';
    assert(exist('custom_path', 'var') ~= 0);
    last_frame = 956;
    K = [845.52896472, 0, 468.10562767;
        0, 835.35434918, 298.01291054;
        0, 0, 1];
else
    assert(false);
end

%% Bootstrap ==============================================================
if ds == 3
    bootstrap_frames=[8,10,12];
else
    bootstrap_frames=[1,2,3];
end
if ds == 0
    img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
    img2 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(3))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path '\'  ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path '\' ...
        left_images(bootstrap_frames(2)).name]));
    img2 = rgb2gray(imread([malaga_path '\' ...
        left_images(bootstrap_frames(3)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
    img2 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(3))]));
elseif ds == 3
    img0 = rgb2gray(imread([custom_path ...
        sprintf('/dataset/scene1%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([custom_path ...
        sprintf('/dataset/scene1%05d.png',bootstrap_frames(2))]));
    img2 = rgb2gray(imread([custom_path ...
        sprintf('/dataset/scene1%05d.png',bootstrap_frames(3))]));
else
    assert(false);
end

%% Initialization =========================================================

%[R_C2_W, T_C2_W, keys_init, P3D_init] = initialization(img0,img1, K);
[R_C2_W, T_C2_W, keys_init, P3D_init]= initialization_KLT(img0,img1,img2, K, ds);

T_W_C2 = [R_C2_W, T_C2_W];

T_initialization = [T_W_C2; 0, 0, 0, 1]

% check real solution -----------------------------------------------------
if ds ==0
    poses = load([kitti_path '\poses\00.txt']);  
    R_rea = zeros(3,3,4541);
    T_rea = zeros(3,1,4541);
elseif ds == 1
    poses = load([kitti_path '\poses\00.txt']);
elseif ds == 2
    poses = load([parking_path '\poses.txt']);
    R_rea = zeros(3,3,599);
    T_rea = zeros(3,1,599);
elseif ds ==3
    poses = load([parking_path '\poses.txt']);
    R_rea = zeros(3,3,599);
    T_rea = zeros(3,1,599);
end
Rot_real_all(1,1,:) = poses(:,1);
Rot_real_all(1,2,:) = poses(:,2);
Rot_real_all(1,3,:) = poses(:,3);
Rot_real_all(2,1,:) = poses(:,5);
Rot_real_all(2,2,:) = poses(:,6);
Rot_real_all(2,3,:) = poses(:,7);
Rot_real_all(3,1,:) = poses(:,9);
Rot_real_all(3,2,:) = poses(:,10);
Rot_real_all(3,3,:) = poses(:,11);
Trasl_real_all(1,:) = poses(:,4);
Trasl_real_all(2,:) = poses(:,8);
Trasl_real_all(3,:) = poses(:,12);

T_real = [Rot_real_all(:,:,3), Trasl_real_all(:,3); 0, 0, 0, 1]

%% Continuous operation ===================================================

% continuous operation initialization -------------------------------------
range = (bootstrap_frames(end)+1):last_frame;    
% keys = Pi = 2xK
% P3D = Xi = 3xK
% Can = Ci = 2xM
% F_can = Fi = 2xM
% T_can = Ti = 12xM
Can = [];
F_can =[];
T_can = [];
S_i_prev = struct('keypoints', keys_init, 'landmarks', P3D_init, 'candidates', Can, 'first_obser', F_can, 'cam_pos_first_obser', T_can);
prev_image = img2;

% plot initialization -----------------------------------------------------
T_i_wc_history = cell(2,last_frame); % 1 --> R, 2 --> t
for i = 1:bootstrap_frames(end)-1
    T_i_wc_history{1,i} = eye(3);
    T_i_wc_history{2,i} = [0 0 0]';
end
T_i_wc_history{1,bootstrap_frames(end)} = R_C2_W;
T_i_wc_history{2,bootstrap_frames(end)} = T_C2_W;

R_i_wc = R_C2_W;
T_i_wc = T_C2_W;

keypoints_min = 150;

X=[];
Y=[];
Z=[];

% analyze every frame -----------------------------------------------------
for i = range
    
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
        prev_image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i-1)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path '\'  ...
        left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
        prev_image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i-1)])));
    elseif ds ==3
        image = im2uint8(rgb2gray(imread([custom_path ...
            sprintf('/dataset/scene1%05d.png',i)])));
        prev_image = im2uint8(rgb2gray(imread([custom_path ...
            sprintf('/dataset/scene1%05d.png',i-1)])));
    else
        assert(false);
    end
    
    % check num_keypoints for re-initializing -----------------------------
    key_num = size(S_i_prev.keypoints);

    if key_num < keypoints_min 
        if ds == 2
            image0 = im2uint8(rgb2gray(imread([parking_path ...
                sprintf('/images/img_%05d.png',i-3)])));
            image1 = im2uint8(rgb2gray(imread([parking_path ...
                sprintf('/images/img_%05d.png',i-2)])));
        elseif ds == 0
            image0 = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i-3)]);
            image1 = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i-2)]);
        elseif ds == 3
            image0 = rgb2gray(imread([custom_path '/dataset/' sprintf('scene1%05d.png',i-3)]));
            image1 = rgb2gray(imread([custom_path '/dataset/' sprintf('scene1%05d.png',i-2)]));
        else
        end
        [~, ~, keys_init, P3D_init]= initialization_KLT(image0, image1, prev_image, K, ds);
        S_i_prev.keypoints = [S_i_prev.keypoints, keys_init];
        S_i_prev.landmarks = [S_i_prev.landmarks, T_i_wc_history{1,i-3}'*P3D_init - ... 
                         T_i_wc_history{1,i-3}'*T_i_wc_history{2,i-3}];
        
    end
    max_candidates = 600;
    if size(S_i_prev.candidates,2) > max_candidates
        S_i_prev.candidates = S_i_prev.candidates(:,1:max_candidates);
        S_i_prev.first_obser = S_i_prev.first_obser(:,1:max_candidates);
        S_i_prev.cam_pos_first_obser = S_i_prev.cam_pos_first_obser(:,1:max_candidates);
    end
    
    [S_i, T_i_wc] = CO_processFrame(image, prev_image, S_i_prev, K, ds);

    % add poses for plotting ----------------------------------------------
    T_i_wc_history{1,i} = T_i_wc(1:3,1:3);
    T_i_wc_history{2,i} = T_i_wc(1:3,4);

    figure(42)
    hold on;
    
    X = S_i.landmarks(1,:);
    Y = S_i.landmarks(2,:);
    Z = S_i.landmarks(3,:);

    traj = [-T_i_wc_history{1,i}'* T_i_wc_history{2,i}, ...
            -T_i_wc_history{1,i-1}'* T_i_wc_history{2,i-1}]';
    plot3(traj(:,1),traj(:,2), traj(:,3), '-b');
    plot3(traj(1,1),traj(1,2), traj(1,3), 'ro');

    if exist('h', 'var') == 1
        set(h,'Visible','off');
    end
    h = plot3(X,Y,Z, 'ko');

    xlim([traj(1,1)-30,traj(1,1)+10])
    ylim([traj(1,2)-10,traj(1,2)+10])
    zlim([traj(1,3)-30,traj(1,3)+50])

    % end of continuous operation -----------------------------------------
    S_i_prev = S_i;
    % Makes sure that plots refresh.    
    pause(0.01);
    
end

%% plot results ===========================================================

figure(4)
subplot(1,2,1);
plot3(Trasl_real_all(1,1), Trasl_real_all(2,1), Trasl_real_all(3,1), '-');
hold on
plot3(Trasl_real_all(1,3:i), Trasl_real_all(2,3:i), Trasl_real_all(3,3:i), '-');
hold on
plot3(Trasl_real_all(1,1), Trasl_real_all(2,1), Trasl_real_all(3,1), 's');
hold on
plot3(Trasl_real_all(1,3:i), Trasl_real_all(2,3:i), Trasl_real_all(3,3:i), 's');
title(['Real trajectory from 1 to ' num2str(i)]);

subplot(1,2,2);
x = zeros(i,1);
y = x;
z = x;
for k = [1, bootstrap_frames(end):i]
    cam_center = - T_i_wc_history{1,k}'* T_i_wc_history{2,k};
    x(k) = cam_center(1);
    y(k) = cam_center(2);
    z(k) = cam_center(3);
end
plot3(x,y,z, '-');
hold on
plot3(x,y,z, 's');
if ds == 0
    xlim([-3,3])
    ylim([-3,3])
    zlim([-1,30])
end
if ds == 2
    xlim([-1,30])
    ylim([-3,3])
    zlim([-3,3])
end
title(['Found trajectory from 1 to ' num2str(i)]);
% 
% for j = [1, bootstrap_frames(end):i]
%     figure(5)
%     subplot(1,2,2);
%     plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
%     text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
%     center_cam2_W =  Trasl_real_all(:,j);
%     plotCoordinateFrame(Rot_real_all(:,:,j),center_cam2_W, 0.8);
%     text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,['Cam ' num2str(j)],'fontsize',10,'color','k','FontWeight','bold');
%     axis equal
%     rotate3d on;
%     grid
%     title('Cameras poses real')
%     
%     subplot(1,2,1);
%     plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
%     text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
%     center_cam2_W = - T_i_wc_history{1,j}'* T_i_wc_history{2,j};
%     plotCoordinateFrame(T_i_wc_history{1,j}',center_cam2_W, 0.8);
%     text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,['Cam ' num2str(j)],'fontsize',10,'color','k','FontWeight','bold');
%     axis equal
%     rotate3d on;
%     grid
%     title('Cameras poses found')
% end