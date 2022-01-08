clear all
close all
clc

%% Setup ==================================================================
addpath("continuous_operation\");
addpath("initialization\");

cfgp = getparameters();
ds = cfgp.ds; 


if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    kitti_path = '..\datasets\kitti';
    assert(exist('kitti_path', 'var') ~= 0);
    last_frame = 2300;
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
    % Path containing images, depths and all...
    parking_path='..\datasets\parking';
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '\K.txt']);
else
    assert(false);
end

%% Bootstrap ==============================================================

bootstrap_frames=cfgp.bootstrap_frames;

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
else
    assert(false);
end

%% Initialization =========================================================
cameraParams = cameraParameters("IntrinsicMatrix",K');
[R_C2_W, T_C2_W, keys_init, P3D_init] = initialization_KLT(img0,img1,img2, cameraParams, cfgp);

% check real solution -----------------------------------------------------
if ds ==0
    poses = load([kitti_path '\poses\00.txt']);  
    R_rea = zeros(3,3,4541);
    T_rea = zeros(3,1,4541);
elseif ds == 2
    poses = load([parking_path '\poses.txt']);
    R_rea = zeros(3,3,599);
    T_rea = zeros(3,1,599);
end
if ds==0 || ds==2
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

    T_real = [Rot_real_all(:,:,3), Trasl_real_all(:,3); 0, 0, 0, 1];
end

%% Continuous operation ===================================================

% continuous operation initialization -------------------------------------

pc = 0; %for plotting number of candidates and keypoints
pk = 0;

range = (bootstrap_frames(end)+1):last_frame;
keypoints_min = cfgp.keypoints_min; 
Can = [];
F_can =[];
T_can = [];
C_count = [];
prev_image = img2;
S_i_prev = struct('keypoints', keys_init, 'landmarks', P3D_init, 'candidates', Can, ...
                  'first_obser', F_can, 'cam_pos_first_obser', T_can, 'C_count', C_count);

% plot initialization -----------------------------------------------------
T_i_wc_history = cell(2,last_frame); % 1 --> R, 2 --> t
for i = 1:bootstrap_frames(end)-1
    T_i_wc_history{1,i} = eye(3);
    T_i_wc_history{2,i} = [0 0 0]';
end
T_i_wc_history{1,bootstrap_frames(end)} = R_C2_W';
T_i_wc_history{2,bootstrap_frames(end)} = -R_C2_W'*T_C2_W;

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
        prev_image = rgb2gray(imread([malaga_path '\'  ...
        left_images(i-1).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
        prev_image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i-1)])));
    else
        assert(false);
    end
    
    % check num_keypoints for re-initializing -----------------------------
    key_num = size(S_i_prev.keypoints,2);

    if key_num < keypoints_min
        
        if ds==0
            image0 = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i-3)]);
            image1 = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i-2)]);
        elseif ds == 1
            image0 = rgb2gray(imread([malaga_path '\'  ...
            left_images(i-3).name]));
            image1 = rgb2gray(imread([malaga_path '\' ...
            left_images(i-2).name]));
        elseif ds==2
            image0 = im2uint8(rgb2gray(imread([parking_path ...
                sprintf('/images/img_%05d.png',i-3)])));
            image1 = im2uint8(rgb2gray(imread([parking_path ...
                sprintf('/images/img_%05d.png',i-2)])));
        end
        [R, T, keys_init, P3D_init]= initialization_KLT(image0, image1, prev_image, cameraParams, cfgp);
        S_i_prev.keypoints = [S_i_prev.keypoints, keys_init];
        S_i_prev.landmarks = [S_i_prev.landmarks, T_i_wc_history{1,i-3}*P3D_init - ... 
                              T_i_wc_history{1,i-3}'*T_i_wc_history{2,i-3}];
        
        AbsolutePose = rigid3d(T_i_wc_history{1,i-1}',(-T_i_wc_history{1,i-1}'*T_i_wc_history{2,i-1})');
        ViewId = uint32(1);
        tab = table(ViewId,AbsolutePose);
        u = S_i_prev.keypoints(1,:);
        v = S_i_prev.keypoints(2,:);
        pt_array = [pointTrack(1,[u(1),v(1)])];
        parfor k = 2:size(S_i_prev.keypoints,2)
            pt_array(k) = pointTrack(1,[u(k),v(k)]);
        end
        S_i_prev.landmarks = bundleAdjustmentStructure(S_i_prev.landmarks',pt_array,tab,cameraParams)';
    end
    
    [S_i, T_i_wc] = CO_processFrame(image, prev_image, S_i_prev, cameraParams, cfgp, T_i_wc_history{1,i-1}, T_i_wc_history{2,i-1});

    max_candidates = cfgp.max_candidates;
    if size(S_i.candidates,2) >= max_candidates
        S_i.candidates = S_i.candidates(:,1:max_candidates);
        S_i.first_obser = S_i.first_obser(:,1:max_candidates);
        S_i.cam_pos_first_obser = S_i.cam_pos_first_obser(:,1:max_candidates);
        S_i.C_count = S_i.C_count(1:max_candidates);
    end

    % add poses for plotting ----------------------------------------------
    T_i_wc_history{1,i} = T_i_wc(1:3,1:3);
    T_i_wc_history{2,i} = T_i_wc(1:3,4);
    
    figure(42)
    if cfgp.plot_local
        subplot(2,2,1);
        view(0,0)
        hold on;
        axis equal;
        
        X = S_i.landmarks(1,:);
        Y = S_i.landmarks(2,:);
        Z = S_i.landmarks(3,:);
    
        traj = [-T_i_wc_history{1,i}' * T_i_wc_history{2,i}, ...
                -T_i_wc_history{1,i-1}' * T_i_wc_history{2,i-1}]';
        plot3(traj(:,1),traj(:,2), traj(:,3), '-b');
        plot3(traj(1,1),traj(1,2), traj(1,3), 'ro');
    
        if exist('h', 'var') == 1
            set(h,'Visible','off');
        end
        h = plot3(X,Y,Z, 'ko');
    
        xlim([traj(1,1)-10,traj(1,1)+10])
        ylim([traj(1,2)-10,traj(1,2)+10])
        zlim([traj(1,3)-10,traj(1,3)+10])
    end
    
    if cfgp.plot_kc
        subplot(2,2,4)
        hold on;
        candidates = [pc, size(S_i.C_count,2)];
        keypoints = [pk, size(S_i_prev.keypoints,2)];
        plot([i-1,i],candidates, 'r-');
        plot([i-1,i],keypoints,'g-');
        xlim([i-12,i+2])
        ylim([0,600])
        pc = size(S_i.C_count,2);
        pk = size(S_i_prev.keypoints,2);
    end

    % end of continuous operation -----------------------------------------
    S_i_prev = S_i;
    % Makes sure that plots refresh.    
    pause(0.1);
    
end

%% plot results ===========================================================

if cfgp.plot_final
    figure(4)
    hold on;
    if ds==0 || ds==2
        plot3(Trasl_real_all(1,1), Trasl_real_all(2,1), Trasl_real_all(3,1), '-k');
        plot3(Trasl_real_all(1,3:i), Trasl_real_all(2,3:i), Trasl_real_all(3,3:i), '-k');
        plot3(Trasl_real_all(1,1), Trasl_real_all(2,1), Trasl_real_all(3,1), '.');
        plot3(Trasl_real_all(1,3:i), Trasl_real_all(2,3:i), Trasl_real_all(3,3:i), '.');
    end
    
    x = zeros(i,1);
    y = x;
    z = x;
    for k = [1, bootstrap_frames(end):i]
        cam_center = - T_i_wc_history{1,k}'* T_i_wc_history{2,k};
        x(k) = cam_center(1);
        y(k) = cam_center(2);
        z(k) = cam_center(3);
    end
    plot3(x,y,z, 'r-');
    hold on
    plot3(x,y,z, 's');
    view(0,0)
    axis equal
end
