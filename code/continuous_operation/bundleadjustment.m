function [P, M_history_bundled] = bundleadjustment(S_history_bundled, M_history_bundled, K)
    
% OUTPUT: 
%       P = Mx3
%       M_history_bundled = see below (but bundled)
% INPUT: 
%       S_history_bundled = 5 cells structure. Each element is a S_i.
%       M_history_bundled = 2x5 cells structure. 1 x i = Rotation of 1..5
%                                                2 x i = Traslation of 1..5
%       K = intrisic matrix

    intrinsics = cameraParameters('IntrinsicMatrix',  K');
    ViewId = [1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5]';
    Orientation = [M_history_bundled{1,1}; M_history_bundled{1,2}; M_history_bundled{1,3}; M_history_bundled{1,4}; M_history_bundled{1,5}];
    Location = [M_history_bundled{2,1}; M_history_bundled{2,2}; M_history_bundled{2,3}; M_history_bundled{2,4}; M_history_bundled{2,5}];
%     size(IDs)
%     size(Rotations)
%     size(Traslations)
%     cameraPoses = table('ViewId', IDs, 'Orientation',  Rotations, 'Location',  Traslations);
    cameraPoses = table(ViewId,  Orientation,  Location);
    
%     Points = [S_history_bundled{1}.keypoints'; S_history_bundled{2}.keypoints'; S_history_bundled{3}.keypoints'; ...
%         S_history_bundled{4}.keypoints'; S_history_bundled{5}.keypoints'];
%     for i = 1:5
%         sizes(i) = size(S_history_bundled{i}.keypoints, 2);
%     end
%     ViewIds = [ones(sizes(1),1); 2 * ones(sizes(2),1); 3 * ones(sizes(3),1); 4 * ones(sizes(4),1); 5 * ones(sizes(5),1)];
%     size(ViewIds)
%     size(Points)
%     pointTracks = pointTrack(ViewIds',Points);
%     size(pointTracks)
%     xyzPoints = [S_history_bundled{1}.landmarks'; S_history_bundled{2}.landmarks'; S_history_bundled{3}.landmarks'; ...
%         S_history_bundled{4}.landmarks'; S_history_bundled{5}.landmarks'];
%     size(xyzPoints)

    xyzPoints = []; %init landmarks
    for i = 1:5
        xyzPoints = [xyzPoints; S_history_bundled{i}.landmarks']; %stack all landmarks
    end
    xyzPoints = unique(xyzPoints, 'rows'); %delete duplicates
    % tracks = pointTracks(zeros(size(xyzPoints, 1)), zeros(xyzPoints, 2)); %init tracks
    p = pointTrack();
    tracks = [p];
    
    for i = 1:size(xyzPoints, 1)
        tracks(i) = p;
    end
    
    for i = 1:5
        [frame_mask, xyz_loc] = ismember([S_history_bundled{i}.landmarks';zeros(size(xyzPoints,1)-size(S_history_bundled{i}.landmarks',1),3)], xyzPoints, 'rows');
        %frame_mask has 1 where elements of S_history are found in xyzPoints [0 0 1 1 0]
        %xyz_loc has indexes of rows where elements of S_history are found   [0 0 3 5 0]
        
        h = 0;
        xyz_loc = xyz_loc(xyz_loc ~=0);
        for k = 1:size(xyz_loc, 1)%analyze indexes of xyz_loc
            j = xyz_loc(k);
            h = h + 1; 
            A = tracks(j);
            A.ViewIds = [A.ViewIds, i]; %set all vieIDs to i
            ind = find(frame_mask, h); %find indexes of first h non zero elements in mask
            ind = ind(end); %take last one, corrisponding to j element
            A.Points = [A.Points; S_history_bundled{i}.keypoints(:, ind)']; %set new points
        end
    end
    vSet = imageviewset;

    for i = 1:5
        vSet = addView(vSet, i, rigid3d(M_history_bundled{1,i}, M_history_bundled{2,i}'));
    end 
    [xyzRefinedPoints,refinedPoses] = bundleAdjustment(xyzPoints, tracks, poses(vSet), intrinsics, 'PointsUndistorted', 1);

    P = xyzRefinedPoints';
    for i = 1:5
        R = refinedPoses.AbsolutePose(i).Rotation;
        M_history_bundled{1,i} = R;
        T = refinedPoses.AbsolutePose(i).Translation;
        M_history_bundled{2,i} = T;
    end
    
end