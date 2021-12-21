function [P, M_history_bundled] = bundleadjustment(, M_history_bundled, K)
    
% OUTPUT: 
%       P = Mx3
%       M_history_bundled = see below (but bundled)
% INPUT: 
%       S_history_bundled = 5 cells structure. Each element is a S_i.
%       M_history_bundled = 2x5 cells structure. 1 x i = Rotation of 1..5
%                                                2 x i = Traslation of 1..5
%       K = intrisic matrix

    intrinsics = cameraParameters('IntrinsicMatrix',  K');
    IDs = [1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5]';
    Rotations = [M_history_bundled{1,1}; M_history_bundled{1,2}; M_history_bundled{1,3}; M_history_bundled{1,4}; M_history_bundled{1,5}];
    Traslations = [M_history_bundled{2,1}; M_history_bundled{2,2}; M_history_bundled{2,3}; M_history_bundled{2,4}; M_history_bundled{2,5}];
    size(IDs)
    size(Rotations)
    size(Traslations)
    cameraPoses = table(IDs, Rotations, Traslations);
    
    Points = [S_history_bundled{1}.keypoints'; S_history_bundled{2}.keypoints'; S_history_bundled{3}.keypoints'; ...
        S_history_bundled{4}.keypoints'; S_history_bundled{5}.keypoints'];
    for i = 1:5
        sizes(i) = size(S_history_bundled{i}.keypoints, 2);
    end
    ViewIds = [ones(sizes(1),1); 2 * ones(sizes(2),1); 3 * ones(sizes(3),1); 4 * ones(sizes(4),1); 5 * ones(sizes(5),1)];
    size(ViewIds)
    size(Points)
    pointTracks = pointTrack(ViewIds',Points);
    size(pointTracks)
    xyzPoints = [S_history_bundled{1}.landmarks'; S_history_bundled{2}.landmarks'; S_history_bundled{3}.landmarks'; ...
        S_history_bundled{4}.landmarks'; S_history_bundled{5}.landmarks'];
    size(xyzPoints)
    [xyzRefinedPoints,refinedPoses] = bundleAdjustment(xyzPoints, pointTracks, cameraPoses, intrinsics, ' PointsUndistorted', true);

    P = xyzRefinedPoints';
    for i = 1:5
        R = refinedPoses.Orientation;
        M_history_bundled{1,i} = R(4:6,1:3)';
        T = refinedPoses.Location;
        M_history_bundled{2,i} = T(4:6);
    end
%     
%     %% TEST NEW FUNCTION
%     %% try imageviewset
%     vSet = imageviewset;
%     for i = 1:5
%         absPose = rigid3d(M_history_bundled{1,i},M_history_bundled{2,i}'); %è pose di frame wrt mondo?
%         vSet = addView(vSet,i,absPose,'Points',S_history_bundled{i}.keypoints');
%         if i>1
%             index = find(mask_TOT_BA{i});
%             index_2 = 1:1:size(mask_TOT_BA{i}(mask_TOT_BA{i} > 0));
%             index_pairs = [index, index_2'];
%             vSet = addConnection(vSet, i-1, i, 'Matches', index_pairs);
%         end
%     end
%     pointTracks = findTracks(vSet);
%     xyzPoints = unique(xyzPoints, 'rows', 'stable');
%     size(xyzPoints)
%     size(pointTracks)
%     [xyzRefinedPoints,refinedPoses] = bundleAdjustment(xyzPoints, pointTracks, cameraPoses, intrinsics, ' PointsUndistorted', true);
%     P = xyzRefinedPoints';
%     for i = 1:5
%         R = refinedPoses.Orientation;
%         M_history_bundled{1,i} = R(4:6,1:3)';
%         T = refinedPoses.Location;
%         M_history_bundled{2,i} = T(4:6);
%     end
    
    %fare add connection, serve relative pose tra due frame, serve         
    %trovare corrispondenza tra keypoints dei vari frame, farlo         
    %landmark per landmark?
end