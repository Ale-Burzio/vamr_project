function E = estimateEssentialMatrix_mia(p1, p2, K1, K2,img, img0)
% estimateEssentialMatrix_normalized: estimates the essential matrix
% given matching point coordinates, and the camera calibration K
% 
% Input: point correspondences
%  - p1(3,N): homogeneous coordinates of 2-D points in image 1
%  - p2(3,N): homogeneous coordinates of 2-D points in image 2
%  - K1(3,3): calibration matrix of camera 1
%  - K2(3,3): calibration matrix of camera 2
%
% Output:
%  - E(3,3) : fundamental matrix
%

    %F = fundamentalEightPoint_normalized(p1, p2);
    
    [F, inliers] =estimateFundamentalMatrix(p1(1:2, :)', p2(1:2, :)', 'Method','RANSAC')
    
    inp1 = p1(:,inliers);
    inp2 = p2(:,inliers);
    figure(4),
    showMatchedFeatures(img, img0, inp1(1:2, :)', inp2(1:2, :)');
    
    E=K2.'*F*K1;
    
end
