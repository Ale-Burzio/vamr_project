function F = fundamentalEightPoint(p1,p2)
% fundamentalEightPoint  The 8-point algorithm for the estimation of the fundamental matrix F
%
% The eight-point algorithm for the fundamental matrix with a posteriori
% enforcement of the singularity constraint (det(F)=0).
% Does not include data normalization.
%
% Reference: "Multiple View Geometry" (Hartley & Zisserman 2000), Sect. 10.1 page 262.
%
% Input: point correspondences
%  - p1(3,N): homogeneous coordinates of 2-D points in image 1
%  - p2(3,N): homogeneous coordinates of 2-D points in image 2
%
% Output:
%  - F(3,3) : fundamental matrix

    Q=[];

    for i=1:size(p1, 2)
        p_kron=kron(p1(:, i), p2(:, i)); %create kron product of the two points
        Q=[Q; p_kron.']; %stack in the Q matrix
    end
    
    %solve Q*vec_F=0
    [~, ~, V]=svd(Q);
        
    vec_F=V(:, end);
    
    %reshape the vec_F to the true 3x3 matrix F
    F=reshape(vec_F, 3, 3);
    
    %impose det(F)=0, to make sure all epipolar lines intersect at epipole
    [U, S, V]=svd(F);
    
    S(3, 3)=0;
    
    F=U*S*V.';
    
    
