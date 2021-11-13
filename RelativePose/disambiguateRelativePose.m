function [R,T] = disambiguateRelativePose(Rots,u3,p1,p2,K1,K2)
% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Arguments:
%   Rots -  3x3x2: the two possible rotations returned by decomposeEssentialMatrix
%   u3   -  a 3x1 vector with the translation information returned by decomposeEssentialMatrix
%   p1   -  3xN homogeneous coordinates of point correspondences in image 1
%   p2   -  3xN homogeneous coordinates of point correspondences in image 2
%   K1   -  3x3 calibration matrix for camera 1
%   K2   -  3x3 calibration matrix for camera 2
%
% Returns:
%   R -  3x3 the correct rotation matrix
%   T -  3x1 the correct translation vector
%
%   where [R|t] = T_C2_C1 = T_C2_W is a transformation that maps points
%   from the world coordinate system (identical to the coordinate system of camera 1)
%   to camera 2.
%

    M1=K1*[eye(3), zeros(3, 1)];
    
    M2_cand={};
    
    M2_cand{1}=K2*[Rots{1}, u3];
    M2_cand{2}=K2*[Rots{2}, u3];
    M2_cand{3}=K2*[Rots{1}, -u3];
    M2_cand{4}=K2*[Rots{2}, -u3];
    
    b=0;
    i=0;
    while b==false
        i=i+1;
        P=linearTriangulation(p1,p2,M1,M2_cand{i});
        P2 = [K2\M2_cand{i}; 0 0 0 1]*P;
        if all(P(3, :)>=0) && all(P2(3, :)>=0)
            b=1;
        end
    end
    MAT=K2^(-1)*M2_cand{i};
    
    R=MAT(1:3, 1:3);
    T=MAT(1:3, 4);
end
        
