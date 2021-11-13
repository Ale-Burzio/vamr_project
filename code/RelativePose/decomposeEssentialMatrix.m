function [R,u3] = decomposeEssentialMatrix(E)
% Given an essential matrix, compute the camera motion, i.e.,  R and T such
% that E ~ T_x R
% 
% Input:
%   - E(3,3) : Essential matrix
%
% Output:
%   - R(3,3,2) : the two possible rotations
%   - u3(3,1)   : a vector with the translation information

    %W is given as this
    W=[0, -1, 0;
       1, 0, 0;
       0, 0, 1];
    
    [U, ~, V]=svd(E);
    
    u3=U(:, 3); %u3 is the last colomn of U
    
    %the two possible rot. matrices
    R{1}=U*W*V.';
    
    R{2}=U*W.'*V.';
    
    %check that matrices are valid (det=1)
    if det(R{1})<0
        R{1}=-R{1};
    end
    
    if det(R{2})<0
        R{2}=-R{2};
    end
    
end
    
    
    
    
    
    
    
    
    
