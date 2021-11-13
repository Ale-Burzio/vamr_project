function P = linearTriangulation(p1,p2,M1,M2)
% LINEARTRIANGULATION  Linear Triangulation
%
% Input:
%  - p1(3,N): homogeneous coordinates of points in image 1
%  - p2(3,N): homogeneous coordinates of points in image 2
%  - M1(3,4): projection matrix corresponding to first image
%  - M2(3,4): projection matrix corresponding to second image
%
% Output:
%  - P(4,N): homogeneous coordinates of 3-D points

    P=[];

    for i=1:size(p1, 2)
        p1_s=cross2Matrix(p1(:, i)); %create skew matrix of p1
        p2_s=cross2Matrix(p2(:, i));
        
        p1_M=p1_s*M1; %multiply for projection matrix
        p2_M=p2_s*M2;
        
        A=[p1_M; p2_M]; %create A matrix stacking the two matrixes
        
        %solve A*Pi=0
        [~, ~, V]=svd(A); 
        
        Pi=V(:, end); %last column of V
        
        P=[P, Pi]; %stack Pi to the general matrix P containing all W points
    end
    
    P=P./P(4, :) %compute the homogeneous coords dividing for last row
end
