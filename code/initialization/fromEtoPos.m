function [R,T] = fromEtoPos(E, p1, p2, K)
%% decompose

[U,~,V] = svd(E);

% Translation
u3 = U(:,3);

% Rotations
W = [0 -1 0; 1 0 0; 0 0 1];
Rots(:,:,1) = U*W*V.';
Rots(:,:,2) = U*W.'*V.';

if det(Rots(:,:,1))<0
    Rots(:,:,1)=-Rots(:,:,1);
end

if det(Rots(:,:,2))<0
    Rots(:,:,2)=-Rots(:,:,2);
end

if norm(u3) ~= 0
    u3 = u3/norm(u3);
end

%% disambiguate

M1 = K * eye(3,4); % Projection matrix of camera 1

total_points_in_front_best = 0;
for iRot = 1:2
    R_C2_C1_test = Rots(:,:,iRot);
    
    for iSignT = 1:2
        T_C2_C1_test = u3 * (-1)^iSignT;
        
        M2 = K * [R_C2_C1_test, T_C2_C1_test];
        P_C1 = linearTriangulation(p1,p2,M1,M2);
        
        % project in both cameras
        P_C2 = [R_C2_C1_test T_C2_C1_test] * P_C1;
        
        num_points_in_front1 = sum(P_C1(3,:) > 0);
        num_points_in_front2 = sum(P_C2(3,:) > 0);
        total_points_in_front = num_points_in_front1 + num_points_in_front2;
              
        if (total_points_in_front > total_points_in_front_best)
            % Keep the rotation that gives the highest number of points
            % in front of both cameras
            R = R_C2_C1_test;
            T = T_C2_C1_test;
            total_points_in_front_best = total_points_in_front;
        end
    end
end
end

