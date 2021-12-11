function angle = C_F_angle(C, F, T_new, T_prev, K)
    % C = Mx2
    % F = Mx2
    % T = Mx12
    num_candidates = size(F,1);
    
    C = K \ [C'; ones(1, num_candidates)]; % 3xM
    F = K \ [F'; ones(1, num_candidates)]; % 3xM
    angle = zeros(size(F,1),1); 
    
    for i = 1:num_candidates
        R_new_i = T_new(1:3,1:3);
        T_prev_i = reshape(T_prev(i,:), [3,4]); 
        R_prev_i = T_prev_i(1:3,1:3);
        Tro =  R_prev_i' * R_new_i ;
        angle(i) = acos(dot(C(:,i), Tro * F(:,i)) / (norm(C(:,i)) * norm(Tro * F(:,i))));
    end
end

