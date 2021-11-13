function [pts_tilda, T] = normalise2dpts(pts)
% NORMALISE2DPTS - normalises 2D homogeneous points
%
% Function translates and normalises a set of 2D homogeneous points
% so that their centroid is at the origin and their mean distance from
% the origin is sqrt(2).
%
% Usage:   [pts_tilda, T] = normalise2dpts(pts)
%
% Argument:
%   pts -  3xN array of 2D homogeneous coordinates
%
% Returns:
%   pts_tilda -  3xN array of transformed 2D homogeneous coordinates.
%   T      -  The 3x3 transformation matrix, pts_tilda = T*pts
%

    pts=pts./pts(3, :); %make homog. coordinates

    mu=mean(pts, 2); %find the mean of the points in the set
    
    %find the sigma squared
    sigma_sq=0.0;
    
    for i=1:size(pts, 2)
        sigma_sq = sigma_sq + sum((pts(:, i) - mu).^2);
    end
    
    sigma_sq=sigma_sq/size(pts, 2);
    
    s=(2/sigma_sq)^(1/2);
    
    %find transformation matrix
    T=[s, 0, -s*mu(1);
       0, s, -s*mu(2);
       0, 0, 1];
   
   %find transformed points
   pts_tilda=T*pts;
   
end
    
