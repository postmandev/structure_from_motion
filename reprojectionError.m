function [ perr ] = reprojectionError( K, C, R, X, x )
%REPROJECTIONERROR Summary of this function goes here
%   Detailed explanation goes here

rprj = project3DTo2D(X, K, C, R);

perr = sum(sqrt((rprj(:,1) - x(:,1)).^2 + (rprj(:,2) - x(:,2)).^2)) / size(x,1);
fprintf('\npixel error: %2.8f\n\n', perr);

end

