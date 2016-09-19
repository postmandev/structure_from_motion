function [ ReconX, V ] = deleteZPoints( ReconX, V, X3D )
%DELETEZPOINTS Summary of this function goes here
%   Detailed explanation goes here

for i=1:length(X3D),
    if X3D(i,3) < 0
        V(i,:) = 0;
        ReconX(i) = 0;
    end
end

end

