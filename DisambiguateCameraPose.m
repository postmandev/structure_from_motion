function [ C, R, X ] = DisambiguateCameraPose( Cset, Rset, Xset )
%DISAMBIGUATECAMERAPOSE Summary of this function goes here
%   Detailed explanation goes here
%
% (Input)
%  Cset and Rset: four configurations of camera centers and rotations
%  Xset: four sets of triangulated points from four camera pose configs.
%
% (Output)
%  C, R and X: the correct camera pose

best = 0;

for i=1:4,
    N = size(Xset{i},1);
    n = 0;
    for j=1:N
        if (Rset{i}(3,:)*(Xset{i}(j,:)' - Cset{i}) > 0 && Xset{i}(j,3) >= 0)
            n = n + 1;
        end
    end
    if n > best
       C = Cset{i};
       R = Rset{i};
       X = Xset{i};
       best = n;
    end
end

end

