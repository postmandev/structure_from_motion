function [ Cset, Rset ] = ExtractCameraPose( E )
%EXTRACTCAMERAPOSE Summary of this function goes here
%   Detailed explanation goes here

[U D V] = svd(E);

W = [0 -1 0; 1 0 0; 0 0 1];
tp = U(:,3);
tn = -tp;

Cset{1} = tp;
Rset{1} = U * W * V';

Cset{2} = tn;
Rset{2} = U * W * V';

Cset{3} = tp;
Rset{3} = U * W' * V';

Cset{4} = tn;
Rset{4} = U * W' * V';

for i=1:4,
    if (det(Rset{i}) < 0)
        Cset{i} = -Cset{i};
        Rset{i} = -Rset{i};
    end
end

end

