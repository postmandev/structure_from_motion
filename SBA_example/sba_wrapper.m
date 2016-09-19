function [Cr_set, Rr_set, X] = sba_wrapper(Mx, My, Cset, Rset, V, X, K)
% cP: camera poses
% X: 3D points
% measurements: 2D points
% K: intrinsic parameter

global pts2D cams pts3D nP

nFrames = length(Cset);
nFeatures = length(X);
nP = nFeatures;

r0 = [];    cams = [];  pts2D = [];
spmask=sparse([], [], [], nFeatures, nFrames);
    
% Concaternate all camera poses into cams vector
for i = 1 : nFrames   
    q = matrix2Quaternion(Rset{i}); % inv(K)*
    q = quatNormalize(q);
    r0 = [r0;q'];
    cams = [cams; q' Cset{i}'];
end

[ncams, cnp] = size(cams);

npts = 0;
% Concatenate all 2D measurements into pts2D

pts2D = [];
count = 0;
for i = 1 : nFeatures
    pts2 = [];
    for j = 1 : nFrames
    
        if V(i,j) == 0
            continue;
        end
        pts2 = [pts2 Mx(i,j) My(i,j)];
        count = count + 1;
    end
    pts2D = [pts2D pts2];
end

pts3D = X;
[pmp, pnp]=size(pts3D);

% ignore this
r0=reshape(r0', 1, numel(r0));

% Calibration parameters
cal = [K(1,1) K(1,2), K(1,3), K(2,2), K(2,3)];

%opts=[1E-1, 0, 0, 1E-5, 0.0];
opts=[1E-05, 1E-12, 1E-12, 1E-12, 0.0];
p0=[reshape(cams', 1, nFrames*cnp) reshape(pts3D', 1, pmp*pnp)];

if isreal(p0) ~= 1
    k = 1;
end

for i = 1 : nFrames
    camera = p0(7*(i-1)+1:7*i)';
    Rr_set{i} = quaternion2Matrix(camera(1:4));
    Cr_set{i} = camera(5:end);
end

X = [];
for i = 1 : nFeatures
    pts3 = p0(7*nFrames+3*(i-1)+1:7*nFrames+3*i);
    X = [X;pts3];
end

[ret, p, info]=sba(nFeatures, 0, nFrames, 1, V, p0, 7, 3, pts2D, 2, 'projection', '', 1e+2, 1, opts, 'mot', cal);

% Retrieve paramters
for i = 1 : nFrames
    camera = p(7*(i-1)+1:7*i)';
    Rr_set{i} = quaternion2Matrix(camera(1:4));
    Cr_set{i} = camera(5:end);
end

X = [];
for i = 1 : nFeatures
    pts3 = p(7*nFrames+3*(i-1)+1:7*nFrames+3*i);
    X = [X;pts3];
end

end
