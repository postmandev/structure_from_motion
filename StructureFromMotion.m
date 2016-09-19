function StructureFromMotion

%%
% clear all;

K = [568.996140852 0 643.21055941;
     0 568.988362396 477.982801038;
     0 0 1];
nImages = 6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Load images
for iImage = 1 : nImages
    str = sprintf('data/image%07d.bmp', iImage);
    im{iImage} = imread(str);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Load matching
Mx = []; My = []; M = [];
for iImage = 1 : nImages-1;
    str = sprintf('data/matching%d.txt', iImage);
    [mx, my, m] = LoadMatching(str, iImage, nImages);
    Mx = [Mx;mx];
    My = [My;my];
    M = [M;m];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Initialize 3D points, reconstruction index, and visibility matrix
X3D = zeros(size(M,1), 3);
ReconX = zeros(size(M,1),1);
V = zeros(size(M,1), nImages);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Exclude outliers using F matrix
for iImage1 = 1 : nImages-1
    for iImage2 = iImage1+1 : nImages
        idx1 = find(M(:,iImage1)==1);
        idx2 = find(M(:,iImage2)==1);
        idx = intersect(idx1, idx2);
        
        x1 = [Mx(idx,iImage1) My(idx,iImage1)];
        x2 = [Mx(idx,iImage2) My(idx,iImage2)];
        if size(x1,1) < 8
            continue;
        end
        [x1, x2, inlier] = GetInliersRANSAC(x1, x2);
%         sz = size(x1,1);
%         figure(1); showMatchedFeatures(im{iImage1},im{iImage2},x1(1:sz,:),x2(1:sz,:),'blend');
%         pause;
        M(idx(~inlier),iImage1) = 0;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Set initial two frames
initialframe1 = 1;
initialframe2 = 4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Get point index for two frames
idx1 = find(M(:,initialframe1)==1);
idx2 = find(M(:,initialframe2)==1);
idx = intersect(idx1, idx2);

x1 = [Mx(idx,initialframe1) My(idx,initialframe1)];
x2 = [Mx(idx,initialframe2) My(idx,initialframe2)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Get fundamental matrix and essential matrix
F = EstimateFundamentalMatrix(x1, x2)
E = EssentialMatrixFromFundamentalMatrix(F,K)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Camera pose estimation
[Cset, Rset] = ExtractCameraPose(E);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Triangulation and pose disambiguation
for i = 1 : 4
    Xset{i} = LinearTriangulation(K, zeros(3,1), eye(3), Cset{i}, Rset{i}, x1, x2);    
end
[C,R,X] = DisambiguateCameraPose(Cset, Rset, Xset);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Nonlinear triangulation
disp('Nonlinear triangulation');
X = NonlinearTriangulation(K, zeros(3,1), eye(3), C, R, x1, x2, X);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Set reconstructed frame
r_idx = [initialframe1, initialframe2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Set camera pose
Cr_set{1} = zeros(3,1);
Rr_set{1} = eye(3,3);
Cr_set{2} = C;
Rr_set{2} = R;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Set points and visibility matrix
X3D(idx,:) = X;
ReconX(idx) = 1;
V(idx, initialframe1) = 1; 
V(idx, initialframe2) = 1;

% save('initial_data.mat');

%%
% pause;
% clear all;
% clc;
%load('initial_data.mat');
% load('run1.mat');
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Add images
for iImage = 1 : nImages
    if ~isempty(find(r_idx==iImage,1))
        continue;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Get 2D-3D correspondences
    idx1 = find(ReconX==1);
    idx2 = find(M(:,iImage)==1);
    idx = intersect(idx1, idx2);
    if length(idx) < 6
        continue;
    end
    
    X = X3D(idx,:);
    x = [Mx(idx,iImage) My(idx,iImage)];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Run PnP
    disp('PnPRANSAC');
    % disp(iImage);
    [C, R] = PnPRANSAC(X, x, K);
%     disp('C = ');
%     disp('\begin{bmatrix}');
%     latex2(C, '%2.8f', 'nomath');
%     disp('\end{bmatrix} \\ ');
%     disp('R = ');
%     disp('\begin{bmatrix}');
%     latex2(R, '%2.8f', 'nomath');
%     disp('\end{bmatrix} \\');
%     
    disp('Nonlinear PnP');
    [C, R] = NonlinearPnP(X, x, K, C, R);
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Set camera poses and reconstructed frame index
    Cr_set{end+1} = C;
    Rr_set{end+1} = R;
    r_idx(end+1) = iImage;
    V(idx, iImage) = 1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Triangulation for additional points
    %disp('Adding more points');
    for iImage1 = 1 : length(r_idx)-1
        idx1 = find(ReconX~=1);
        idx2 = find(M(:,r_idx(iImage1))==1);
        idx3 = find(M(:,iImage)==1);
        idx = intersect(idx1, idx2);
        idx = intersect(idx, idx3);
        x1 = [Mx(idx,r_idx(iImage1)) My(idx,r_idx(iImage1))];
        x2 = [Mx(idx,iImage) My(idx,iImage)];
        X = LinearTriangulation(K, Cr_set{iImage1}, Rr_set{iImage1}, C, R, x1, x2);
        X = NonlinearTriangulation(K, Cr_set{iImage1}, Rr_set{iImage1}, C, R, x1, x2, X);
        X3D(idx,:) = X;
        ReconX(idx) = 1;
        V(idx, r_idx(iImage1)) = 1;
        V(idx, iImage) = 1;
    end    
    
    [ReconX, V] = deleteZPoints( ReconX, V, X3D );
    
    
%     switch(iImage),
%         case 1
%             ptColor = 'b.';
%         case 2
%             ptColor = 'r.';
%         case 3
%             ptColor = 'c.';
%         case 4
%             ptColor = 'g.';
%         case 5
%             ptColor = 'm.';
%         otherwise
%             ptColor = 'k.';
%     end
%     
%     draw3DPointCloud( X,  Cr_set, Rr_set, length(X), ptColor, 1 );
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Set visibiltiy and measurements for bundle adjustment
    V_bundle = V(:,r_idx);
    Mx_bundle = Mx(:,r_idx);
    My_bundle = My(:,r_idx);
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Run bundle adjustment
    disp('Bundle adjustment');
    tic;
    [Cr_set, Rr_set, X3D] = BundleAdjustment(K, Cr_set, Rr_set, X3D, ReconX, V_bundle, Mx_bundle, My_bundle);
    toc
    
end

%drawCameras(Cr_set, Rr_set, 0.3);


