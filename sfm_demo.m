% Final Project - Structure from Motion
% CIS-580, Machine Perception, Spring 2015
% Preliminary Report due: Apr 27, 2015, 11:59AM
% Final Report due: May 6, 2015, 11:59AM
%
% written by Michael O'Meara & Michael Woods
%

%% Load Images
clear all;

for i=1:6
    filename = sprintf('image000000%d.bmp',i);
    im{i} = imread(filename);
end
clear i;
clear filename;

% Load matching data
clear match;
for i=1:5,
    filename = sprintf('matching%d.txt',i);
    match{i} = dlmread(filename,' ', 1, 0);
end
clear i;
clear filename;


%% Get correspondences
initialframe1 = 1;
initialframe2 = 4;
[y1, y2] = GetPointCorrespondences(match{initialframe1}, initialframe2); % match image1 with image2

%% Show images side by side
M = size(y1,1);
figure; showMatchedFeatures(im{initialframe1},im{initialframe2},y1(1:M,:),y2(1:M,:),'blend');

%% Structure from Motion pipeline
%
clear x1;
clear x2;

K = [568.996140852 0 643.21055941;
     0 568.988362396 477.982801038;
     0 0 1];
 
% for all possible pairs of images do
[x1, x2, inliers] = GetInliersRANSAC(y1, y2);

%% Show images side by side
M = size(x1,1);
figure; showMatchedFeatures(im{initialframe1},im{initialframe2},x1(1:M,:),x2(1:M,:),'blend');

%%

F = EstimateFundamentalMatrix( x1, x2 );
E = EssentialMatrixFromFundamentalMatrix( F, K );

[Cset, Rset] = ExtractCameraPose( E );

% LinearTriangulation( K, C1, R1, C2, R2, x1, x2 )
for i=1:4,
    Xset{i} = LinearTriangulation( K, zeros(3,1), eye(3), ...
        Cset{i}, Rset{i}, x1, x2 );
end

[C,R,X0] = DisambiguateCameraPose( Cset, Rset, Xset );

X = NonlinearTriangulation( K, zeros(3,1), eye(3), C, R, x1, x2, X0);

%%
M = size(X,1);
idx = randperm(M,6);
x2_ = makeHomogeneous(x2(idx,:), '2D');

% x2_ = (K \ x2_')' ./ norm((K \ x2_')');
% X_ = makeHomogeneous(X(idx,:), '3D');

[C3, R3] = LinearPnP(X(:,:), x2(:,:), K);

R * [eye(3) -C]
R3'*R3
test_x = project3DTo2D(X(idx,:), K, C3, R3)

%%

[ Cnew, Rnew ] = PnPRANSAC( X, x2, K );
[ Cnew, Rnew ] = NonlinearPnP( X, x2, K, Cnew, Rnew)
Rnew'*Rnew


%% Print C and R sets

for i=1:length(Cset),
    Cset{i}
    Rset{i}
end

%%

draw3DPointCloud( X0,  Cr_set, Rr_set, 417, 'b.', 1);
draw3DPointCloud( X,  Cr_set, Rr_set, 417, 'r.', 1);