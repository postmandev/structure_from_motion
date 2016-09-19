function [ X3D ] = LinearTriangulation( K, C1, R1, C2, R2, x1, x2 )
%LINEARTRIANGULATION Summary of this function goes here
%   Detailed explanation goes here
%
% (Input)
%  C1 and R1: the first camera pose
%  C2 and R2: the second camera pose
%  x1 and x2: two Nx2 matrices of correspondence points 
%             between 1st and 2nd images
%
% (Output)
%  X: Nx3 matrix whose rows represent 3D triangulation points.
%  
[M,N] = size(x1);

P1 = K * R1 * [eye(3) -C1];
P2 = K * R2 * [eye(3) -C2];

% Convert to homogeneous Correspondences
X1 = makeHomogeneous(x1, '2D');
X2 = makeHomogeneous(x2, '2D');


X3D = zeros(M,3); % 3D points

for i=1:M,
    skew1 = Vec2Skew(X1(i,:));
    skew2 = Vec2Skew(X2(i,:));
    A = [skew1*P1; skew2*P2];
    [~,~,v] = svd(A);
    x = v(:,end) / v(end,end);
    X3D(i,:) = x(1:3)';
    
end


end

