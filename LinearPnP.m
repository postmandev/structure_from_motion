function [ C, R ] = LinearPnP( X, x, K )
%LINEARPNP Estimate a camera pose using linear least squares
%  
%
% (Input)
%  X and x: Nx3 and Nx2 matrices whose row represents correspondences
%           between 3D and 2D points, respectively
%
%  K:       intrinsic parameters
%
% (Output)
%  C and R: camera pose C (camera center position) and R (camera
%           orientation)
%

N = size(X,1);

X_ = makeHomogeneous(X, '3D');
x_ = makeHomogeneous(x, '2D');

% needed to convert to optical world
x_ = (inv(K) * x_')';

A = [];
for i=1:N
    xt = X_(i,:);
    z = zeros(1,4);
    p = x_(i,:);
    a = [z -xt p(2)*xt; xt z -p(1)*xt; -p(2)*xt p(1)*xt z]; 
    A = [A; a];
end

[~,b,v] = svd(A);
%semilogy(diag(b),'*-')

P = reshape(v(:,12),4,3)';
%P = inv(K) * P;

R = P(:,1:3);
t = P(:,4);

[u,~,v] = svd(R);
R = u*v';
d = eye(3);
d(3,3) = det(u*v');
R = u * d * v';
C = -inv(R) * t;

if det(R) < 0
    R = -R;
    C = -C;
end


end

