function [ F ] = EstimateFundamentalMatrix( X1, X2 )
%ESTIMATEFUNDAMENTALMATRIX Summary of this function goes here
% Compute the fundamental matrix from source x1 to destination x2
%
%    x1 are coordinates of source points
%    x2 are coordinates of destination points
%    x1/x2, each is a vector of n*2, n = 8
%
%    F is the fundamental matrix output 3x3
%   [x2(1) x2(2) 1] * F [x1(1) x1(2) 1]^T = 0
% 

n = size(X1,1);

x1 = X1(:,1);
y1 = X1(:,2);

x2 = X2(:,1);
y2 = X2(:,2);

% [u2*u1 u2*v1 u2 v2*u1 v2*v1 v2 u1 v1 1]
A = [x2.*x1 x2.*y1 x2 y2.*x1 y2.*y1 y2 x1 y1 ones(n,1)];

[U D V] = svd(A);
F = reshape(V(:,9),3,3)';

% enforce rank 2 constraint  Lecture 10 p. 34
[U D V] = svd(F,0);
F = U*diag([D(1,1) D(2,2) 0])*V';


end

