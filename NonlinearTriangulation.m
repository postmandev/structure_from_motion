function [ X ] = NonlinearTriangulation( K, C1, R1, C2, R2, x1, x2, X0)
%NONLINEARTRIANGULATION Summary of this function goes here
%   Detailed explanation goes here
%
% (Input)
%  C1 and R1: the first camera pose
%  C2 and R2: the second camera pose
%  x1 and x2: two Nx2 matrices whose rows represent correspondence between
%             the first and second images
%  X0: Nx3 matrix whose rows represent 3D triangulated points
%
% (Output)
%  X: Nx3 matrix whose rows represent 3D triangulated points
%
N = size(x1,1);

% opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
%     'TolX', 1e-64, 'TolFun', 1e-64, 'MaxFunEvals', 1e+64, ...
%     'MaxIter', 1e+64, 'Display', 'none'); % 'Display', 'iter'
opts = optimset('Algorithm', 'levenberg-marquardt', ...
    'TolX', 1e-64, 'TolFun', 1e-64, 'Display','none'); %'final-detailed');

% X0 = LinearTriangulation( K, C1, R1, C2, R2, x1, x2 );
% f = @(x)reprojError(x, K, C1, R1, C2, R2, x1, x2);
% [X,fval] = lsqnonlin(f, X0,[],[],opts);

X = zeros(N,3);
for i=1:N,
    f = @(x)reprojError(x, K, C1, R1, C2, R2, x1(i,:), x2(i,:));
    [x,fval] = lsqnonlin(f, X0(i,:),[],[],opts);
%     if x(3) < 0 
%         x(3) = -x(3);
%     end
    X(i,:) = x;
end

end

function [e] = reprojError(X, K, C1, R1, C2, R2, x1, x2)
    X = makeHomogeneous(X, '3D');
    
    P1 = K * R1 * [eye(3) -C1];
    P2 = K * R2 * [eye(3) -C2];
    
    u_rprj = (P1(1,:)*X')' ./ (P1(3,:)*X')';
    v_rprj = (P1(2,:)*X')' ./ (P1(3,:)*X')';
    u_rprj2 = (P2(1,:)*X')' ./ (P2(3,:)*X')';
    v_rprj2 = (P2(2,:)*X')' ./ (P2(3,:)*X')';
    
    diff  = (x1 - [u_rprj v_rprj]);
    diff2 = (x2 - [u_rprj2 v_rprj2]);

    e = [diff diff2];
    
end

