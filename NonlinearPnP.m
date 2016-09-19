function [ Cnew, Rnew ] = NonlinearPnP( X, x, K, C0, R0 )
%NONLINEARPNP Summary of this function goes here
%   Detailed explanation goes here

%N = size(x,1);

% opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
%     'TolX', 1e-64, 'TolFun', 1e-64, 'MaxFunEvals', 1e+64, ...
%     'MaxIter', 100, 'Display', 'none'); % 'Display', 'iter'
opts = optimset('Algorithm', 'levenberg-marquardt', 'Display','final-detailed');

Q0 = matrix2Quaternion(R0);
Q0 = quatNormalize(Q0);

f = @(CQ)reprojError(CQ, K, X, x);
[Y, fval] = lsqnonlin(f, [C0; Q0],[],[],opts);

Cnew = Y(1:3);
Qnew = Y(4:7);

Rnew = quaternion2Matrix(Qnew);

% [w^2, x^2, y^2, z^2]
% enforce unit quaternion

end

function [e] = reprojError(CQ, K, X, x)
    X = makeHomogeneous(X, '3D');
    x = makeHomogeneous(x, '2D');
    C = CQ(1:3);
    q = CQ(4:7);
    
    R = quaternion2Matrix(q);
    
    rprj = project3DTo2D( X, K, C, R );
    
    e  = x(:,1:2) - rprj(:,1:2);
    
end

