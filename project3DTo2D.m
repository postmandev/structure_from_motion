function [ X2D ] = project3DTo2D( X3D, K, C, R )
%PROJECT3DTO2D Summary of this function goes here
%   Detailed explanation goes here

P = K * R * [eye(3) -C];
X3D = makeHomogeneous(X3D, '3D'); 

u_rprj = (P(1,:)*X3D')' ./ (P(3,:)*X3D')';
v_rprj = (P(2,:)*X3D')' ./ (P(3,:)*X3D')';

X2D = [u_rprj, v_rprj];

end

