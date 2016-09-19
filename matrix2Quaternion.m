function [ q ] = matrix2Quaternion( R )
%TOQUATERNIONFROMROT Summary of this function goes here
%   Convert a rotation matrix to a quaternion
% (Input)
%    R: 3x3 rotation matrix
%
% (Output)
%    q: a quaternion of the form [w, xi, yj, zk]


w = sqrt(1.0 + R(1,1) + R(2,2) + R(3,3)) / 2.0;
w4 = (4.0 * w);
x = (R(3,2) - R(2,3)) / w4 ;
y = (R(1,3) - R(3,1)) / w4 ;
z = (R(2,1) - R(1,2)) / w4 ;

q = [w, x, y, z]';    

% [v,d] = eig(R - eye(3));
% 
% d = diag(abs(d));
% [s, idx] = sort(d);
% 
% column = v(:, idx(1));
% 
% two_cos_theta   = trace(R) - 1;
% two_sin_theta_v = [R(3,2)-R(2,3), R(1,3)-R(3,1), R(2,1)-R(1,2)]';
% two_sin_theta   = column'*two_sin_theta_v;
% 
% theta = atan2(real(two_sin_theta), real(two_cos_theta));
% 
% q = [cos(theta/2); column*sin(theta/2)];


end

