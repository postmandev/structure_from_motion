function m = projection(j, i, rt, xyz, a)

global pts2D cams pts3D nP
% symbolic projection function
% code automatically generated with maple
%j
%i



K = [a(1) a(2) a(3); 0 a(4) a(5); 0 0 1];

%% Code to fill
% Build P matrix from rt
% rt is 7 dimensional vector-the first three are camera center and the rest are quaternion
q = rt(1:4);
C = rt(5:7)';
R = quaternion2Matrix(quatNormalize(q)); % rt(4:end)

P = K * R * [eye(3) -C];

%% Get xyz
X3D = makeHomogeneous(xyz, '3D');

%% Code to fill
% Project the 3D point to the camera to produce (u,v)
% u = (P(1,:)*X3D')' ./ (P(3,:)*X3D')';
% v = (P(2,:)*X3D')' ./ (P(3,:)*X3D')';
m = project3DTo2D(X3D, K, C, R);

% Return reprojection
% m(1) = u;
% m(2) = v;

%norm(m-pts2D(i*4+j*2+1:i*4+j*2+2))
%if j == 1 && i == 1
%    e = norm(m-pts2D(i*4+j*2+1:i*4+j*2+2))
%     R
%     C
%     xyz
%end

