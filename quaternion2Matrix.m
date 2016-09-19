function [ R ] = quaternion2Matrix( Q )
%TOROTATIONMATRIX 
% Create a rotation matrix using quaternions

w = Q(1);
x = Q(2);
y = Q(3);
z = Q(4);

Tx = 2*x;
Ty = 2*y;
Tz = 2*z;

Twx = Tx*w;
Twy = Ty*w;
Twz = Tz*w;

Txx = Tx*x;
Txy = Ty*x;
Txz = Tz*x;

Tyy = Ty*y;
Tyz = Tz*y;
Tzz = Tz*z;

R = zeros(3);
R(1,1) = 1 - (Tyy + Tzz);
R(1,2) = Txy - Twz;
R(1,3) = Txz + Twy;
R(2,1) = Txy + Twz;
R(2,2) = 1 - (Txx + Tzz);
R(2,3) = Tyz - Twx;
R(3,1) = Txz - Twy;
R(3,2) = Tyz + Twx;
R(3,3) = 1 - (Txx + Tyy);

[u,d,v] = svd(R);
R = u * v';
if (det(R)<0)
    R = -R;
end

end

