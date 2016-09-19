function R = getRotationMatrix( x,y,z )

R = zeros(3);
R(1,1) = cos(y)*cos(z);
R(1,2) = sin(x)*sin(y)*cos(z) + cos(x)*sin(z);
R(1,3) = -cos(x)*sin(y)*cos(z) + sin(x)*sin(z);
R(2,1) = -cos(y)*sin(z);
R(2,2) = -sin(x)*sin(y)*sin(z) + cos(x)*cos(z);
R(2,3) = cos(x)*sin(y)*sin(z) + sin(x)*cos(z);
R(3,1) = sin(y);
R(3,2) = -sin(x)*cos(y);
R(3,3) = cos(x)*cos(y);


% R(1,1) = cos(x)*cos(z) - sin(x)*sin(y)*sin(z);
% R(1,2) = -cos(z)*sin(x)*sin(y) - cos(x)*sin(z);
% R(1,3) = cos(y)*sin(x);
% R(2,1) = cos(y)*sin(z);
% R(2,2) = cos(y)*cos(z);
% R(2,3) = sin(y);
% R(3,1) = -cos(x)*sin(y)*sin(z) - cos(z)*sin(x);
% R(3,2) = -cos(x)*cos(z)*sin(y)+sin(x)*sin(z);
% R(3,3) = cos(x)*cos(y);

end

