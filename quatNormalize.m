function [ Q ] = quatNormalize( q )
%QUATNORMALIZE Summary of this function goes here
%   Detailed explanation goes here

  Q = q/norm(q);
%mag = sqrt(q(1)^2 + q(2)^2 + q(3)^3 + q(4)^2);
%if(q(1)<0), 
%    mag=-mag; 
%end;

%Q = ([q(1) q(2) q(3) q(4)] ./ mag)';

end

