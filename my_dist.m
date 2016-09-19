function [ d ] = my_dist( l, x )
%MY_DIST Summary of this function goes here
%   Detailed explanation goes here

d = abs(l(1)*x(1) + l(2)*x(2) + l(3)) / sqrt(l(1)^2 + l(2)^2);

end

