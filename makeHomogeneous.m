function [ x_ ] = makeHomogeneous( x, type )
%MAKEHOMOGENOUS Summary of this function goes here
%   Detailed explanation goes here

[m,n] = size(x);

switch (type)
    case '3D'
        if (n == 3)
            x_ = [x ones(m,1)];
        else
            x_ = x;
        end
    case '2D'
        if (n == 2)
            x_ = [x ones(m,1)];
        else
            x_ = x;
        end
end

end

