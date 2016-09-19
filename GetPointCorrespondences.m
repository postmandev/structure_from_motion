function [ x1, x2 ] = GetPointCorrespondences( input, match )
%GET Summary of this function goes here
%   Detailed explanation goes here

[M,N] = size(input);
% x1 = zeros(M,2);
% x2 = zeros(M,2);
x1 = [];
x2 = [];
for i=1:M,
    % n = input(i,1);
    for j=7:N,
        if (input(i,j) == match)
            x1 = [x1; input(i,5:6)];
            x2 = [x2; input(i,j+1:j+2)];
            break;
        end
    end
end

