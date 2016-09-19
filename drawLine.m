function [x,y] = drawLine( I,a,b,c )

[row,col,~] = size(I);

x = 1:col;
y = -c/b - a/b * x;

end

