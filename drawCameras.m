function drawCameras( Cr_set, Rr_set, sz )
%DRAWCAMERAS Summary of this function goes here
%   Detailed explanation goes here

if nargin < 3
    sz = 0.3;
end

hold on;
for i=1:length(Cr_set),
    drawCamera(Cr_set{i}, Rr_set{i}, sz);
end
 hold off;

end

