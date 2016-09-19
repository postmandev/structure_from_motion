function draw3DPointCloud( X3D,  Cr_set, Rr_set, ptsCount, ptColor, fig )
%DRAW3DPOINTCLOUD Summary of this function goes here
%   Detailed explanation goes here

if nargin < 6
    h = findobj('type', 'figure');
    fig = length(h);
    if fig == 0
        fig = 1;
    end
end

len = length(X3D);

if nargin < 5
    ptColor = 'k.';
end

if nargin < 4
   ptsCount = len;
end

for i=1:round(len/ptsCount):len,
    if X3D(i,3) > 0
        X_(i,:) = X3D(i,:);
    end
end
fidx = find(abs(X_(:,3))<(median(X_(:,3))+40));

figure(fig);
grid on;
axis image;
hold on;
plot3(X_(fidx,1),X_(fidx,2),X_(fidx,3), ptColor);

drawCameras(Cr_set, Rr_set, 0.3);

xlabel('x', 'FontSize', 12);
ylabel('y', 'FontSize', 12);
zlabel('z', 'FontSize', 12);
drawnow;
hold off;

end

