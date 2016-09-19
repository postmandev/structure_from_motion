function drawCamera(C, R, scaleFactor)
%  C = 3x1 camera position
%  R = 3x3 camera rotation matrix
% scaleFactor = Float camera drawing scale factor. Larger -> bigger camera

lineWidth = 1;
f = scaleFactor*2;
w = scaleFactor*2;
h = scaleFactor*2;

V = [...
     0 0 0 f -w/2  w/2 w/2 -w/2
     0 0 f 0 -h/2 -h/2 h/2  h/2
     0 f 0 0  w/2  w/2 w/2  w/2 ];

V = R' * (V - repmat(C,1,size(V,2))); 

% Draw the basis vectors:
plot3(V(1,[1 4]),V(2,[1 4]),V(3,[1 4]), '-r', 'LineWidth', lineWidth);
plot3(V(1,[1 3]),V(2,[1 3]),V(3,[1 3]), '-g', 'LineWidth', lineWidth);
plot3(V(1,[1 2]),V(2,[1 2]),V(3,[1 2]), '-b', 'LineWidth', lineWidth);

% Draw the camera's screen as a mesh square:
cameraCorners = [
    V(1,5),V(2,5),V(3,5); ...
    V(1,6),V(2,6),V(3,6); ...
    V(1,7),V(2,7),V(3,7); ...
    V(1,8),V(2,8),V(3,8) ...
];
faceAlpha = 0.33;
faceColor = [0.7 0.7 0.7];

h = patch(cameraCorners(:,1), cameraCorners(:,2), cameraCorners(:,3), [0,0,0]);
set(h,'FaceAlpha', faceAlpha);
set(h,'FaceColor', faceColor);

% Sides:
%1 
h = patch(V(1,[1 5 6]), V(2,[1 5 6]), V(3,[1 5 6]), [0,0,0]);
set(h,'FaceAlpha', faceAlpha);
set(h,'FaceColor', faceColor);
%2
h = patch(V(1,[1 6 7]), V(2,[1 6 7]), V(3,[1 6 7]), [0,0,0]);
set(h,'FaceAlpha', faceAlpha);
set(h,'FaceColor', faceColor);
%3 
h = patch(V(1,[1 7 8]), V(2,[1 7 8]), V(3,[1 7 8]), [0,0,0]);
set(h,'FaceAlpha', faceAlpha);
set(h,'FaceColor', faceColor);
% %4 
h = patch(V(1,[1 8 5]), V(2,[1 8 5]), V(3,[1 8 5]), [0,0,0]);
set(h,'FaceAlpha', faceAlpha);
set(h,'FaceColor', faceColor);

% Draw the camera frustum:
plot3(V(1,[1 5]),V(2,[1 5]),V(3,[1 5]), ['-', faceColor], 'LineWidth', lineWidth);
plot3(V(1,[1 6]),V(2,[1 6]),V(3,[1 6]), ['-', faceColor], 'LineWidth', lineWidth);
plot3(V(1,[1 7]),V(2,[1 7]),V(3,[1 7]), ['-', faceColor], 'LineWidth', lineWidth);
plot3(V(1,[1 8]),V(2,[1 8]),V(3,[1 8]), ['-', faceColor], 'LineWidth', lineWidth);

plot3(V(1,[5 6 7 8 5]),V(2,[5 6 7 8 5]),V(3,[5 6 7 8 5]), '-k', 'LineWidth', 2*lineWidth);

end
