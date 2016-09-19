
%% Output of RANSAC

M = size(x1,1);
figure; showMatchedFeatures(im{initialframe1},im{initialframe2},x1(1:M,:),x2(1:M,:),'blend');



%% Draw epipolar lines

figure; imshow(image{4});
hold on;

for i=1:100,
    x_0 = [x1(i,:) 1];
    %x_1 = [x2(i,:) 1];
    
    l1 = F*x_0';
    
    [x,y] = drawLine(image{2}, l1(1),l1(2),l1(3));
    
    plot(x,y, 'r', 'LineWidth', 1);
    %plot(x_1(1), x_1(2), 'bo');
    %plot(x_2(1), x_2(2), 'co');

end
hold off;


%% Draw 3D point cloud

len = length(X3D);
numPoints = len;

for i=1:round(len/numPoints):len,
    X_(i,:) = X3D(i,:); %3D(ReconX==1,:);
end

% Filter extreme values
fidx = find(abs(X_(:,3))<(median(X_(:,3))+40));

figure;
xlabel('x', 'FontSize', 12);
ylabel('y', 'FontSize', 12);
zlabel('z', 'FontSize', 12);
grid on;
hold on;
plot3(X_(fidx,1),X_(fidx,2),X_(fidx,3), 'k.');
axis equal;
for i=1:length(Cr_set),
    drawCamera(Cr_set{i}, Rr_set{i}, 0.3);
end
hold off;

%% Project 3D points back to image with 2D points and show error

for j=1:length(Rr_set),
    i = r_idx(j);
    test_x = project3DTo2D(X3D(V(:,i)==1,:), K, Cr_set{j}, Rr_set{j});
    perr = sum(sqrt((test_x(:,1) - Mx(V(:,i)==1,i)).^2 + ...
        (test_x(:,2) - My(V(:,i)==1,i)).^2)) / size(Mx(V(:,i)==1,:),1);
    figure(i);
    J = im{i};
    im_sz = size(J);
    str = sprintf('Image: %d\npixel error: %2.8f\n', i, perr);
    textColor = [255, 255, 255];
    textLocation = [100,(im_sz(1)-100)];
    textInserter = vision.TextInserter(str, 'Color', textColor, 'FontSize', 28, 'Location', textLocation); 
    imshow(step(textInserter, J));
    hold on;
    plot(Mx(V(:,i)==1,i), My(V(:,i)==1,i), 'ro', test_x(:,1), test_x(:,2), 'bx');
    legend({'Sift Detected','Reprojected'},'FontSize',12,'Location','southeast');
    
    
    drawnow;
    hold off;

    fprintf('pixel error: %2.8f\n', perr);
end

%% Projecting 3D points back to first image with 2D points

test_x = project3DTo2D(X, K, zeros(3,1), eye(3));

figure; 
imshow(im{initialframe1});
hold on;
plot(x1(:,1), x1(:,2), 'ro', test_x(:,1), test_x(:,2), 'bx');
legend({'Sift Detected','Reprojected'},'FontSize',12,'Location','southeast');
% plot(control_x(:,1), control_x(:,2), 'gx');
drawnow;
hold off;

perr = sum(sqrt((test_x(:,1) - x1(:,1)).^2 + (test_x(:,2) - x1(:,2)).^2)) / size(x1,1);
fprintf('\npixel error: %2.8f\n\n', perr);

%% Projecting 3D points back to 2D points using 2nd image correspondence pts

test_x = project3DTo2D(X, K, C, R);

figure(2); 
imshow(im{initialframe2});
hold on;
plot(x2(:,1), x2(:,2), 'ro', test_x(:,1), test_x(:,2), 'bx');
legend({'Sift Detected','Reprojected'},'FontSize',12,'Location','southeast');
% plot(control_x(:,1), control_x(:,2), 'gx');
drawnow;
hold off;

perr = sum(sqrt((test_x(:,1) - x2(:,1)).^2 + (test_x(:,2) - x2(:,2)).^2)) / size(x2,1);
fprintf('\npixel error: %2.8f\n\n', perr);


%%
test_x = project3DTo2D(X, K, Cnew, Rnew);

figure; 
imshow(im{initialframe2});
hold on;
plot(x2(:,1), x2(:,2), 'ro', test_x(:,1), test_x(:,2), 'bx');
legend({'Sift Detected','Reprojected'},'FontSize',12,'Location','southeast');
hold off;

perr = sum(sqrt((test_x(:,1) - x2(:,1)).^2 + (test_x(:,2) - x2(:,2)).^2)) / size(x1,1);
fprintf('\npixel error: %2.8f\n\n', perr);

%%

figure;
xlabel('x', 'FontSize', 12);
ylabel('y', 'FontSize', 12);
zlabel('z', 'FontSize', 12);
grid on;
hold on;

for j=1:length(Cr_set) % :-1:1
    Xs = saved_points{j};
    if nnz(Xs) == 0
        disp(j);
        continue;
    end
    fidx = find(abs(Xs(:,3))<(median(Xs(:,3))+40));
    switch(j),
        case 1
            ptColor = 'b.';
        case 2
            ptColor = 'r.';
        case 3
            ptColor = 'c.';
        case 4
            ptColor = 'g.';
        case 5
            ptColor = 'm.';
        otherwise
            ptColor = 'k.';
    end
    plot3(Xs(:,1),Xs(:,2),Xs(:,3), ptColor);
    drawnow;
    pause;
    Xs = [];
end

axis image;
for i=1:length(Cr_set),
    drawCamera(Cr_set{i}, Rr_set{i}, 0.3);
end
hold off;

