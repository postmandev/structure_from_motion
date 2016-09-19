% function sfm_test

%%
clear all;
clc;
load('initial_data.mat');

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
% Add images
for iImage = 1 : nImages
    if ~isempty(find(r_idx==iImage,1))
        continue;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Get 2D-3D correspondences
    idx1 = find(ReconX==1);
    idx2 = find(M(:,iImage)==1);
    idx = intersect(idx1, idx2);
    if length(idx) < 6
        continue;
    end
    
    X = X3D(idx,:);
    x = [Mx(idx,iImage) My(idx,iImage)];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Run PnP
    [C, R] = PnPRANSAC(X, x, K);
    disp('Nonlinear PnP');
    [C, R] = NonlinearPnP(X, x, K, C, R);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Set camera poses and reconstructed frame index
    Cr_set{end+1} = C;
    Rr_set{end+1} = R;
    r_idx(end+1) = iImage;
    V(idx, iImage) = 1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Triangulation for additional points
    disp('Adding more points');
    for iImage1 = 1 : length(r_idx)-1
        idx1 = find(ReconX~=1);
        idx2 = find(M(:,r_idx(iImage1))==1);
        idx3 = find(M(:,iImage)==1);
        idx = intersect(idx1, idx2);
        idx = intersect(idx, idx3);
        x1 = [Mx(idx,r_idx(iImage1)) My(idx,r_idx(iImage1))];
        x2 = [Mx(idx,iImage) My(idx,iImage)];
        X = LinearTriangulation(K, Cr_set{iImage1}, Rr_set{iImage1}, C, R, x1, x2);
        X = NonlinearTriangulation(K, Cr_set{iImage1}, Rr_set{iImage1}, C, R, x1, x2, X);
        
        X3D(idx,:) = X;
        ReconX(idx) = 1;
    end    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Set visibiltiy and measurements for bundle adjustment
    V_bundle = V(:,r_idx);
    Mx_bundle = Mx(:,r_idx);
    My_bundle = My(:,r_idx);
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    % Run bundle adjustment
    disp('Bundle adjustment');
    tic;
    [Cr_set, Rr_set, X] = BundleAdjustment(K, Cr_set, Rr_set, X3D, ReconX, V_bundle, Mx_bundle, My_bundle);
    toc
    % X3D(ReconX==1,:)
    figure; plot3(X(:,1),X(:,2),X(:,3),'kx');
    axis image; grid on;
    xlabel('x', 'FontSize',12);
    ylabel('y', 'FontSize',12);
    zlabel('z', 'FontSize',12);
    drawnow;
    pause;
    
end

% end

