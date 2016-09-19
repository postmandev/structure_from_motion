function [ Cnew, Rnew ] = PnPRANSAC( X, x, K )

cnt = 0;
M = size(x,1);
p = 0.99;

threshold = 6;
N = 1;
trials = 0;
%idx = 0;

X_ = makeHomogeneous(X, '3D');
x_ = makeHomogeneous(x, '2D');

Cnew = zeros(3,1);
Rnew = eye(3);

% while N > trials
while trials < 1000
    % Choose 6 correspondences randomly
    rand_idx = randperm(M,6);
    [C, R] = LinearPnP(X(rand_idx,:), x(rand_idx,:), K);
    S = [];
    for j=1:M
        %P = K*R*[eye(3) -C];
        
        rprj = project3DTo2D( X_(j,:), K, C, R );
        e  = sqrt((x_(j,1) - rprj(1))^2 + (x_(j,2) - rprj(2))^2);
        
        if (e < threshold)
            S(end+1) = j;
        end
    end
    cntS = length(S);
    if (cnt < cntS)
        cnt = cntS;
        Rnew = R;
        Cnew = C;
    end
    
    if (cntS == M)
        break;
    end
    trials = trials + 1;
end

fprintf('inliers: %d / %d\n', cnt, M);

end

