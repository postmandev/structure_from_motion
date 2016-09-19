function [ y1, y2, idx ] = GetInliersRANSAC( x1, x2 )
%GETINLIERRANSAC Summary of this function goes here
%   Reject outlier correspondences
n = 0;
M = size(x1,1);
p = 0.99;

threshold = 6; % 1e-32;
N = 1;
trials = 0;
idx = [];

x1 = makeHomogeneous(x1, '2D');
x2 = makeHomogeneous(x2, '2D');

%while N > trials
while trials < 1000
    % Choose 8 correspondences, xhat1 and xhat2, randomly
    rand_idx = randperm(M,8);
    F = EstimateFundamentalMatrix(x1(rand_idx,:), x2(rand_idx,:));
    S = [];
    for j=1:M
        if abs(my_dist(F'*x2(j,:)',x1(j,:)') + my_dist(F*x1(j,:)', x2(j,:)')) < threshold
            S(end+1) = j;
        end
    end
    nS = length(S);
    if (n < nS)
        n = nS;
        idx = S;
        %nS
        %abs(my_dist(F'*x2(j,:)',x1(j,:)') + my_dist(F*x1(j,:)', x2(j,:)'))
        %e = (M-nS) / M; % ratio of outliers
        %N = log(1-p) / log(1-(1-e)^8); % Leature slides 13, pg. 42
    end
    
    if (nS == M)
        break;
    end
    trials = trials + 1;
end

y1 = x1(idx,1:2);
y2 = x2(idx,1:2);

