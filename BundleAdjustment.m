function [Cset,Rset,X] = BundleAdjustment(K,Cset,Rset,X,ReconX,V,Mx,My)

global nImages nPoints V_ Mx_ My_ K_;


X_ = X(ReconX==1,:);
V_ = V(ReconX==1,:);
K_ = K;

nImages = length(Cset);
nPoints = size(X_,1);
Mx_ = Mx(ReconX==1,:);
My_ = My(ReconX==1,:);

x = [];
for i=1:length(Cset),
    x = [x; matrix2Quaternion(Rset{i}); Cset{i} ];
end

for i=1:nPoints,
    x = [x; X_(i,:)'];
end

f = BundleReprojection(x);
perr = sum(f.^2)/length(f)*2;
fprintf('pixel error before optimization: %2.8f\n', perr);

use_sba = 1;

if use_sba == 1
    X1 = X_;
    [Cset, Rset, X_] = sba_wrapper(Mx_(1:nPoints,:), My_(1:nPoints,:), ...
        Cset, Rset, V_(1:nPoints,:), X_(1:nPoints,:), K);

    x = [];
    for i=1:length(Cset),
        x = [x; matrix2Quaternion(Rset{i}); Cset{i}];
    end
    for i=1:nPoints, %length(X_),
        x = [x; X_(i,:)'];
    end
else 
    opts = optimset('Algorithm', 'levenberg-marquardt', 'Display','iter-detailed');
    x = lsqnonlin(@BundleReprojection, x, [],[], opts);
end

f = BundleReprojection(x);
perr = sum(f.^2)/length(f)*2;
fprintf('pixel error after optimization: %2.8f\n\n', perr);

for i=1:length(Cset),
    Cset{i} = x(7*(i-1)+5:7*(i-1)+7);
    q = x(7*(i-1)+1:7*(i-1)+4);
    Rset{i} = quaternion2Matrix(quatNormalize(q));
end

for i=1:nPoints,
    X_(i,:) = x(7*nImages+3*(i-1)+1:7*nImages+3*(i-1)+3)';
end


X(ReconX==1,:) = X_;


end

function [f] = BundleReprojection(x)

    global nImages nPoints V_ Mx_ My_ K_;
    
    for i=1:nImages,
        q = x(7*(i-1)+1:7*(i-1)+4);
        C = x(7*(i-1)+5:7*(i-1)+7);
        
        R = quaternion2Matrix(quatNormalize(q));
        P{i} = K_ * R * [eye(3) -C];
    end
    for i=1:nPoints,
        X(i,:) = x(7*nImages+3*(i-1)+1:7*nImages+3*(i-1)+3)';
    end

    f = [];
    for i=1:nPoints, % size(V_,1),
        for j=1:size(V_,2),
            if (V_(i,j)~=1)
                continue;
            end
            proj = P{j}*[X(i,:)'; 1];

            f(end+1) = proj(1)/proj(3) - Mx_(i,j);
            f(end+1) = proj(2)/proj(3) - My_(i,j);
            
        end
    end

end

