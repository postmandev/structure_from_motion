function [ E ] = EssentialMatrixFromFundamentalMatrix( F, K )
%ESSENTIALMATRIXFROMFUNDAMENTALMATRIX Summary of this function goes here
%   Calculate the essential matrix from the estimated fundamental matrix F
%   and calibration matrix K


E = K'*F*K;

[U D V] = svd(E);
E = U*diag([1 1 0])*V';


end

