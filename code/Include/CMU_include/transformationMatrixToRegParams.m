function params = transformationMatrixToRegParams(T)
% function params = transformationMatrixToRegParams(T)

params(1:3,1) = T(1:3,4);
params(4,1) = atan2(T(2,1), T(1,1));
params(5,1) = atan2(-T(3,1), sqrt(T(3,2)^2 + T(3,3)^2));
params(6,1) = atan2(T(3,2), T(3,3));
