function T = regParamsToTransformationMatrix(params)
% function T = regParamsToTransformationMatrix(params)

T = eye(4,4);
T(1:3,4) = params(1:3);
rz = params(4); ry = params(5); rx = params(6);
T(1,1:3) = [cos(ry)*cos(rz), -cos(rx)*sin(rz)+sin(rx)*sin(ry)*cos(rz), sin(rx)*sin(rz)+cos(rx)*sin(ry)*cos(rz)];
T(2,1:3) = [cos(ry)*sin(rz), cos(rx)*cos(rz)+sin(rx)*sin(ry)*sin(rz), -sin(rx)*cos(rz)+cos(rx)*sin(ry)*sin(rz)];
T(3,1:3) = [-sin(ry), sin(rx)*cos(ry), cos(rx)*cos(ry)];