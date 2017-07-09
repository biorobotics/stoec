function [pointsTransformed] = computeTransformedPoints(points, Xk)

Testimated = regParamsToTransformationMatrix(Xk(1:6));

pointsTransformed =bsxfun(@plus, Testimated(1:3,1:3)*points(:,1:3)' , Testimated(1:3,4));

