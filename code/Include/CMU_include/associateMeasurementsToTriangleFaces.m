function [faceAssociations] = associateMeasurementsToTriangleFaces(experimentDataTransformed, trianglePoints, triangleFaces)
% function [faceAssociations] = associateMeasurementsToTriangleFaces(experimentDataTransformed, trianglePoints, triangleFaces)

%experimentDataTransformed=Nx3
faceAssociations = zeros(size(experimentDataTransformed,1),1);
for i = 1:size(experimentDataTransformed,1),
    s = experimentDataTransformed(i,:);
    bestFace = 1;
    bestDist = Inf;
    for j = 1:length(triangleFaces(:,1)),
        tri = trianglePoints(triangleFaces(j,1:3),:);
        
        dist = pointTriangleDistance(tri,s');
        if (dist < bestDist)
            bestDist = dist;
            bestFace = j;
        end
    end
    faceAssociations(i,1) = bestFace;
end
