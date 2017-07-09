function normal=findnormal(V1,gt)
%
%
%     [faceAssociations] = associateMeasurementsToTriangleFaces(grid_points, V,F);
%     normal=N(faceAssociations,:);
% %     p1=V(F(faceAssociations,1),:);
% %     p2=V(F(faceAssociations,2),:);
% %     p3=V(F(faceAssociations,3),:);
% %     normal=cross(p2-p1,p3-p1);normal=normal/norm(normal);


idx=knnsearch(V1,gt,'k',3);
p1=V1(idx(:,1),:);
p2=V1(idx(:,2),:);
p3=V1(idx(:,3),:);
normal=cross(p2-p1,p3-p1);normal=normal/norm(normal);

if normal(:,3)<0
    normal=-normal;
end
end

