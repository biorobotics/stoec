clc;close all;clear
load('Elif_liver.mat')
figure();
trisurf(F1,V1(:,1),V1(:,2),V1(:,3),'FaceColor','c','EdgeColor','k','EdgeAlpha',0.5,'Facealpha',0.5,'FaceLighting','gouraud','AmbientStrength',0.5);axis equal;
hold on
view(0,0)
tri2=delaunay(xs(:,1),xs(:,2));
trisurf(tri2,xs(:,1),xs(:,2),xs(:,3),cgrid(:))
shading interp
title('Experiment surface on the CAD model')

%%
close all
figure();
trisurf(F1,V1(:,1),V1(:,2),V1(:,3),'FaceColor','c','EdgeColor','k','EdgeAlpha',0.5,'Facealpha',0.5,'FaceLighting','gouraud','AmbientStrength',0.5);axis equal;
hold on
shading interp
quiver3(gt(:,1),gt(:,2),gt(:,3),normal_gt(:,1),normal_gt(:,2),normal_gt(:,3))
%%
% [trianglePoints,ia,ic]=unique(V1,'rows');
% normal_xs=zeros(length(xs),3);
% for ii=1:length(normal_xs)
%     normal_xs(ii,:)=findnormal(trianglePoints,xs(ii,:));
% end
%%
close all
figure();
trisurf(F1,V1(:,1),V1(:,2),V1(:,3),'FaceColor','c','EdgeColor','k','EdgeAlpha',0.5,'Facealpha',0.5,'FaceLighting','gouraud','AmbientStrength',0.5);axis equal;
hold on
shading interp
quiver3(xs(:,1),xs(:,2),xs(:,3),normal_xs(:,1),normal_xs(:,2),normal_xs(:,3))
%%
idx=find(cgrid<0.1);
cgrid(idx)=0.1;
def_xs=zeros(length(xs),3);
force_xs=zeros(length(xs),1);
for ii=1:length(xs)
    force_xs(ii)=0.1+rand;
    def_xs(ii,:)=xs(ii,:)-force_xs(ii)*normal_xs(ii,:)/cgrid(ii);
end
%%
close all
figure
scatter3(xs(:,1),xs(:,2),xs(:,3),'k','fill');
hold on
scatter3(def_xs(:,1),def_xs(:,2),def_xs(:,3),'r','fill');
axis equal
% save liverRegistration.mat cgrid def_xs force_xs gt normal_gt normal_xs xs F1 V1 
