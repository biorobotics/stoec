
[F1,V1,N1]=stlread('2balls_tissue.stl');
[F2,V2]=stlread('2balls_tumor.stl');
load('GT_2balls.mat')
gt=xs;
c=cgrid(:);

Xcad2gt=[ -101.5079   89.9249   89.4183   -3.1291    0.0235    1.5683];
V1=computeTransformedPoints(V1,Xcad2gt)';
V2=computeTransformedPoints(V2,Xcad2gt)';
Tcad2gt=regParamsToTransformationMatrix(Xcad2gt);
N1=(Tcad2gt(1:3,1:3)*N1')';

close all
figure();
 trisurf(F1,V1(:,1),V1(:,2),V1(:,3),'FaceColor','c','EdgeColor','k','EdgeAlpha',0.5,'Facealpha',0.5,'FaceLighting','gouraud','AmbientStrength',0.5);axis equal;
hold on
trisurf(F2,V2(:,1),V2(:,2),V2(:,3),'FaceColor','r','EdgeColor','k','EdgeAlpha',0.5,'Facealpha',0.5,'FaceLighting','gouraud','AmbientStrength',0.5);axis equal;
view(0,0)
tri2=delaunay(gt(:,1),gt(:,2));
trisurf(tri2,gt(:,1),gt(:,2),gt(:,3),c)

shading interp
%%
%Extract CAD surface
figure;
V1surf=V1(V1(:,3)>-61.5,:);
% scatter3(V1surf(:,1),V1surf(:,2),V1surf(:,3),'k.');
triV1=delaunay(V1surf(:,1),V1surf(:,2));
trisurf(triV1,V1surf(:,1),V1surf(:,2),V1surf(:,3))
axis equal
%Remove outliers below the surface

%%



[trianglePoints,ia,ic]=unique(V1,'rows');
normal_gt=zeros(length(gt),3);
for ii=1:length(gt)
    normal_gt(ii,:)=findnormal(trianglePoints,gt(ii,:));
end
%%

close all

figure();
 trisurf(F1,V1(:,1),V1(:,2),V1(:,3),'FaceColor','c','EdgeColor','k','EdgeAlpha',0.5,'Facealpha',0.5,'FaceLighting','gouraud','AmbientStrength',0.5);axis equal;
hold on
%  
% 
% tri2=delaunay(gt(:,1),gt(:,2));
% trisurf(tri2,gt(:,1),gt(:,2),gt(:,3))
 shading interp

 quiver3(gt(:,1),gt(:,2),gt(:,3),normal_gt(:,1),normal_gt(:,2),normal_gt(:,3))
 %%
 
 

[trianglePoints,ia,ic]=unique(V1,'rows');
normal_xs=zeros(length(xs),3);
for ii=1:length(normal_xs)
    normal_xs(ii,:)=findnormal(trianglePoints,xs(ii,:));
end
%%


close all

figure();
 trisurf(F1,V1(:,1),V1(:,2),V1(:,3),'FaceColor','c','EdgeColor','k','EdgeAlpha',0.5,'Facealpha',0.5,'FaceLighting','gouraud','AmbientStrength',0.5);axis equal;
hold on
%  
% 
% tri2=delaunay(gt(:,1),gt(:,2));
% trisurf(tri2,gt(:,1),gt(:,2),gt(:,3))
 shading interp

 quiver3(xs(:,1),xs(:,2),xs(:,3),normal_xs(:,1),normal_xs(:,2),normal_xs(:,3))


%%
idx=find(cgrid<0.1);
cgrid(idx)=min(c);
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
    
%% ********** for 3balls data

[F1,V1,N1]=stlread('3balls_tissue.stl');
[F2,V2]=stlread('3balls_tumor.stl');


 load('siliconFeb3balls.mat')

% load('for_elif_experiment_2_11_2016_whole_silicon_three_balls.mat')
% gt=datau;
% t_shift=min(datau);
% gt=bsxfun(@minus, gt,[t_shift(1),t_shift(2),0]);


Xcad2gt=[ 100-1,150+0.017,80+7.8601,   pi/2+0.0065,    0.0235-0.024,    1.5683+0.0094];
V1=computeTransformedPoints(V1,Xcad2gt)';
V2=computeTransformedPoints(V2,Xcad2gt)';
Tcad2gt=regParamsToTransformationMatrix(Xcad2gt);
N1=(Tcad2gt(1:3,1:3)*N1')';

close all
figure();
axis equal
tri2=delaunay(gt(:,1),gt(:,2));
trisurf(tri2,gt(:,1),gt(:,2),gt(:,3),c)
  shading interp
hold on
 trisurf(F1,V1(:,1),V1(:,2),V1(:,3),'FaceColor','c','EdgeColor','k','EdgeAlpha',0.5,'Facealpha',0.5);axis equal;
hold on
 trisurf(F2,V2(:,1),V2(:,2),V2(:,3),'FaceColor','r','EdgeColor','k','EdgeAlpha',0.5,'Facealpha',0.5);axis equal;


%%
trianglePoints,ia,ic]=unique(V1,'rows');
normal_gt=zeros(length(gt),3);
for ii=1:length(gt)
    normal_gt(ii,:)=findnormal(trianglePoints,gt(ii,:));
end
%%

close all

figure();
 trisurf(F1,V1(:,1),V1(:,2),V1(:,3),'FaceColor','c','EdgeColor','k','EdgeAlpha',0.5,'Facealpha',0.5,'FaceLighting','gouraud','AmbientStrength',0.5);axis equal;
hold on
%  
% 
% tri2=delaunay(gt(:,1),gt(:,2));
% trisurf(tri2,gt(:,1),gt(:,2),gt(:,3))
 shading interp

 quiver3(gt(:,1),gt(:,2),gt(:,3),normal_gt(:,1),normal_gt(:,2),normal_gt(:,3))
 %%
 
 

[trianglePoints,ia,ic]=unique(V1,'rows');
normal_xs=zeros(length(xs),3);
for ii=1:length(normal_xs)
    normal_xs(ii,:)=findnormal(trianglePoints,xs(ii,:));
end
%%


close all

figure();
 trisurf(F1,V1(:,1),V1(:,2),V1(:,3),'FaceColor','c','EdgeColor','k','EdgeAlpha',0.5,'Facealpha',0.5,'FaceLighting','gouraud','AmbientStrength',0.5);axis equal;
hold on
%  
% 
% tri2=delaunay(gt(:,1),gt(:,2));
% trisurf(tri2,gt(:,1),gt(:,2),gt(:,3))
 shading interp

 quiver3(xs(:,1),xs(:,2),xs(:,3),normal_xs(:,1),normal_xs(:,2),normal_xs(:,3))


%%
idx=find(cgrid<0.1);
cgrid(idx)=min(c);
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
    
 save siliconFeb3balls.mat c cgrid def_xs force_xs gt normal_gt normal_xs xs
    

    