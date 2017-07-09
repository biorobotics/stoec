function [Xicp ] = vanilla_icp( data,cad_points,Xreg,n)
Treg=regParamsToTransformationMatrix(Xreg);
[TR,TT]=icp(cad_points',computeTransformedPoints( data,Xreg),n);

Tt=eye(4);
Tt(1:3,1:3)=TR;
Tt(1:3,4)=TT;

Xicp=transformationMatrixToRegParams(Tt*Treg);


end

