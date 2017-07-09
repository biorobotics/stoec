function [R,T] = findT(q,p)

m = size(p,2);
n = size(q,2);

weights=1/m*ones(1,m);

% find data centroid and deviations from centroid
q_bar = q * transpose(weights);
q_mark = q - repmat(q_bar, 1, n);
% Apply weights
q_mark = q_mark .* repmat(weights, 3, 1);

% find data centroid and deviations from centroid
p_bar = p * transpose(weights);
p_mark = p - repmat(p_bar, 1, m);
% Apply weights
%p_mark = p_mark .* repmat(weights, 3, 1);

N = p_mark*transpose(q_mark); % taking points of q in matched order

[U,~,V] = svd(N); % singular value decomposition

eigN=eig(N);
eigN=abs(eigN);
indx=find(eigN<10^-4);
if(size(indx,1)==1)
    V(:,indx)=-V(:,indx);
R = V*transpose(U);

T = q_bar - R*p_bar;

elseif(size(indx,1)>1)
    R=eye(3);
    T=zeros(3,1);
   
else


R = V*transpose(U);

T = q_bar - R*p_bar;

end