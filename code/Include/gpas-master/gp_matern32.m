function f = gp_matern32(gp, xa, xb)

f = zeros(size(xa,2), size(xb,2));
for i=1:size(xa,2),
  d = repmat(xa(:,i),1,size(xb,2)) - xb;    
  r = sqrt(sum(d.*d, 1));
  
  %  ss = sqrt(s);
  %  I = find(ss < .1);  
  %  f(i,I) = (gp.s.^2).*exp(-s(I)./(2*(gp.l.^2))).*(1 - ss(I)/.1);
  c = sqrt(3)/gp.l;
  f(i,:) = (gp.s.^2)*(1 + c*r).*exp(-c*r);
%  I = find(s < eps);
%  f(i,I) = f(i,I) + gp.sigma^2;
end
