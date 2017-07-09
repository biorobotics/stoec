function f = env_scalar2d(xs, S)

xr = S.xub(1)-S.xlb(1);
yr = S.xub(2)-S.xlb(2);
is = floor((S.xub(2) - xs(2,:))/yr*size(S.I,2)) + 1;
js = floor((xs(1,:)-S.xlb(1))/xr*size(S.I,1)) + 1;

is(find(is>size(S.I,1)))=size(S.I,2);
js(find(js>size(S.I,2)))=size(S.I,2);
is(find(is<1))=1;
js(find(js<1))=1;

for i=1:size(xs,2)
  f(i)= S.I(is(i),js(i),1);
end

%f = f + randn(size(f))*S.sigma;
