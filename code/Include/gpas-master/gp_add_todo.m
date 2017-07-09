function gp = gp_add_todo(gp, x, f)
% Add a data point (x,f) to gp and return updated gp

ks = gp_sqexp(gp, gp.xs, x);

k = gp_sqexp(gp, x, x);

gp.fs = [gp.fs, f];


if 0,

opts.LT = true;

L = full(gp.L);

ls = linsolve(L, ks, opts); 

l = sqrt(k - ls'*ls);

gp.L = [L, zeros(size(gp.L, 1), 1);
       ls', l];


a1 = linsolve(gp.L, gp.fs', opts); 

opts.TRANSA = true;

gp.a = linsolve(gp.L, a1, opts); 

else 

  ls = gp.L\ks;
  
  l = sqrt(k - ls'*ls);
  
  gp.L = [gp.L, zeros(size(gp.L, 1), 1);
          ls', l];
  
  gp.a = gp.L'\(gp.L\gp.fs');
  
end


%gp = gp_train(gp)

gp.xs = [gp.xs, x];

gp = gp_train(gp);

gp.n = gp.n + 1;

gp.lp = -gp.fs*gp.a/2 - sum(log(diag(gp.L))) - gp.n/2*log(2*pi);
