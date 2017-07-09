function [m, s] = gp_predict(gp, x)

k = sparse(gp_sqexp(gp, gp.xs, x));

m = k'*gp.a;

v = gp.L\k;

s = gp_sqexp(gp, x, x) - v'*v;

% s(find(s<1e-10))=0;