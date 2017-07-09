function gp = gp_train(gp)
% update GP internal parameters L and a
% that are used for prediction

gp.Ks = sparse(gp_sqexp(gp, gp.xs, gp.xs));

%gp.Ks = gp_sqexp(gp, gp.xs, gp.xs);
% disp(['removed' num2str(100*(1 - nnz(Ks)/prod(size(Ks)))) '% of nnz']) 

gp.n = size(gp.xs,2);

gp.L = chol(gp.Ks + eye(size(gp.Ks))*(gp.sigma^2))';

gp.a = gp.L'\(gp.L\gp.fs');

% a different implementation using conjugate gradient
%gp.a = inv(gp.Ks)*gp.fs';
%tol = 1e-3;
%maxit = 100;
%L = ichol(gpKs);
%[a,flag,err,iter,res] = pcg(Ks, gpfs, tol,maxit,L,L')
%gpa = a;

gp.lp = -gp.fs*gp.a/2 - sum(log(diag(gp.L))) - gp.n/2*log(2*pi);
