function gp = gp_init(xs, fs, opts)
% Initialize a GP over f(x) using an initial dataset (xs, fs)
% 
% Required options
% opts.l
% opts.s

gp = [];

gp.l = opts.l;
gp.s = opts.s;
gp.sigma = opts.sigma;

gp.xs = xs;
gp.fs = fs;

gp = gp_train(gp);

% optimize hyperparams
%p = [gp.l, gp.s];
%
%[p,FVAL,EXITFLAG,OaUTPUT] = fminsearch(@(p) gp_minhp(p, gp), p);
%
%gp.l = p(1);
%gp.s = p(2);
%gp = gp_train(gp);


function f = gp_minhp(p, gp)

gp.l = p(1);
gp.s = p(2);

gp = gp_train(gp);

f = -gp.lp;