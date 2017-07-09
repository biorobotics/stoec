function gp = gp_optparams(gp);

p = [gp.l, gp.s];
[p,FVAL,EXITFLAG,OaUTPUT] = fminsearch(@(p) gp_minhp(p, gp), p);

gp.l = p(1);
gp.s = p(2);
gp = gp_train(gp);

function f = gp_minhp(p, gp)

gp.l = p(1);
gp.s = p(2);

gp = gp_train(gp);

f = -gp.lp;