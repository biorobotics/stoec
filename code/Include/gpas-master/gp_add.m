function gp = gp_add(gp, x, f)
gp.fs = [gp.fs, f];
gp.xs = [gp.xs, x];
gp = gp_train(gp);
