function opt = gp_fit(opt)

opt.xss = mvnrnd(opt.mu, opt.Sigma, opt.Ns)';

[ms, ss] = gp_predict(opt.fgp, opt.fgp.xs);
vs = sqrt(diag(ss));

ws = normcdf(opt.fmin, ms, vs);

wxs = repmat(ws'./sum(ws'), size(opt.fgp.xs,1), 1).*opt.fgp.xs;

%opt.mu = mean(wxs')';
%opt.Sigma = cov(wxs')';
opt.mu = mean(opt.fgp.xs')';
opt.Sigma = cov(opt.fgp.xs');

opt.mu
opt.Sigma