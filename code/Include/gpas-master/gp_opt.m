function f = gp_opt(fun, sample, N)

if f < gp.fmin
  gp.fmin = f;
  gp.xmin = x;
end

gp = gp_add(gp, x, f);


global S

% S.N - number of initial samples

% initial samples
S.xs = feval(sample, S.N0);

%S.xs = [-.2 0 .2 .21 .4];
S.fs = feval(fun, S.xs);

% test points
S.xss = feval(sample, S.Nmax);
S.fss = feval(fun, S.xss);

gp_train;


% optimize hyperparams
%[p,FVAL,EXITFLAG,OUTPUT] = fminsearch(@gp_minhp, [S.l, S.s])
%S.l = p(1);
%S.s = p(2);



if 0
xts = [-2:.01:2];

  [S.ms, S.ss] = gp_predict(xts);

vs = sqrt(diag(S.ss));

plot(xts, S.ms + vs, '--', xts, S.ms - vs, '--', xts, S.ms, '-');
hold on
plot(S.xs, S.fs, 'o')
pause(0)
end

fmin = inf;
xmin = [];


for i=1:N
%  [S.ms, S.ss] = gp_predict(S.xss(1));
%  return
  
  [S.ms, S.ss] = gp_predict(S.xss);
  
  J = S.ms - 1.96*sqrt(diag(S.ss));
  [y, i] = min(J);
  x = S.xss(:,i);
  f = feval(fun, x);
  
  if f < fmin
    fmin = f;
    xmin = x;
  end
  fmin
  xmin
  
  gp_add(x, f);
end


function f = gp_minhp(p)
global S

S.l = p(1);
S.s = p(2);
gp_train;
f = -S.lp;
