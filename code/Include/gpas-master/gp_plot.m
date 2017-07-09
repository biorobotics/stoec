function f = gp_plot(gp)

xts = [-3:.1:3];

[ms, ss] = gp_predict(gp, xts);

vs = sqrt(diag(ss));

subplot(2,1,1)
hold off
plot(xts, ms + 1.96*vs, '--', xts, ms - 1.96*vs, '--', xts, ms, '-');
hold on
plot(gp.xs, gp.fs, 'or');

subplot(2,1,2)
hold off
if isfield(gp, 'J')
  plot(gp.xss, gp.J, 'b')
  hold on
end

if isfield(gp, 'P')
  plot(gp.xss, gp.P, 'g')
  hold on
end