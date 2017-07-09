function f = gp_plot2(opt)


set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 18)

set(0,'DefaultTextFontname', 'Times New Roman')
set(0,'DefaultTextFontSize', 18)

if (~isempty(opt.figs))
  figure(opt.figs(1));
  set(opt.figs(1), 'Position', [100, 100, 800, 600]);
else
  sp = subplot(2,2,1);
end

hold off
[ms, ss] = gp_predict(opt.fgp, opt.xss);
vs = sqrt(diag(ss));

h = surf(opt.xr, opt.yr, reshape(ms, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','FaceLighting','phong');
hold on
h1 = surf(opt.xr, opt.yr, reshape(ms + 1.96*vs, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','FaceLighting','phong','EdgeAlpha',.3);
%set(h,'FaceAlpha',0);
h2 = surf(opt.xr, opt.yr, reshape(ms - 1.96*vs, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','FaceLighting','phong','EdgeAlpha',.3);
%set(h,'FaceAlpha',0);


alpha(h,.8)
alpha(h1,.3)
alpha(h2,.3)

%xlabel('$x_1$','Interpreter','latex')
%%ylabel('$x_2$','Interpreter','latex')
%zlabel('$\mathbb{E}[J(x)] \pm \beta Var[J(x)]$', 'Interpreter', 'latex')

view(-20,48)
axis tight

print(gcf,'-dpng', 'figures/Jx.png')
%savesp(sp, 'figures/Jx');

%plot(xts, ms + 1.96*vs, '--', xts, ms - 1.96*vs, '--', xts, ms, '-');
%hold on
%plot3(opt.fgp.xs(1,:), opt.fgp.xs(2,:), opt.fgp.fs, 'or');

%title('Probabilistic Model of Trajectory Cost $J(x)$')

if (~isempty(opt.figs))
  figure(opt.figs(2));
  set(opt.figs(2), 'Position', [100, 100, 800, 600]);

else
  sp = subplot(2,2,2);
end

hold off
[ms, ss] = gp_predict(opt.cgp, opt.xss);
vs = sqrt(diag(ss));

h = surf(opt.xr, opt.yr, reshape(ms, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','FaceLighting','phong');
%set(h,'FaceAlpha',0);
hold on
h1 = surf(opt.xr, opt.yr, reshape(ms + 1.96*vs, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','EdgeAlpha', .3,'FaceLighting','phong');
%set(h,'FaceAlpha',0);
h2 = surf(opt.xr, opt.yr, reshape(ms - 1.96*vs, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','EdgeAlpha',.3,'FaceLighting','phong');
%set(h,'FaceAlpha',0);

alpha(h,.8)
alpha(h1,.3)
alpha(h2,.3)

view(-20,48)
axis tight



%plot(xts, ms + 1.96*vs, '--', xts, ms - 1.96*vs, '--', xts, ms, '-');
hold on
plot3(opt.cgp.xs(1,:), opt.cgp.xs(2,:), opt.cgp.fs, 'or');
%title('Probablistic Model of Constraints $g(x)$')

%xlabel('$x_1$','Interpreter','latex')
%ylabel('$x_2$','Interpreter','latex')
%zlabel('$\mathbb{E}[g(x)] \pm \beta Var[g(x)]$', 'Interpreter', 'latex')

print(gcf,'-dpng', 'figures/Fx.png')
%savesp(sp, 'figures/Fx');


if (~isempty(opt.figs))
  figure(opt.figs(3));
  set(opt.figs(3), 'Position', [100, 100, 800, 600]);
else
  sp = subplot(2,2,3);
end

hold off
if ~isempty(opt.J)
  h = surfc(opt.xr, opt.yr, reshape((opt.J)/sum(opt.J)/(opt.dr^2), length(opt.xr), length(opt.yr)),...
            'FaceColor','interp','FaceLighting','phong');
  
  %  set(h,'FaceAlpha',0);
  view(-20,48)
  axis tight


  %  plot(gp.xss, gp.J, 'b')
  hold on
end
%title('States Sampling Distribution')

%xlabel('$x_1$','Interpreter','latex')
%ylabel('$x_2$','Interpreter','latex')
%zlabel('$P(x)$', 'Interpreter', 'latex')


print(gcf,'-dpng', 'figures/Px.png')
%savesp(sp, 'figures/Px');

if (~isempty(opt.figs))
  figure(opt.figs(4));
  set(opt.figs(4), 'Position', [100, 100, 800, 600]);
else
  sp = subplot(2,2,4);
end

hold off
if ~isempty(opt.cP)
  
  h = surfc(opt.xr, opt.yr, reshape(opt.cP, length(opt.xr), length(opt.yr)),...
  'FaceColor','interp','FaceLighting','phong');
  
  %  set(h,'FaceAlpha',0);
  view(-20,48)
axis tight

  
  hold on
end
%title('Constraint Satisfaction Probability')

%zlabel('$P(g(x)\geq 0)$', 'Interpreter', 'latex')
%xlabel('$x_1$', 'Interpreter', 'latex')
%ylabel('$x_2$', 'Interpreter', 'latex')

print(gcf,'-dpng', 'figures/Pg.png')
%savesp(sp, 'figures/Pf');