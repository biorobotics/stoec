function f = gp_plot3(opt)

if (opt.sn>1)
  return
end

subplot(2,2,1)
hold off
[ms, ss] = gp_predict(opt.fgp, opt.xss);
vs = sqrt(diag(ss));

[X,Y] = meshgrid(linspace(min(opt.xss(1,:)),max(opt.xss(1,:)),20),...
                 linspace(min(opt.xss(2,:)),max(opt.xss(2,:)),20));

tsi = TriScatteredInterp(opt.xss(1,:)',opt.xss(2,:)',ms);

Z = tsi(X,Y);

h = surfc(X, Y, Z, ...
          'FaceColor','interp','FaceLighting','phong');
%set(h,'FaceAlpha',0);
return
hold on
h1 = surf(opt.xr, opt.yr, reshape(ms + 1.96*vs, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','EdgeColor','none','FaceLighting','phong');
%set(h,'FaceAlpha',0);
h2 = surf(opt.xr, opt.yr, reshape(ms - 1.96*vs, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','EdgeColor','none','FaceLighting','phong');
%set(h,'FaceAlpha',0);

alpha(h,.8)
alpha(h1,.2)
alpha(h2,.2)

view(-20,48)

%plot(xts, ms + 1.96*vs, '--', xts, ms - 1.96*vs, '--', xts, ms, '-');
hold on
plot3(opt.fgp.xs(1,:), opt.fgp.xs(2,:), opt.fgp.fs, 'or');
title('Cost GP')

subplot(2,2,2)
hold off
[ms, ss] = gp_predict(opt.cgp, opt.xss);
vs = sqrt(diag(ss));

h = surfc(opt.xr, opt.yr, reshape(ms, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','FaceLighting','phong');
%set(h,'FaceAlpha',0);
hold on
h1 = surf(opt.xr, opt.yr, reshape(ms + 1.96*vs, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','EdgeColor','none','FaceLighting','phong');
%set(h,'FaceAlpha',0);
h2 = surf(opt.xr, opt.yr, reshape(ms - 1.96*vs, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','EdgeColor','none','FaceLighting','phong');
%set(h,'FaceAlpha',0);

alpha(h,.8)
alpha(h1,.2)
alpha(h2,.2)

view(-20,48)

%plot(xts, ms + 1.96*vs, '--', xts, ms - 1.96*vs, '--', xts, ms, '-');
hold on
plot3(opt.cgp.xs(1,:), opt.cgp.xs(2,:), opt.cgp.fs, 'or');
title('Constraints GP')


subplot(2,2,3)
hold off
if ~isempty(opt.J)
  h = surfc(opt.xr, opt.yr, reshape(opt.J, length(opt.xr), length(opt.yr)),...
            'FaceColor','interp','FaceLighting','phong');
  
  %  set(h,'FaceAlpha',0);
  view(-20,48)
  
  %  plot(gp.xss, gp.J, 'b')
  hold on
end
title('Combined Cost J')

subplot(2,2,4)
hold off
if ~isempty(opt.cP)
  
  h = surfc(opt.xr, opt.yr, reshape(opt.cP, length(opt.xr), length(opt.yr)),...
  'FaceColor','interp','FaceLighting','phong');
  
  %  set(h,'FaceAlpha',0);
  view(-20,48)
  
  hold on
end
title('Constraint Satisfaction Probability')
