function f = gp_test2
% An example of path planning b/n two given states around an
% obstacle and learning the optimal waypoint the system 
% should pass through

clear

N0 = 25;

opt.figs(1) = figure;
opt.figs(2) = figure;
opt.figs(3) = figure; 
opt.figs(4) = figure;

opt.dr = .2;

opt.xi = [-2.5; -2.5];
opt.xf = [2.5; 2.5];

opt.xr = -2.5:opt.dr:2.5;
opt.yr = -2.5:opt.dr:2.5;

opt.o = [0;0];
opt.r = .8;

opt.xmin = [];
opt.fmin = [];
opt.J = [];
opt.cP = [];
opt.fP = [];
opt.sel = 'pi';


opt.snf = 20;

[X,Y] = meshgrid(opt.xr, opt.yr);
xss = [reshape(X, 1, size(X,1)*size(X,2));
       reshape(Y, 1, size(Y,1)*size(Y,2))];

p = randperm(size(xss,2));

% initial data
cxs = xss(:,p(1:N0));

%xss = xss(sort(p(N0+1:end)));

cs = zeros(1,size(cxs,2));
for i=1:size(cxs,2),
  cs(i) = con(cxs(:,i),opt);
end
  
xs = cxs(:,find(cs > 0));
fs = zeros(1,size(xs,2));
for i=1:size(xs,2),
  fs(i) = fun(xs(:,i),opt);
end


% cost function gp
fopts.l = 1;
fopts.s = 1;
fopts.sigma = .1;
fgp = gp_init(xs, fs, fopts)
fgp.fun = @fun;


% constraints gp
copts.l = 1;
copts.s = 1;
copts.sigma = .1;
cgp = gp_init(cxs, cs, copts)
cgp.fun = @con;

% test points
%Nmax = 500;

% init opt
[y,ind] = min(fs);
opt.xmin = fgp.xs(:,ind);
opt.fmin = y;
opt.xss = xss;
opt.fgp = fgp;
opt.cgp = cgp;

N = 50;

%plot(S.xss, S.fss, '.b')
%ofig = figure
gp_plot2(opt)

xfig = figure
plot_env(opt)


for i=1:3,

  [opt, x, y] = gp_select(opt)
  
%  if (abs(x-cgp.xs)<.005)
%    continue;
%  end
    
  if ~isempty(cgp)
    c = con(x, opt);        
    opt.cgp = gp_add(opt.cgp, x, c);
    
    
    if (c < 0)     
%      figure(ofig)
      gp_plot2(opt)
%      plot(x,c,'xb')

      pause(3)
      continue
    end
  end    
  
  f = fun(x,opt);
  
  if f < opt.fmin
    opt.fmin = f;
    opt.xmin = x;
  end
  
  opt.fgp = gp_add(opt.fgp, x, f);
%  cgp = cgp_add(cgp, x, c);
  
  display('opt:') 
  opt.xmin

%  figure(ofig)
  gp_plot2(opt)
  
  figure(xfig)
  plot_env(opt)
  print(gcf,'-dpng', 'figures/env.png')
  
%  plot(x,f,'ob')

  pause(3)
%  S.xmin
 % S.fmin
end

%plot(S.xss, S.ms, '.r')


function f = fun(xs, opt)

xi = [-2.5; -2.5];
xf = [2.5; 2.5];

xsa = [xi, xs];
xsb = [xs, xf];
dxs = xsb - xsa;
f = sum(sqrt(sum(dxs.*dxs, 1)));


function xs = sample(N, opt)
xs = 2.5 - 5*rand([2, N]);


function cmin = con(xs, opt)

o = opt.o
r = opt.r;


kin = 0

if ~kin,
  
xs = [opt.xi, xs, opt.xf];
xs = interp1(linspace(0, 1, size(xs,2)), xs', ...
             linspace(0, 1, opt.snf), 'cubic')';

xsa = xs(:,1:end-1);
xsb = xs(:,2:end);


else

xsa = [opt.xi, xs];
xsb = [xs, opt.xf];
end


dxs = xsb - xsa;

cmin = inf;

for i=1:size(xsa,2)
  xa = xsa(:,i);
  dx = dxs(:,i);
  a = dx/norm(dx);
  b = o - xa;
  d = b'*a;
  if d > 0
    c = norm(d*a - b) - r;
  else
    c = norm(xa - o) - r;
  end
  if c < cmin
    cmin = c;
  end
end


function f = plot_env(opt)
o = opt.o;
r = opt.r;

a = 0:.1:2*pi;
hold off
%plot(o(1) + cos(a)*r, o(2) + sin(a)*r, '-k', 'LineWidth',3);

[Xs,Ys,Zs] = cylinder2P([r, r], 50, [o;0]', [o;.4]');

%[Xs,Ys,Zs] = sphere;
%Zs = .1*Zs;
%Xs = r*Xs + o(1)*ones(size(Xs));
%Ys = r*Ys + o(2)*ones(size(Ys));

hs = surf(Xs, Ys, Zs);
set(hs,'FaceAlpha',1);

%set(hs,'EdgeColor','none', ...
%       'FaceColor','r', ...
%       'FaceLighting','phong', ...
%       'AmbientStrength',0.3, ...
%       'DiffuseStrength',0.8, ...
%       'SpecularStrength',0.9, ...
%       'SpecularExponent',25, ...
%       'BackFaceLighting','lit');

hold on

if ~isempty(opt.xmin)
  xs = [opt.xi, opt.xmin, opt.xf];
  xs = interp1(linspace(0, 1, size(xs,2)), xs', ...
               linspace(0, 1, opt.snf), 'cubic')';
  
  plot3(opt.xmin(1), opt.xmin(2), 0, '*','MarkerSize', 6, 'LineWidth',5)
  plot3(xs(1,:), xs(2,:), zeros(size(xs,2),1), '-g', 'LineWidth',5);
end

for i=1:size(opt.fgp.xs,2)
  xs = [opt.xi, opt.fgp.xs(:,i), opt.xf];
  xs = interp1(linspace(0, 1, size(xs,2)), xs', ...
               linspace(0, 1, opt.snf), 'cubic')';
  
  plot3(opt.fgp.xs(1,i), opt.fgp.xs(2,i), zeros(size(xs,2),1), ...
        'o','MarkerSize',5,  'LineWidth',2);
  plot3(xs(1,:), xs(2,:), zeros(size(xs,2),1), '--', 'LineWidth',2); 
end

axis square
axis equal
view(-20,48)
%view(3)

