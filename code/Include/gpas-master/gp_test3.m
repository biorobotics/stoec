function f = gp_test3

clear

N0 = 500;
Ns = 5000;

opt.Ns = Ns;

opt.xi = [-2.5; -2.5];
opt.xf = [2.5; 2.5];

opt.xr = -2.5:.1:2.5;
opt.yr = -2.5:.1:2.5;

%%%%%%%%%       
%    5 g% 
%       %
%  12 4 %
%   3   %
%s      %
%%%%%%%%%
opt.os = [-1.2, -.8, -.25,   1.3, 0.1; 
          0,    -.3,  -1, -.6, 1.8];
opt.r = [.4, .5, .4,  .9, .9];

opt.xmin = [];
opt.fmin = [];
opt.J = [];
opt.cP = [];
opt.fP = [];
opt.sel = 'pi';

opt.sn = 2;
opt.snf = 10;

xs0 = stline(opt.xi, opt.xf, opt.sn); 

opt.mu = reshape(xs0(:,2:end-1), 2*opt.sn, 1);
opt.Sigma = 2*eye(2*opt.sn);


[X,Y] = meshgrid(opt.xr, opt.yr);
xss = [reshape(X, 1, size(X,1)*size(X,2));
       reshape(Y, 1, size(Y,1)*size(Y,2))];

xss = sample(opt, Ns);

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

opt.N0f = size(xs,2);


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
ofig = figure
gp_plot3(opt)

xfig = figure
plot_env(opt)

%return

for i=1:10,
  opt = gp_fit(opt);
  
  tic
  [opt, x, y] = gp_select(opt);
  disp('select')
  toc
  
%  if (abs(x-cgp.xs)<.005)
%    continue;
%  end
    
  if ~isempty(cgp)
    c = con(x, opt);        
    tic
    opt.cgp = gp_add(opt.cgp, x, c);
    disp('cgp add')
    toc

    if (c < 0)
      disp('added con')
      continue
    end
  end    
  
  disp('added fun')
  f = fun(x,opt);
  
  if f < opt.fmin
    opt.fmin = f;
    opt.xmin = x;
  end
  
  tic
  opt.fgp = gp_add(opt.fgp, x, f);
  disp('fgp add')
  toc  
  
  %  cgp = cgp_add(cgp, x, c);
  
  display('opt:') 
  opt.xmin

 % figure(ofig)
 % gp_plot3(opt)
  
  figure(xfig)
  plot_env(opt);
%  plot(x,f,'ob')

%  S.xmin
 % S.fmin
end

%plot(S.xss, S.ms, '.r')


function f = fun(xs, opt)
xs = reshape(xs, 2,length(xs)/2);
xsa = [opt.xi, xs];
xsb = [xs, opt.xf];
dxs = xsb - xsa;
f = sum(sqrt(sum(dxs.*dxs, 1)));


function xs = sample(opt,N)
xs = mvnrnd(opt.mu, opt.Sigma, N)';


function cmin = con(xs, opt)

cmin = inf;
xs = reshape(xs, 2,length(xs)/2);

xsa = [opt.xi, xs];
xsb = [xs, opt.xf];

dxs = xsb - xsa;

for i=1:size(opt.os,2)
  o = opt.os(:,i);
  r = opt.r(i);
    
  for i=1:size(dxs,2)
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
end


function f = plot_env(opt)

a = 0:2*pi/20:2*pi;
hold off

for i=1:size(opt.os,2)
  o = opt.os(:,i);
  r = opt.r(i);
  
  plot(o(1) + cos(a)*r, o(2) + sin(a)*r, '-k', 'LineWidth',3);

  hold on
  if ~isempty(opt.xmin)
    xs = [opt.xi, reshape(opt.xmin, 2,length(opt.xmin)/2), opt.xf];
    plot(xs(1,:), xs(2,:), '-*g', 'LineWidth',4);
  end
end

for i=1:size(opt.fgp.xs,2)
  ps = opt.fgp.xs(:,i);
  xs = [opt.xi, reshape(ps, 2,length(ps)/2), opt.xf];
  if (i<=opt.N0f)
    plot(xs(1,:), xs(2,:), '--o', 'LineWidth',1);  
  else
    plot(xs(1,:), xs(2,:), '-xb', 'LineWidth',2);  
  end    
end
axis equal
%axis([-2.5 2.5 -2.5 2.5])
drawnow

function xs = stline(xi, xf, sn)
n = length(xi);
xs = zeros(n, sn + 2);
for i=1:n
  xs(i,:) = linspace(xi(i), xf(i), sn+2);
end


function xs = si_traj(ps, S)
xs = [S.xi, reshape(ps, 2, S.sn), S.xf];
xs = interp1(linspace(0, 1, size(xs,2)), xs', ...
             linspace(0, 1, S.snf), 'cubic')';
