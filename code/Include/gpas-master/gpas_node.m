function f = gpas_node(opt)
% Adaptive Sampling for discovering peak concentration in a 2d scalar field
%
% Author: Marin Kobilarov, marin(at)jhu.edu 

% Options:

% workspace lower bound
if ~isfield(opt, 'xlb')
  opt.xlb = [-50;-50];
end

% workspace upper bound
if ~isfield(opt, 'xub')
  opt.xub = [50;50];
end

% grid cells along each dimension
if ~isfield(opt, 'ng')
  opt.ng = [30;30];
end

% trajectory parameters
if ~isfield(opt, 'sn')
  opt.sn = 5;
end

% time-step
if ~isfield(opt, 'dt')
  opt.dt = 3;
end

% time horizon (seconds)
if ~isfield(opt, 'tf')
  opt.tf = 30;
end

% forward velocity lower bound
if ~isfield(opt, 'vlb')
  opt.vlb = .1;
end

% forward velocity upper bound
if ~isfield(opt, 'vub')
  opt.vub = 5;
end


% angular velocity lower bound
if ~isfield(opt, 'wlb')
  opt.wlb = -.5;
end


% angular velocity upper bound
if ~isfield(opt, 'wub')
  opt.wub = .5;
end

% Cross-entropy parameters

% outer CE iterations (only useful if we want to display/save each CE iteration)
if ~isfield(opt, 'iters')
  opt.iters = 2;
end

if ~isfield(opt, 'ce')
  opt.ce = [];
end

% #of samples
if ~isfield(opt.ce, 'N')
  opt.ce.N = 100;
end

% smoothing parameter
if ~isfield(opt.ce, 'v')
  opt.ce.v = .8;
end

% CE iterations per optimization
if ~isfield(opt.ce, 'iter')
  opt.ce.iter = 10;
end

% use the sigma-point CE 
if ~isfield(opt.ce, 'sigma')
  opt.ce.sigma = 0;
end

% initial covariance
if ~isfield(opt.ce, 'C0')
  opt.ce.C0 = diag(repmat([1;1],opt.sn,1));
end

% initial mean (straight line)
if ~isfield(opt.ce, 'z0')
  opt.ce.z0 = repmat([1; 0], opt.sn,1);
end


% GP parameters
if ~isfield(opt, 'gp')
  opt.gp = [];
end

% length-scale
if ~isfield(opt.gp, 'l')
  opt.gp.l = 5;
end

% maximum variance
if ~isfield(opt.gp, 's')
  opt.gp.s = 0.4;
end

% output noise
if ~isfield(opt.gp, 'sigma')
  opt.gp.sigma = 0.001;
end


% environment parameters
if ~isfield(opt, 'mapFile')
  opt.map = [];
end

if isfield(opt, 'envFile')
  
  if ~isfield(opt, 'scale')
    opt.scale = 1.5;
  end

  opt.I = opt.scale*double(imread(opt.envFile))/255; 
else
  opt.I = [];
end


% how many time stages to run
if ~isfield(opt, 'stages')
  opt.stages = 500;
end

% obstacle map
if isfield(opt, 'mapFile')
  opt.map = double(imread(opt.mapFile))/255; 
else
  opt.map = [];
end



% obstacle map
if ~isfield(opt, 'devBias')
  opt.devBias = .001;
end

% use one or separate figures?
if 0
opt.figs(1) = figure;
opt.figs(2) = figure;
opt.figs(3) = figure; 
opt.figs(4) = figure;
else
  opt.figs = [];  
end



% meshgrid for display
opt.xr = linspace(opt.xlb(1), opt.xub(1), opt.ng(1))';
opt.yr = linspace(opt.xlb(2), opt.xub(2), opt.ng(2))';

opt.xopt = [];
opt.fopt = [];
opt.J = [];
opt.sel = 'pi';

[X,Y] = meshgrid(opt.xr, opt.yr);
% list of grid points
xss = [reshape(X, 1, size(X,1)*size(X,2));
       reshape(Y, 1, size(Y,1)*size(Y,2))];


%opt.xi = [-45; -45; pi/4];  
%opt.xi = [25; 25; 1.1*pi];  % for DO


% sequence of measured (unprocessed) states and current state
global odomData envData startCmd
startCmd = [];
odomData = [];
envData.xs = [];
envData.fs = [];

% setup ROS node
rosshutdown

if isfield(opt, 'ROS_MASTER_URI')
  setenv('ROS_MASTER_URI', opt.ROS_MASTER_URI)
end

if isfield(opt, 'ROS_IP')
  setenv('ROS_IP', opt.ROS_IP)
end

rosinit

odomSub = rossubscriber('/insekf/pose', rostype.nav_msgs_Odometry, ...
                        @odomCallback)

% wait for valid odom data
while isempty(odomData)
  pause(.01);
end

envSub = rossubscriber('/env', rostype.sensor_msgs_Temperature, ...
                        @envCallback)

% wait for valid env data
while isempty(envData.xs)
  pause(.01);
end

startSub = rossubscriber('/adp_start', rostype.std_msgs_Bool, ...
                        @startCallback)
                    
% wait for start command
while isempty(startCmd)
  pause(1);
end               

cmdPub = rospublisher('/adp_path', rostype.nav_msgs_Path)
cmdMsg = rosmessage(cmdPub)

cmdPub_rviz = rospublisher('/adp_path_rviz', rostype.geometry_msgs_PoseStamped)
cmdMsg_rviz = rosmessage(cmdPub_rviz)



% if this is simulated (i.e. from a file) then display the true
if ~isempty(opt.I)
  opt.fs = env_scalar2d(xss,opt);
  p = randperm(size(xss,2));
  opt.pss = prior(xss, opt);
end

opt.xi = odomData(:,end);
xps = opt.xi;

% init current trajectory and measurements to start data

% init GP
fgp = gp_init(envData.xs(1:2,:), envData.fs, opt.gp);
%fgp.fun = @fun;

% init opt
[y,ind] = min(envData.fs);

% empty env data
envData.xs = [];
envData.fs = [];

opt.xopt = fgp.xs(:,ind);
opt.fopt = y;
opt.xss = xss;
opt.fgp = fgp;

%plot(opt.xss, opt.fss, '.b')
%ofig = figure

gp_plot(opt)

%xfig = figure
%plot_env(opt)

z=opt.ce.z0;
mu = z;

zlb = repmat([opt.vlb; opt.wlb], opt.sn,1);
zub = repmat([opt.vub; opt.wub], opt.sn,1);

xs = traj(z, opt)
opt.z = z;
c = traj_cost(z, opt)

%[z,fval,exitflag,output] = fmincon(@(z)traj_cost(z,opt), z, [], [],[],[],zlb,zub);

xs = traj(z, opt);

opt.ce.z = z;
opt.ce.C = opt.ce.C0;
opt.ce.lb = zlb;
opt.ce.ub = zub;

% traveled path

%xps = opt.xi;


video =0;
mov = 0;
if video,
  mov = VideoWriter('as.avi');
  open(mov);
  
%  mov = avifile('as.avi');
%  mov.Quality = 100;
%  mov.fps = 2*iters;
end
j=1; % image index

% time-series
ts = 0;
ys = y;
cs = c;



for k=1:opt.stages
  
    % wait for start command
    while isempty(startCmd)
        pause(1);
    end   
    
    
  %% PLANNING
  opt.ce.C = .5*opt.ce.C + .5*opt.ce.C0;
  subplot(2,3,1);
  Gp =  draw_path(traj(z, opt), 0, 'r', 2,5);
  z = opt.ce.z0;

  for i=1:opt.iters
    opt.ce.z = z; 
    [z, c, mu, C] = cem(@traj_cost, z, opt.ce, opt);
    opt.ce.C = C;
    xs = traj(z,opt);
    
    set(Gp,'XData', xs(1,:));
    set(Gp,'YData', xs(2,:));
    drawnow
    if video,
      saveas(gcf,['as/as' num2str(j,'%03d') '.jpg'])
      j=j+1;
      %    saveas(gca,['se2opt/v' num2str(c) '.eps'],'psc2');
      %    mov = addframe(mov,getframe(gca));
      %    writeVideo(mov,getframe);
      %    print(gcf,'-dpng', 'figures/env.png')
    end
  end
  
  
  % "EXECUTE" START OF PATH
  % broadcast all the points in the planned path
  for i_p=2:size(xs,2) %start from the second point, as first point is current location.
    xd = xs(:,i_p);
    m = rosmessage(rostype.geometry_msgs_PoseStamped);
    m.Pose.Position.X = xd(1);
    m.Pose.Position.Y = xd(2);
    m.Pose.Orientation.Z = xd(3);
    % send command 
    cmdMsg.Poses(i_p-1) = m;
  end

  send(cmdPub, cmdMsg);
  
  %publish another message for rviz visualization
  cmdMsg_rviz.Pose.Position.X = xd(1);
  cmdMsg_rviz.Pose.Position.Y = xd(2);
  cmdMsg_rviz.Pose.Orientation.Z = xd(3);
  cmdMsg_rviz.Header.FrameId = 'map';
  
  send(cmdPub_rviz,cmdMsg_rviz);
    
  while(1)
    % wait for env data
    while isempty(envData.xs)
       pause(.1)
    end
    if norm(opt.xi(1:2)-odomData(1:2)) > 1 
       break
    end
    pause(.1)
  end
  
  
  ts = [ts, ts(end) + opt.dt];
  ys = [ys, envData.fs(end)];
  cs = [cs, c];
  
  % the latest measurement
  opt.xi = odomData;

  % process accumulated measurements

  opt.fgp = gp_add(opt.fgp, envData.xs(1:2,:), envData.fs);
  envData.xs = [];
  envData.fs = [];
  
  xps = [xps, opt.xi];  
  
  hold off
  gp_plot(opt)
  hold on
  
  subplot(2,3,1)
  draw_path(xs(1:2,2:end), 0, 'b',3, 5);
  hold on
  draw_path(xps(1:2,:), 0, 'g',3, 5);
  xlabel('m')
  ylabel('m')
  drawnow
  
  subplot(2,3,4)
  hold off
  contour(opt.xr, opt.yr, reshape(opt.pss, length(opt.xr), length(opt.yr)));
  
  %h = contour(opt.xr, opt.yr, reshape(ms, length(opt.xr), ...
  % length(opt.yr)));
  
  hold on
  draw_path(xs(1:2,2:end), 0, 'b',2, 5);
  draw_path(xps(1:2,:), 0, 'g',3, 5);
  axis square
  axis equal
  axis([min(opt.xr),max(opt.xr),min(opt.yr),max(opt.yr)]  )
  title('Executed (g) and Planned (b) paths')
  xlabel('m')
  ylabel('m')

  subplot(2,3,3)  
  hold off
  plot(ts, ys)
  xlabel('s')
  title('Env Data')

  subplot(2,3,6)  
  hold off
  plot(ts, cs)
  xlabel('s')
  title('Trajectory Cost')

end

if video,
  close(mov);
end



%plot(opt.xss, opt.ms, '.r')

function f = odomCallback(src, msg)
global odomData

odomData = [msg.Pose.Pose.Position.X;
            msg.Pose.Pose.Position.Y;
            msg.Pose.Pose.Orientation.Z];
disp(['odomCallback: p=' num2str(odomData(1)) ',' num2str(odomData(2))])


function f = envCallback(src, msg)
global envData odomData

fm = msg.Temperature_;
disp(['envCallback: envData=' num2str(fm) ])

% use current odom
if isempty(odomData)
  disp('[W] envCallback: empty odomData!')
  return
end

envData.xs = [envData.xs, odomData];
envData.fs = [envData.fs, fm];


function f = startCallback(src, msg)
global startCmd

if msg.Data
    startCmd = 1;
    disp('Adaptive Sampling Enabled');
else
    startCmd = [];
    disp('Adaptive Sampling Disabled');
end

function G = draw_path(xs, z, c, lw, ms)

G=plot3(xs(1,:), xs(2,:), z*ones(size(xs,2),1), [c '-'], ...
        'LineWidth', lw, 'MarkerSize', ms);


function f = prior(xs, opt)

%mvnpdf([0, 0], [0, 0], diag([.05, .1])) 

if ~isempty(opt.I)
  f = mvnpdf(xs', [0, 0], diag([1000, 1000]));
else
  f = mvnpdf(xs', [0, 0], diag([1000, 1000]));
end

f = f/max(f);


function xs = traj(z, opt)

tl = opt.tf/opt.sn;

xs = zeros(3, 1);
xs = opt.xi;

for i=1:opt.sn,
  v = z(2*(i-1) + 1);
  w = z(2*(i-1) + 2);
  th = xs(3,end);

  t = opt.dt:opt.dt:tl;
  
  if (abs(w) < 1e-10)     
    xs = [xs(1,:), xs(1,end) + t*v*cos(th);
          xs(2,:), xs(2,end) + t*v*sin(th);
          xs(3,:), xs(3,end) + t*0];

  else
    xs = [xs(1,:), xs(1,end) + v/w*(sin(th + t*w) - sin(th));
          xs(2,:), xs(2,end) + v/w*(-cos(th + t*w) + cos(th));
          xs(3,:), xs(3,end) + t*w];
  end  
end


function f = traj_cost(z, opt)

xs = traj(z, opt);

% check for bounds
for i=1:2
  if sum(find(xs(i,:) < opt.xlb(i))) || sum(find(xs(i,:) > opt.xub(i)))
    f = 1000;
    return
  end
end

% check for obstacles
if ~isempty(opt.map)
  xr = opt.xub(1)-opt.xlb(1);
  yr = opt.xub(2)-opt.xlb(2);
  is = floor((opt.xub(2) - xs(2,:))/yr*size(opt.I,2)) + 1;
  js = floor((xs(1,:)-opt.xlb(1))/xr*size(opt.I,1)) + 1;
  
  is(find(is>size(opt.I,1)))=size(opt.I,2);
  js(find(js>size(opt.I,2)))=size(opt.I,2);
  is(find(is<1))=1;
  js(find(js<1))=1;
  
  for i=1:size(xs,2)
    if opt.map(is(i),js(i),1) > .5
      f = 1000;
      return
    end
  end
end

[ms, ss] = gp_predict(opt.fgp, xs(1:2,2:end));
vs = sqrt(diag(ss));

%ps = prior(xs(1:2,2:end), opt);
%ps = ones(size(ps));

f = -sum(ms + 1.96*vs);

f = f + opt.devBias*norm(opt.z-z);

%f = sum(ps.*(ms));
%f = -sum(ps.*vs);

vs = z(1:2:end-1);
ws = z(2:2:end);
dws = ws(2:end)-ws(1:end-1);

%f = f*(1 + .01*ws'*ws);% + .5*dws'*dws);

%f = f + .001*vs'*vs;

%f = mean(ms);

%fs = fun(xs, opt);

%for i=1:size(xs,2)%
%  gp = gp_add(gp, xs(:,i), 
%end


function f = gp_plot(opt)

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 15)

set(0,'DefaultTextFontname', 'Times New Roman')
set(0,'DefaultTextFontSize', 15)

if (~isempty(opt.figs))
  figure(opt.figs(1));
  set(opt.figs(1), 'Position', [100, 100, 800, 600]);
else
  sp = subplot(2,3,1);
  set(gcf, 'Position', [100, 100, 1400, 800]);
end

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

xlabel('m')
ylabel('m')

alpha(h,.8)
alpha(h1,.3)
alpha(h2,.3)

%xlabel('$x_1$','Interpreter','latex')
%%ylabel('$x_2$','Interpreter','latex')
%zlabel('$\mathbb{E}[J(x)] \pm \beta Var[J(x)]$', 'Interpreter', 'latex')

view(-20,48)
axis tight
%axis equal
title('Guassin Process Model (+-95% conf.)')
drawnow

%print(gcf,'-dpng', 'figures/Jx.png')

%savesp(sp, 'figures/Jx');

%plot(xts, ms + 1.96*vs, '--', xts, ms - 1.96*vs, '--', xts, ms, '-');
%hold on
%plot3(opt.fgp.xs(1,:), opt.fgp.xs(2,:), opt.fgp.fs, 'or');

%title('Probabilistic Model of Trajectory Cost $J(x)$')


if (~isempty(opt.figs))
  figure(opt.figs(2));
  set(opt.figs(2), 'Position', [100, 100, 800, 600]);
else
  sp = subplot(2,3,2);
end

cns = 1;

hold off
if ~isempty(opt.fs)
  if cns
    h = contour(opt.xr, opt.yr, reshape(opt.fs, length(opt.xr), ...
                                        length(opt.yr)));
    hold on
    colorbar;
    plot(opt.fgp.xs(1,:), opt.fgp.xs(2,:), '-g','LineWidth',2); 
  else
    h = surfc(opt.xr, opt.yr, reshape(opt.fs, length(opt.xr), length(opt.yr)),...
              'FaceColor','interp','FaceLighting','phong','EdgeAlpha',.3);
    
    set(h,'FaceAlpha',0.7);
    view(-20,48)
  end

  axis tight
  axis equal
  %  plot(gp.xss, gp.J, 'b')
  hold on
end
title('True Scalar Field')
xlabel('m')
ylabel('m')

if (~isempty(opt.figs))
  figure(opt.figs(3));
  set(opt.figs(3), 'Position', [100, 100, 800, 600]);
else
  sp = subplot(2,3,5);
end

hold off

if cns
  h = contour(opt.xr, opt.yr, reshape(ms, length(opt.xr), ...
                                      length(opt.yr)));
  hold on
  colorbar;
  plot(opt.fgp.xs(1,:), opt.fgp.xs(2,:), 'ok-'); 
else
  h = surf(opt.xr, opt.yr, reshape(ms, length(opt.xr), length(opt.yr)), ...
           'FaceColor','interp','FaceLighting','phong');
  %alpha(h,.5)
  set(h,'FaceAlpha',0.5);
  %  plot(gp.xss, gp.J, 'b')
  hold on
  plot3(opt.fgp.xs(1,:), opt.fgp.xs(2,:), opt.fgp.fs(:),'*'); 
  view(-20,48)

end

axis tight
axis equal
xlabel('m')
ylabel('m')
title('Estimated Field')

%title('States Sampling Distribution')

%xlabel('$x_1$','Interpreter','latex')
%ylabel('$x_2$','Interpreter','latex')
%zlabel('$P(x)$', 'Interpreter', 'latex')


%print(gcf,'-dpng', 'figures/Px.png')
%savesp(sp, 'figures/Px');


if (~isempty(opt.figs))
  figure(opt.figs(4));
  set(opt.figs(4), 'Position', [100, 100, 800, 600]);
else
%  sp = subplot(2,2,3);
end
