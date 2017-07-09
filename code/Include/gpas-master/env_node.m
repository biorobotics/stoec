function S = env_node(S)
% Simulate environmental data ROS node
% Will send back data after receiving odom
% or could just broadcast when new data is available
%
% @param S.envFile environment image file
%          scale
%          xlb, xub bounds
%          sigma meas noise
%

rosshutdown

if isfield(S, 'ROS_MASTER_URI')
  setenv('ROS_MASTER_URI', S.ROS_MASTER_URI)
end

if isfield(S, 'ROS_IP')
  setenv('ROS_IP', S.ROS_IP)
end

if ~isfield(S, 'envFile')
  S.envFile = 'data/do1.ppm';
end

if ~isfield(S, 'xlb')
  S.xlb = [-50;-50];
end

if ~isfield(S, 'xub')
  S.xub = [50;50];
end

if ~isfield(S, 'scale')
  S.scale = 1.5;
end

if ~isfield(S, 'sigma')
  S.sigma = 0;
end

rosinit


% scalar field over 2d domain loaded from a file
S.I = S.scale*double(imread(S.envFile))/255;


% environmental sensor data (use Temperature for now)
S.envPub = rospublisher('/env', rostype.sensor_msgs_Temperature)
S.envMsg = rosmessage(S.envPub)


% subscriber 
odomSub = rossubscriber('/insekf/pose', rostype.nav_msgs_Odometry, {@odomCallback, S}, 'BufferSize', 100)

disp('waiting for odoms...')
while(1)
  pause(1)
end

function f = odomCallback(src, msg, S)
x = [msg.Pose.Pose.Position.X;
     msg.Pose.Pose.Position.Y];

% get reading and publish it
S.envMsg.Temperature_ = env_scalar2d(x, S);

send(S.envPub, S.envMsg);

disp(['odomCallback: sent data=' num2str(S.envMsg.Temperature_)...
      ' at p=(' num2str(x(1)) ',' num2str(x(2)) ')']);


