function S = odom_node(S)
% Simulate odometry data ROS node, by waiting for
% commanded path and taking the next pose along the path
%
%

rosshutdown

if isfield(S, 'ROS_MASTER_URI')
  setenv('ROS_MASTER_URI', S.ROS_MASTER_URI)
end

if isfield(S, 'ROS_IP')
  setenv('ROS_IP', S.ROS_IP)
end

rosinit

% published odometry every at every vehicle simulation step
S.odomPub = rospublisher('/insekf/pose', rostype.nav_msgs_Odometry)
S.odomMsg = rosmessage(S.odomPub)

pause(1)

% subscriber 
cmdSub = rossubscriber('/adp_path', rostype.nav_msgs_Path, {@cmdCallback, S})

% start 
startPub = rospublisher('/adp_start', rostype.std_msgs_Bool);
startMsg = rosmessage(startPub);
startMsg.Data = 1;
send(startPub, startMsg);


% simulate initial point
S.odomMsg.Pose.Pose.Position.X = 25;
S.odomMsg.Pose.Pose.Position.Y = 25;
S.odomMsg.Pose.Pose.Orientation.Z = 1.1*pi;
send(S.odomPub, S.odomMsg)

while(1)
  pause(1)
end

function f = cmdCallback(src, msg, S)

% take the next pose along path
S.odomMsg.Pose.Pose = msg.Poses(1).Pose;
send(S.odomPub, S.odomMsg)

x = [S.odomMsg.Pose.Pose.Position.X;
     S.odomMsg.Pose.Pose.Position.Y;
     S.odomMsg.Pose.Pose.Position.Z];

disp(['cmdCallback: sent odom=' num2str(x(1)) ',' num2str(x(2))]);