clear;close all;clc;
picture=imread("bear2.jpg");
pic_size=64;%64x64pixel

waypoints=pic_2_point(picture,pic_size);

% setting robot arm
dhparams = [   
            0   	pi/2	        0.025         0;
            0.2      0           0         0;
            0.1      0           0         0; 
            0.05   	-pi/2	     0    0
            0           0          -0.15   0];

robot = rigidBodyTree; % 初始化机械臂
body1 = rigidBody('body1');% 定义第一个连杆
jnt1 = rigidBodyJoint('jnt1','revolute');% 定义第一个关节
setFixedTransform(jnt1,dhparams(1,:),'dh');% 给底座加上关节
body1.Joint = jnt1;% 给底座加上关节
addBody(robot,body1,'base')% 定义机械臂为机器人的底座

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.PositionLimits=[0, pi/2];
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.PositionLimits=[-pi, 0];
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
jnt4.PositionLimits=[-pi/2, pi/2];
jnt5 = rigidBodyJoint('jnt5','revolute');

bodyEndEffector = rigidBody('endeffector');
tform5 = trvec2tform([0, 0, -0.15]);

setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
bodyEndEffector.Joint=jnt5;


addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,bodyEndEffector,'body4');

config = homeConfiguration(robot);

time=1;
waypointTimes=1;
for i=2:length(waypoints)
    if waypoints(i,3)==0.02
        time=time+3;
        waypointTimes=[waypointTimes time];
    elseif waypoints(i-1,3)==0.02 && waypoints(i,3)==0
        time=time+3;
        waypointTimes=[waypointTimes time];
    elseif waypoints(i-1,3)==0.02 && waypoints(i,3)==0.02
        time=time+5;
        waypointTimes=[waypointTimes time];
    else
        time=time+1;
        waypointTimes=[waypointTimes time];
    end
end

ts = 0.5;
trajTimes = 0:ts:waypointTimes(end);
waypointAccelTimes = diff(waypointTimes)/4;
[q,qd,qdd,tvec] = trapveltraj(waypoints',numel(trajTimes), ...
 "AccelTime" ,repmat(waypointAccelTimes,[3 1]), ... 
 "EndTime" ,repmat(diff(waypointTimes),[3 1]));

show(robot,config);
hold on

% set(hTraj,  "xdata" , q(1,:),  "ydata" , q(2,:),  "zdata" , q(3,:));

ik = inverseKinematics( 'RigidBodyTree' ,robot);
ikWeights = [1 1 1 1 1 1];
ikInitGuess =  robot.homeConfiguration; % 随机设置一个初始状态

for idx = 1:length(q)
    % 解逆运动学方程

    Q=q(:,idx);
    Q=Q';
    tgtPose = trvec2tform(Q);
    [config,info] = ik( 'endeffector', tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config; % 以上一时刻的状态作为下一时刻的初始值

    % 画出机器人的动态
    show(robot,config , "PreservePlot" ,false);
    ylabel([ 'Trajectory at t = ' num2str(trajTimes(idx))]);
    hTraj = plot3(q(1,idx),q(2,idx),q(3,idx), "b.-" );
    view(135,45);
    % xlim([-0.25 0.25])
    % ylim([-0.25 0.25])
    % zlim([-0.25 0.25])
    drawnow    
end
hold off