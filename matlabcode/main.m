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

ik = inverseKinematics( 'RigidBodyTree' ,robot);
ikWeights = [1 1 1 1 1 1];
ikInitGuess =  robot.homeConfiguration; % 随机设置一个初始状态

joint_position=zeros(5,length(q));

for idx = 1:length(q)
    % 解逆运动学方程
    Q=q(:,idx);
    Q=Q';
    tgtPose = trvec2tform(Q);
    [config,info] = ik( 'endeffector', tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config; % 以上一时刻的状态作为下一时刻的初始值

    for m=1:5
        joint_position(m,idx)=config(m).JointPosition;
    end

end

q2=zeros(3,length(q));
n=5;
joint_position_2=[0;0;0;0;0];
for i=1:n
    q2(:,i)=q(:,i);
    joint_position_2(:,i)= joint_position(:,i);
end

for idx =n+1:length(q)
    %Moving Average Filter
    for m=1:5
        joint_position_2(m,idx)= joint_position_2(m,idx-1)+(joint_position(m,idx)-joint_position(m,idx-n))/n;
        config(m).JointPosition=joint_position_2(m,idx);
    end
    transform = getTransform(robot,config,"endeffector","base");
    q2(1,idx)=transform(1,4);
    q2(2,idx)=transform(2,4);
    q2(3,idx)=transform(3,4);
    % 画出机器人的动态
    show(robot,config , "PreservePlot" ,false);
    view(135,45);
    drawnow
    hTraj2 = plot3(q2(1,idx),q2(2,idx),q2(3,idx), "r.-" );
end
hold off