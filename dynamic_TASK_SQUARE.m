%% GET ROBOT
robot = importrobot('robot.urdf')
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];
endEffector = 'link7'
numJoints = numel(homeConfiguration(robot));
Init = homeConfiguration(robot)
initTime = 0.0;
tStep = 0.1;
time1 = 2;
time2 = 3;
time3 = 4;
time4 = 5;

%% TASK SPACE 2_SQUARE_CUBIC

task0_sq = getTransform(robot, Init, endEffector);
task1_sq = trvec2tform([0.374, 0.2, 0.43])*axang2tform([0 1 0 pi]);
task2_sq = trvec2tform([0.374, 0.0, 0.23])*axang2tform([0 1 0 pi]);
task3_sq = trvec2tform([0.374, -0.2, 0.43])*axang2tform([0 1 0 pi]);

[s1,sd1,sdd1] = cubicpolytraj([0 1], [tStep;time1],tStep:tStep:time1);
[taskWaypoints1_sq, taskVelocities1_sq, taskAccel1_sq] = transformtraj(task0_sq, task1_sq, [tStep;time1], tStep:tStep:time1,'TimeScaling',[s1; sd1; sdd1]);
[s2,sd2,sdd2] = cubicpolytraj([0 1], [time1+tStep;time1+time2],time1+tStep:tStep:time1+time2);
[taskWaypoints2_sq, taskVelocities2_sq, taskAccel2_sq] = transformtraj(task1_sq, task2_sq, [time1+tStep;time1+time2], time1+tStep:tStep:time1+time2,'TimeScaling',[s2; sd2; sdd2]);
[s3,sd3,sdd3] = cubicpolytraj([0 1], [time1+time2+tStep;time1+time2+time3], time1+time2+tStep:tStep:time1+time2+time3);
[taskWaypoints3_sq, taskVelocities3_sq, taskAccel3_sq] = transformtraj(task2_sq, task3_sq, [time1+time2+tStep;time1+time2+time3], time1+time2+tStep:tStep:time1+time2+time3,'TimeScaling',[s3; sd3; sdd3]);
[s4,sd4,sdd4] = cubicpolytraj([0 1], [time1+time2+time3+tStep;time1+time2+time3+time4], time1+time2+time3+tStep:tStep:time1+time2+time3+time4);
[taskWaypoints4_sq, taskVelocities4_sq, taskAccel4_sq] = transformtraj(task3_sq, task0_sq, [time1+time2+time3+tStep;time1+time2+time3+time4], time1+time2+time3+tStep:tStep:time1+time2+time3+time4,'TimeScaling',[s4; sd4; sdd4]);

taskWaypoints_sq = zeros(4,(time1+time2+time3+time4)/tStep);
for i=1:time1/tStep
    taskWaypoints_sq(:,i)= taskWaypoints1_sq(:,4,i);
end    
for i=1:time2/tStep
    taskWaypoints_sq(:,i+time1/tStep)= taskWaypoints2_sq(:,4,i);
end    
for i=1:time3/tStep
    taskWaypoints_sq(:,i+(time1+time2)/tStep)= taskWaypoints3_sq(:,4,i);
end    
for i=1:time4/tStep
    taskWaypoints_sq(:,i+(time1+time2+time3)/tStep)= taskWaypoints4_sq(:,4,i);
end  
taskVelocities_sq = [taskVelocities1_sq,taskVelocities2_sq,taskVelocities3_sq,taskVelocities4_sq]
taskAccel_sq = [taskAccel1_sq,taskAccel2_sq,taskAccel3_sq,taskAccel4_sq];
t_sq = [taskWaypoints1_sq(:,:),taskWaypoints2_sq(:,:),taskWaypoints3_sq(:,:),taskWaypoints4_sq(:,:)];

ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

%% TASK_TRAJ_SQUARE_CUBIC
subplot(2,4,1);
show(robot, Init, 'PreservePlot', false, 'Frames', 'off');
title('Square trajectory _ CUBIC');
hold on
axis([-1 1 -1 1 -0.1 1.5]);
prev = Init

for i=1:(time1+time2+time3+time4)/tStep
    T_joint_sq = ik(endEffector, t_sq(1:4,4*(i-1)+1:4*i), weights, prev);
    show(robot, T_joint_sq, 'PreservePlot', false, 'Frames', 'off');
    prev = T_joint_sq;
    jointSpaceMarker = plot3(taskWaypoints_sq(1,i), taskWaypoints_sq(2,i), taskWaypoints_sq(3,i), 'b.', 'MarkerSize', 5);
    drawnow;
end
subplot(2,4,2);
plot(taskWaypoints_sq');
legend('x','y','z')
xlabel('t')
ylabel('Points')

subplot(2,4,3);
plot(taskVelocities_sq');
legend('th1','th2','th3','th4','th5','th6')
xlabel('t')
ylabel('Velocity')

subplot(2,4,4);
plot(taskAccel_sq');
legend('th1','th2','th3','th4','th5','th6')
xlabel('t')
ylabel('Acceleration')
%% TASK SPACE 2_SQUARE_LSPB

task0_sq = getTransform(robot, Init, endEffector);
task1_sq = trvec2tform([0.374, 0.2, 0.43])*axang2tform([0 1 0 pi]);
task2_sq = trvec2tform([0.374, 0.0, 0.23])*axang2tform([0 1 0 pi]);
task3_sq = trvec2tform([0.374, -0.2, 0.43])*axang2tform([0 1 0 pi]);

[s1,sd1,sdd1] = trapveltraj([0 1], numel(tStep:tStep:time1));
[taskWaypoints1_sq, taskVelocities1_sq, taskAccel1_sq] = transformtraj(task0_sq, task1_sq, [tStep;time1], tStep:tStep:time1,'TimeScaling',[s1; sd1; sdd1]);
[s2,sd2,sdd2] = trapveltraj([0 1], numel(time1+tStep:tStep:time1+time2));
[taskWaypoints2_sq, taskVelocities2_sq, taskAccel2_sq] = transformtraj(task1_sq, task2_sq, [time1+tStep;time1+time2], time1+tStep:tStep:time1+time2,'TimeScaling',[s2; sd2; sdd2]);
[s3,sd3,sdd3] = trapveltraj([0 1], numel(time1+time2+tStep:tStep:time1+time2+time3));
[taskWaypoints3_sq, taskVelocities3_sq, taskAccel3_sq] = transformtraj(task2_sq, task3_sq, [time1+time2+tStep;time1+time2+time3], time1+time2+tStep:tStep:time1+time2+time3,'TimeScaling',[s3; sd3; sdd3]);
[s4,sd4,sdd4] = trapveltraj([0 1], numel(time1+time2+time3+tStep:tStep:time1+time2+time3+time4));
[taskWaypoints4_sq, taskVelocities4_sq, taskAccel4_sq] = transformtraj(task3_sq, task0_sq, [time1+time2+time3+tStep;time1+time2+time3+time4], time1+time2+time3+tStep:tStep:time1+time2+time3+time4,'TimeScaling',[s4; sd4; sdd4]);

taskWaypoints_sq = zeros(4,(time1+time2+time3+time4)/tStep);
for i=1:time1/tStep
    taskWaypoints_sq(:,i)= taskWaypoints1_sq(:,4,i);
end    
for i=1:time2/tStep
    taskWaypoints_sq(:,i+time1/tStep)= taskWaypoints2_sq(:,4,i);
end    
for i=1:time3/tStep
    taskWaypoints_sq(:,i+(time1+time2)/tStep)= taskWaypoints3_sq(:,4,i);
end    
for i=1:time4/tStep
    taskWaypoints_sq(:,i+(time1+time2+time3)/tStep)= taskWaypoints4_sq(:,4,i);
end  
taskVelocities_sq = [taskVelocities1_sq/time1,taskVelocities2_sq/time2,taskVelocities3_sq/time3,taskVelocities4_sq/time4];
taskAccel_sq = [taskAccel1_sq/time1,taskAccel2_sq/time2,taskAccel3_sq/time3,taskAccel4_sq/time4];
t_sq = [taskWaypoints1_sq(:,:),taskWaypoints2_sq(:,:),taskWaypoints3_sq(:,:),taskWaypoints4_sq(:,:)];


%% TASK_TRAJ_SQUARE_LSPB
subplot(2,4,5);
show(robot, Init, 'PreservePlot', false, 'Frames', 'off');
title('Square trajectory _ LSPB');
hold on
axis([-1 1 -1 1 -0.1 1.5]);
prev = Init

for i=1:(time1+time2+time3+time4)/tStep
    T_joint_sq = ik(endEffector, t_sq(1:4,4*(i-1)+1:4*i), weights, prev)
    show(robot, T_joint_sq, 'PreservePlot', false, 'Frames', 'off');
    prev = T_joint_sq;
    jointSpaceMarker = plot3(taskWaypoints_sq(1,i), taskWaypoints_sq(2,i), taskWaypoints_sq(3,i), 'b.', 'MarkerSize', 5);
    drawnow;
end
subplot(2,4,6);
plot(taskWaypoints_sq');
legend('x','y','z')
xlabel('t')
ylabel('Points')

subplot(2,4,7);
plot(taskVelocities_sq');
legend('th1','th2','th3','th4','th5','th6')
xlabel('t')
ylabel('Velocity')

subplot(2,4,8);
plot(taskAccel_sq');
legend('th1','th2','th3','th4','th5','th6')
xlabel('t')
ylabel('Acceleration')