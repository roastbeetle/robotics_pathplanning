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

%% TASK SPACE 1_TRIANGLE_CUBIC

task0_tr = getTransform(robot, Init, endEffector);
task1_tr = trvec2tform([0.3, 0.3, 0.2])*axang2tform([0 1 0 pi]);
task2_tr = trvec2tform([0.3, -0.3, 0.3])*axang2tform([0 1 0 pi]);

[s1,sd1,sdd1] = cubicpolytraj([0 1], [tStep;time1],tStep:tStep:time1);
[taskWaypoints1_tr, taskVelocities1_tr, taskAccel1_tr] = transformtraj(task0_tr, task1_tr, [tStep;time1],tStep:tStep:time1,'TimeScaling',[s1; sd1; sdd1]);
[s2,sd2,sdd2] = cubicpolytraj([0 1], [time1+tStep;time1+time2],time1+tStep:tStep:time1+time2);
[taskWaypoints2_tr, taskVelocities2_tr, taskAccel2_tr] = transformtraj(task1_tr, task2_tr, [time1+tStep;time1+time2], time1+tStep:tStep:time1+time2,'TimeScaling',[s2; sd2; sdd2]);
[s3,sd3,sdd3] = cubicpolytraj([0 1], [time1+time2+tStep;time1+time2+time3], time1+time2+tStep:tStep:time1+time2+time3);
[taskWaypoints3_tr, taskVelocities3_tr, taskAccel3_tr] = transformtraj(task2_tr, task0_tr, [time1+time2+tStep;time1+time2+time3], time1+time2+tStep:tStep:time1+time2+time3,'TimeScaling',[s3; sd3; sdd3]);

taskWaypoints_tr = zeros(4,(time1+time2+time3)/tStep);
for i=1:time1/tStep
    taskWaypoints_tr(:,i)= taskWaypoints1_tr(:,4,i);
end    
for i=1:time2/tStep
    taskWaypoints_tr(:,i+time1/tStep)= taskWaypoints2_tr(:,4,i);
end    
for i=1:time3/tStep
    taskWaypoints_tr(:,i+(time1+time2)/tStep)= taskWaypoints3_tr(:,4,i);
end    
taskVelocities_tr = [taskVelocities1_tr,taskVelocities2_tr,taskVelocities3_tr];
taskAccel_tr = [taskAccel1_tr,taskAccel2_tr,taskAccel3_tr];
t_tr = [taskWaypoints1_tr(:,:),taskWaypoints2_tr(:,:),taskWaypoints3_tr(:,:)];

ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

%% TASK_TRAJ_TRIANGLE_CUBIC
subplot(2,4,1);
show(robot, Init, 'PreservePlot', false, 'Frames', 'off');
title('Triangle trajectory _ CUBIC');
hold on
axis([-1 1 -1 1 -0.1 1.5]);
prev = Init

for i=1:(time1+time2+time3)/tStep
    T_joint_tr = ik(endEffector, t_tr(1:4,4*(i-1)+1:4*i), weights, prev);
    show(robot, T_joint_tr, 'PreservePlot', false, 'Frames', 'off');
    prev = T_joint_tr;
    jointSpaceMarker = plot3(taskWaypoints_tr(1,i), taskWaypoints_tr(2,i), taskWaypoints_tr(3,i), 'g.', 'MarkerSize', 5);
    drawnow;
end

subplot(2,4,2);
plot(taskWaypoints_tr');
legend('x','y','z')
xlabel('t')
ylabel('Points')

subplot(2,4,3);
plot(taskVelocities_tr');
legend('th1','th2','th3','th4','th5','th6')
xlabel('t')
ylabel('Velocity')

subplot(2,4,4);
plot(taskAccel_tr');
legend('th1','th2','th3','th4','th5','th6')
xlabel('t')
ylabel('Acceleration')


%% TASK SPACE 1_TRIANGLE_LSPB

task0_tr = getTransform(robot, Init, endEffector);
task1_tr = trvec2tform([0.3, 0.3, 0.2])*axang2tform([0 1 0 pi]);
task2_tr = trvec2tform([0.3, -0.3, 0.3])*axang2tform([0 1 0 pi]);

[s1,sd1,sdd1] = trapveltraj([0 1],numel(tStep:tStep:time1));
[taskWaypoints1_tr, taskVelocities1_tr, taskAccel1_tr] = transformtraj(task0_tr, task1_tr, [tStep;time1],tStep:tStep:time1,'TimeScaling',[s1; sd1; sdd1]);
[s2,sd2,sdd2] = trapveltraj([0 1],numel(time1+tStep:tStep:time1+time2));
[taskWaypoints2_tr, taskVelocities2_tr, taskAccel2_tr] = transformtraj(task1_tr, task2_tr, [time1+tStep;time1+time2], time1+tStep:tStep:time1+time2,'TimeScaling',[s2; sd2; sdd2]);
[s3,sd3,sdd3] = trapveltraj([0 1],numel(time1+time2+tStep:tStep:time1+time2+time3));
[taskWaypoints3_tr, taskVelocities3_tr, taskAccel3_tr] = transformtraj(task2_tr, task0_tr, [time1+time2+tStep;time1+time2+time3], time1+time2+tStep:tStep:time1+time2+time3,'TimeScaling',[s3; sd3; sdd3]);

taskWaypoints_tr = zeros(4,(time1+time2+time3)/tStep);
for i=1:time1/tStep
    taskWaypoints_tr(:,i)= taskWaypoints1_tr(:,4,i);
end    
for i=1:time2/tStep
    taskWaypoints_tr(:,i+time1/tStep)= taskWaypoints2_tr(:,4,i);
end    
for i=1:time3/tStep
    taskWaypoints_tr(:,i+(time1+time2)/tStep)= taskWaypoints3_tr(:,4,i);
end    
taskVelocities_tr = [taskVelocities1_tr/time1,taskVelocities2_tr/time2,taskVelocities3_tr/time3];
taskAccel_tr = [taskAccel1_tr/time1,taskAccel2_tr/time2,taskAccel3_tr/time3];
t_tr = [taskWaypoints1_tr(:,:),taskWaypoints2_tr(:,:),taskWaypoints3_tr(:,:)];

ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

%% TASK_TRAJ_TRIANGLE_LSPB
subplot(2,4,5);
show(robot, Init, 'PreservePlot', false, 'Frames', 'off');
title('Triangle trajectory _ LSPB');
hold on
axis([-1 1 -1 1 -0.1 1.5]);
prev = Init

for i=1:(time1+time2+time3)/tStep
    T_joint_tr = ik(endEffector, t_tr(1:4,4*(i-1)+1:4*i), weights, prev);
    show(robot, T_joint_tr, 'PreservePlot', false, 'Frames', 'off');
    prev = T_joint_tr;
    jointSpaceMarker = plot3(taskWaypoints_tr(1,i), taskWaypoints_tr(2,i), taskWaypoints_tr(3,i), 'g.', 'MarkerSize', 5);
    drawnow;
end
subplot(2,4,6);
plot(taskWaypoints_tr');
legend('x','y','z')
xlabel('t')
ylabel('Points')

subplot(2,4,7);
plot(taskVelocities_tr');
legend('th1','th2','th3','th4','th5','th6')
xlabel('t')
ylabel('Velocity')

subplot(2,4,8);
plot(taskAccel_tr');
legend('th1','th2','th3','th4','th5','th6')
xlabel('t')
ylabel('Acceleration')

