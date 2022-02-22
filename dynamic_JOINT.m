%% GET ROBOT
robot = importrobot('robot.urdf')
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];
endEffector = 'link7'
numJoints = numel(homeConfiguration(robot));

initTime = 0.0;
Speed = 0.1;
tStep = 0.1;
time1 = 5;
time2 = 10;
time3 = 5;

%% JOINT SPACE
Init = [0 -pi/4 0 0 0 0];
th1 = [pi/3 pi/6 pi/3 0 pi/3 pi/6];
th2 = [-pi/3 pi/6 0 0 pi/3 0];

joint_tform0 = getTransform(robot, Init, endEffector);
joint_tform1 = getTransform(robot, th1, endEffector);
joint_tform2 = getTransform(robot, th2, endEffector);

joint_ctrlpoints1 = [Init', th1']; 
joint_ctrlpoints2 = [th1', th2'];
joint_ctrlpoints3 = [th2', Init'];

% cubic polynomial
[c_q1, c_qd1, c_qdd1, c_pp1] = cubicpolytraj(joint_ctrlpoints1,[tStep;time1], tStep:tStep:time1);
[c_q2, c_qd2, c_qdd2, c_pp2] = cubicpolytraj(joint_ctrlpoints2,[time1+tStep;time1+time2], time1+tStep:tStep:time1+time2);
[c_q3, c_qd3, c_qdd3, c_pp3] = cubicpolytraj(joint_ctrlpoints3,[time1+time2+tStep;time1+time2+time3], time1+time2+tStep:tStep:time1+time2+time3);

% lspb (trapzodial)
[l_q1, l_qd1, l_qdd1, l_tvec1, l_pp1] = trapveltraj(joint_ctrlpoints1, time1/tStep);
[l_q2, l_qd2, l_qdd2, l_tvec2, l_pp2] = trapveltraj(joint_ctrlpoints2, time2/tStep);
[l_q3, l_qd3, l_qdd3, l_tvec3, l_pp3] = trapveltraj(joint_ctrlpoints3, time3/tStep);


%% JOINT_CUBIC TRAJ

subplot(2,4,1);
show(robot, Init, 'PreservePlot', false, 'Frames', 'off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);

c_pos = [c_q1,c_q2,c_q3];
c_vel = [c_qd1,c_qd2,c_qd3];
c_acc = [c_qdd1,c_qdd2,c_qdd3];

for i=1:(time1+time2+time3)/tStep
    poseNow = getTransform(robot, c_pos(:,i)', endEffector);
    show(robot, c_pos(:,i)', 'PreservePlot', false, 'Frames', 'off');
    jointSpaceMarker = plot3(poseNow(1,4), poseNow(2,4), poseNow(3,4), 'g.', 'MarkerSize', 5);
    drawnow;
end
subplot(2,4,2);
plot(c_pos')
legend('th1','th2','th3','th4','th5','th6')
xlabel('t')
ylabel('Angle')

subplot(2,4,3);
plot(c_vel')
legend('th1','th2','th3','th4','th5','th6')
xlabel('t')
ylabel('Velocity')

subplot(2,4,4);
plot(c_acc')
legend('th1','th2','th3','th4','th5','th6')
xlabel('t')
ylabel('Acceleration')

%% JOINT_LSPB TRAJ
subplot(2,4,5);
show(robot, Init, 'PreservePlot', false, 'Frames', 'off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);

l_pos = [l_q1,l_q2,l_q3];
l_vel = [l_qd1,l_qd2,l_qd3]*tStep;
l_acc = [l_qdd1,l_qdd2,l_qdd3]*tStep;

for i=1:(time1+time2+time3)/tStep
    poseNow = getTransform(robot, l_pos(:,i)', endEffector);
    show(robot, l_pos(:,i)', 'PreservePlot', false, 'Frames', 'off');
    jointSpaceMarker = plot3(poseNow(1,4), poseNow(2,4), poseNow(3,4), 'b.', 'MarkerSize', 5);
    drawnow;
end
subplot(2,4,6);
plot(l_pos')
legend('th1','th2','th3','th4','th5','th6');
xlabel('t')
ylabel('Angle')

subplot(2,4,7);
plot(l_vel')
legend('th1','th2','th3','th4','th5','th6');
xlabel('t')
ylabel('Velocity')

subplot(2,4,8);
plot(l_acc')
legend('th1','th2','th3','th4','th5','th6')
xlabel('t')
ylabel('Acceleration')



