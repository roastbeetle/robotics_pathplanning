robot = importrobot('robot.urdf')
showdetails(robot);

th1 = struct;
th1.JointName = 'base_link1';
th1.JointPosition = 0;

th2 = struct;
th2.JointName = 'link1_link2';
th2.JointPosition = 0;

th3 = struct;
th3.JointName = 'link2_link3';
th3.JointPosition = 0;

th4 = struct;
th4.JointName = 'link3_link4';
th4.JointPosition = 0;

th5 = struct;
th5.JointName = 'link4_link5';
th5.JointPosition = 0;

th6 = struct;
th6.JointName = 'link5_link6';
th6.JointPosition = 0;

th = [th1 th2 th3 th4 th5 th6 ];
tform = getTransform(robot, th,'link7','base');

%show(robot, th)

ik = inverseKinematics('RigidBodyTree',robot);
weights = [1 1 1 1 1 1 ];
initialguess = robot.homeConfiguration;

[configSoln, solnInfo] = ik('link7', tform, weights, initialguess);
show(robot,configSoln);

struct2table(th)
struct2table(configSoln)


