function [robot] = create_robot_tree()
% Create associated robot tree to visualize things
robot = rigidBodyTree("DataFormat","column");
base = robot.Base;
arm1 = rigidBody("arm1");
arm2 = rigidBody("arm2");

jnt1 = rigidBodyJoint("jnt1","revolute");
jnt2 = rigidBodyJoint("jnt2","revolute");

jnt1.JointAxis = [1 0 0]; % y-axis
jnt1.HomePosition = pi/4;
jnt2.JointAxis = [1 0 0];
jnt2.HomePosition = pi/4;

setFixedTransform(jnt1,trvec2tform([0 1 0]))
setFixedTransform(jnt2,trvec2tform([0 1 0]))

arm1.Joint = jnt1;
arm2.Joint = jnt2;

addBody(robot,arm1,"base");
addBody(robot,arm2,"arm1");

showdetails(robot)
show(robot)
end

