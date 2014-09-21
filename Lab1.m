%% Initiate connection
robot = neato('pico')

%% Challenge
timeArray = zeros(1,1)
distArray = zeros(1,1)

count = 1

leftStart = robot.encoders.data.left
leftEncoder = 0
tic
while leftEncoder < 200
    pause (0.001)
    robot.sendVelocity(0.05, 0.05)
    leftEncoder = robot.encoders.data.left - leftStart
    timeArray(count) = toc
    distArray(count) = leftEncoder/10
    count = count + 1
end
robot.sendVelocity(0, 0)
pause(2)
goal = leftEncoder-195
while leftEncoder > goal
    pause (0.001)
    robot.sendVelocity(-0.05, -0.05)
    leftEncoder = abs(robot.encoders.data.left - leftStart)
    timeArray(count) = toc
    distArray(count) = leftEncoder/10
    count = count + 1
end
robot.sendVelocity(0, 0)
plot(timeArray, distArray)
%% Terminate
robot.sendVelocity(0, 0)
robot.close()
clear all
clc