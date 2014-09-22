%Lab 4
function distanceBasedPID(robot)

kp = 6;
kd = 4;
ki = 0.1;

startL = robot.encoders.data.left;
startR = robot.encoders.data.right;


ul = 0; %modified left speed
ur = 0; %modified right speed

index = 2;

expectedDist = 0.1; %1 meter

time = tic;
iEr = 0;
iEl = 0;
dt = 0;
while toc(time) < 6
    ul = min(.3,ul);
    ur = min(.3,ur);
    ul = max(-.3,ul);
    ur = max(-.3,ur);
    disp([ul ur]);
    robot.sendVelocity(ul,ur);
    
    dt = toc(time) - dt;
    
    actualLeftDist =  (robot.encoders.data.left-startL)/1000;
    actualRightDist =  (robot.encoders.data.right-startR)/1000;
  
    
    %calc left/right error
    el(index) = expectedDist - actualLeftDist;
    er(index) = expectedDist - actualRightDist;
    
    %calc d/dt(error)
    dEr = (er(index) - er(index-1))/dt;
    dEl = (el(index) - el(index-1))/dt;
    
    %calc integral(error)
    iEr = er(index)*dt;
    iEl = el(index)*dt;
    
    %update speeds
    ul = kp*el(index) + kd*dEl + ki*iEl;
    ur = kp*er(index) + kd*dEr + ki*iEr;
    
    index = index+1;
    pause(.01);
    
end
plot(el);
grid on;
robot.sendVelocity(0,0);
end