%Drive the robot in a figure 8 and plot the path traveled
function out = Lab3(r)
global robot; 
robot = r;
global x;
x=0;
global y;
y=0;
global th;
th=0;
global prevTime;
prevTime = 0;
global prevDistLeft;
prevDistLeft = robot.encoders.data.left;
global prevDistRight;
prevDistRight = robot.encoders.data.right;
global index;
index=1;

ISR = event.listener(robot.encoders,'OnMessageReceived',@neatoEncoderEventListener);

vl = 0;
vr = 0;
ks = .5;
kv = .4;
tf = 12.56*ks/kv;
time = tic;
while toc(time) < tf
 
    vr = (.3*kv + .14125*kv/ks*sin(toc(time)*kv/(2*ks)));
    vl = (.3*kv - .14125*kv/ks*sin(toc(time)*kv/(2*ks)));
    %disp([vr vl]);
    robot.sendVelocity(vl,vr);
    pause(.001);
end
    robot.sendVelocity(0,0);
    pause(1);
    disp('Distance from start in meters');
    disp(sqrt(power(x(end),2)+power(y(end),2)));
    
end