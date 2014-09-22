function trapTest(robot)
vmax = .25;
amax = .75;
dist = 1;

kp = 0;
kd = 0;
ki = 0;

tDelay = .2;

index = 2;

time = tic;

iE = 0;
int = 0;

start = robot.encoders.data.left;

prevTime=0;

while toc(time) < 5
    t =  toc(time) - tDelay;
    t = max(0,t);
    v(index) = trapezoidalVelocityProfile(toc(time), amax, vmax,dist, 1);
   
    graphTime(index) = toc(time);
    
    vref = trapezoidalVelocityProfile(t, amax, vmax,dist, 1);
 
    dt = toc(time) - prevTime;
    prevTime = toc(time);
    int = int + vref*dt;

    intP(index) = int;
    
    d(index) = (robot.encoders.data.left - start)/1000;
    
    error(index) = intP(index) - d(index);
        
    %calc d/dt(error)
    dE = (error(index) - error(index-1))/dt;
  
    %calc integral(error)
    iE = iE + error(index)*dt;
   
    u = kp*error(index) + kd*dE + ki*iE;
    sendV = v(index) + u;
    sendV  = min(.3,sendV);
    sendV  = min(.3,sendV);
    sendV  = max(-.3,sendV);
    sendV  = max(-.3,sendV);
    robot.sendVelocity(sendV,sendV);
    
    index = index+1;
    pause(.01);

end
  
figure;
subplot(2,2,1);
plot(graphTime,d);
hold on;
plot(graphTime,intP);
grid on;
subplot(2,2,2);
plot(graphTime,error);
grid on;

robot.sendVelocity(0,0);
end