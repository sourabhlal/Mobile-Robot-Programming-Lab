function trapTest(robot,controlMethod)
vmax = .25;
amax = .75;
dist = 1;
tDelay = .22;

kp = 2;
kd = 0.2;
ki = 0.1;

index = 2;

time = tic;

iE = 0;
int = 0;

start = robot.encoders.data.left;

prevTime=0;

while toc(time) < 6
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
    
    if(controlMethod ~= 0)
        %calc d/dt(error)
        dE = (error(index) - error(index-1))/dt;
  
        %calc integral(error)
        iE = error(index)*dt;
   
        u = kp*error(index) + kd*dE + ki*iE;
        if (index<=2)
            disp('feedback + feedforward');
        end
    else
        u = 0;
        if (index<=2)
            disp('feedforward');
        end
    end
    sendV = v(index) + u;
    sendV  = min(.3,sendV);
    sendV  = min(.3,sendV);
    sendV  = max(-.3,sendV);
    sendV  = max(-.3,sendV);
    robot.sendVelocity(sendV,sendV);
    
    index = index+1;
    pause(.01);

end
disp(error(end));  

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