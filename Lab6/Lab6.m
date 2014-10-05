function Lab6(robot)
    executeTrajectory(.25,.25,0,robot);
    pause(5);
    executeTrajectory(-.5,-.5,-pi/2.0,robot);
    pause(5);
    executeTrajectory(-.25,.25,pi/2.0,robot);
    pause(5);

end

function executeTrajectory(xf,yf,thf,robot)
    width = .235;
    curve = planTrajectory(xf,yf,thf);
    
    time = tic();
    completionTime = curve.timeArray(end);
    while toc(time) < completionTime
        currTime = toc(time);
        V = getVatTime(currTime);
        w = getwAtTime(currTime);
        
        vl = V + (width/2)*w;
        vr = V - (width/2)*w;
        
        robot.sendVelocity(vl,vr);
        
        pause(.01);
    end
    
end