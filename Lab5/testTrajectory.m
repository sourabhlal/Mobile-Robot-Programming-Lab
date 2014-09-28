%main function for Lab 5
function testTrajectory(robot)
    obj = figure8ReferenceControl(.5,.5,0);
    
    totalTime = getTrajectoryDuration(obj);
   
    %predicted path
    maxTime = getTrajectoryDuration(obj);
    trajectory = robotTrajectory(1000);
    calcPose(trajectory,obj);
     for i = 1:1000
         pose = getPoseAtTime(trajectory,obj, i/1000*maxTime);
         xp(i) = pose(1,1);
         yp(i) = pose(2,1);
         th(i) = pose(3,1);
     end
    
    prevDistLeft = robot.encoders.data.left;
    prevDistRight = robot.encoders.data.right;
    th=0;
    x(1) = 0;
    y(1) = 0;
    width = .237;

    time = tic;
    prevTime = toc(time);
    index =1;
    while toc(time) < totalTime
        [V, w] = computeControl(obj, toc(time));
        rm = robotModel(width);
        [vl, vr] = VwTovlvr(rm,V,w);
        robot.sendVelocity(vl,vr);
        
        currTime = toc(time);
        dt = currTime - prevTime;
        prevTime = currTime;

        currDistRight = robot.encoders.data.right;
        dSr = (currDistRight - prevDistRight)/1000; 
        prevDistRight = currDistRight;

        currDistLeft = robot.encoders.data.left;
        dSl = (currDistLeft - prevDistLeft)/1000; 
        prevDistLeft = currDistLeft;

        dth = (dSr - dSl)/width;
        dS = (dSr+dSl)/2;

        th = th + dth/2;
        x(index+1) = x(index) + (dS/dt)*cos(th)*dt;
        y(index+1) = y(index) + (dS/dt)*sin(th)*dt;
        th = th + dth/2;

        p = getPoseAtTime(trajectory,obj, currTime);
        xError(index) = p(1,1) - x(index+1);
        yError(index) = p(2,1) - y(index+1);
        thError(index) = p(3,1) - th;
        
        index = index+1;
        pause(.01);
    end
    
    figure(1);
    subplot(2,2,1)
    plot(y,x,'r');
    hold on;
    plot(yp,xp);
    subplot(2,2,2)
    plot(xError);
    subplot(2,2,3)
    plot(yError);
    subplot(2,2,4)
    plot(thError);
    
    
    robot.sendVelocity(0,0);
    %clear classes;
end





