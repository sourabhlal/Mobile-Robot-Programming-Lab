%main function for Lab 5
function testTrajectory(robot)
    kpx = 1;
    kpy = 1;
    kdx = .2;
    kdy = .2;
        
    obj = figure8ReferenceControl(.5,.5,5);
    
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
        currTime = toc(time);
        dt = currTime - prevTime;

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

        targetPose = getPoseAtTime(trajectory,obj, currTime);
       
        Twb = inv([cos(th) -sin(th) x(index+1) ; sin(th) cos(th) y(index+1) ; 0 0 1]);
        raw = [targetPose(1,1) ; targetPose(2,1) ; 1];
        rab = Twb*raw;
        xError(index+1) = rab(1,1);
        yError(index+1) = rab(2,1);
        
        
        [V, w] = computeControl(obj, toc(time));
        rm = robotModel(width);
        [vl, vr] = VwTovlvr(rm,V,w);
        
        rab_ddx = (xError(index+1)-xError(index))*dt; 
        rab_ddy = (yError(index+1)-yError(index))*dt;
        
        vl = vl + kpx*rab(1,1) - kpy*rab(2,1) + kdx*rab_ddx - kdy*rab_ddy;
        vr = vr + kpx*rab(1,1) + kpy*rab(2,1) + kdx*rab_ddx + kdy*rab_ddy;
        
        vl  = min(.3,vl);
        vr  = min(.3,vr);
        vl  = max(-.3,vl);
        vr  = max(-.3,vr);

        robot.sendVelocity(vl,vr);
       
        index = index+1;
        prevTime = currTime;
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
    %plot(thError);
    
    
    robot.sendVelocity(0,0);
    %clear classes;
end





