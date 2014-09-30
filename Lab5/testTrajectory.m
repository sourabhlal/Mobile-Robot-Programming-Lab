%main function for Lab 5
function testTrajectory(robot)
   
    if 1==1
        kpx = 1.5;
        kdx = .1;

        kppsi = .1;   
        kdpsi = .01;
        
        kpth = 3;   
        kdth = .3;
    else
        kpx = 0;
        kdx = 0;
        kppsi = 0;   
        kdpsi = 0;
        kpth = 0;   
        kdth = 0;
    end
    
    %obj = figure8ReferenceControl(.5,.5,1);
    obj = trapezoidalStepReferenceControl(.75,.25,1,1,1);
    
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
        currTime(index+1) = toc(time);
        dt = currTime(index+1) - prevTime;

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

        targetPose = getPoseAtTime(trajectory,obj, currTime(index+1));
       
        Twb = inv([cos(th) -sin(th) x(index+1) ; sin(th) cos(th) y(index+1) ; 0 0 1]);
        raw = [targetPose(1,1) ; targetPose(2,1) ; 1];
        rab = Twb*raw;
        xError(index+1) = rab(1,1);
        yError(index+1) = rab(2,1);

        %calc angle between robot and predicted pose
        Taw = [cos(targetPose(3,1)) -sin(targetPose(3,1)) targetPose(1,1) ; sin(targetPose(3,1)) cos(targetPose(3,1)) targetPose(2,1) ; 0 0 1];
        Tab = Twb*Taw;
        thError(index+1) = atan2(Tab(2,1),Tab(1,1));
        
        psiError(index+1) = atan2(yError(index+1),xError(index+1));
        
        rabx_dt = (xError(index+1)-xError(index))*dt; 
        dpsi_dt = (psiError(index+1)-psiError(index))*dt;
        dth_dt = (thError(index+1)-thError(index))*dt;
           
        mag = 10*sqrt(xError(index+1)*xError(index+1)+yError(index+1)*yError(index+1));
        kppsi_mag = min(kppsi/mag,3*kppsi);
        kdpsi_mag = min(kdpsi/mag,3*kdpsi);
        
        [V, w] = computeControl(obj, toc(time));
        V = V + kpx*xError(index+1) + kdx*rabx_dt;
        if xError(index+1) > 0 
           w = w + kppsi_mag*psiError(index+1) + kdpsi_mag*dpsi_dt + kpth*thError(index+1) + kdth*dth_dt;
        else
            w = w + kpth*thError(index+1) + kdth*dth_dt;    
        end
        
        rm = robotModel(width);
        [vl, vr] = VwTovlvr(rm,V,w);
        
        vl  = min(.3,vl);
        vr  = min(.3,vr);
        vl  = max(-.3,vl);
        vr  = max(-.3,vr);

        robot.sendVelocity(vl,vr);
        
        prevTime = currTime(index+1);
        index = index+1;
        pause(.01);
    end

     figure(1);
     subplot(2,3,1);
     plot(y,x,'r');
     hold on;
     title('Path');
     plot(yp,xp);
     subplot(2,3,2);
     plot(currTime,xError);
     title('X Error');
     grid on;
     subplot(2,3,3);
     plot(currTime,yError);
     title('Y Error');
     grid on;
     subplot(2,3,4);
     plot(currTime,thError);
     title('Theta Error');
     grid on;
     subplot(2,3,5);
     plot(currTime,psiError);
     title('Psi Error');
     grid on;
     subplot(2,3,6);
     plot(currTime,sqrt(xError.*xError + yError.*yError));
     title('Distance from target');
     grid on;
     
     disp(sqrt(xError(end)*xError(end) + yError(end)*yError(end)));
     
    robot.sendVelocity(0,0);
    %clear classes;
end





