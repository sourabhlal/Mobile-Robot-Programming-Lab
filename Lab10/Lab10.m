function Lab10(robot)
    global thePose;
    global RobotEstimate ;

    thePose = pose(.5,.5,pi/2);
    
    LineMap.makeMap();
    robot.startLaser();
     
    t1 = tic;
    while(toc(t1) < 5)
       disp(thePose.getPoseVec());
       LineMap.testLineMap(robot); 
    end
    thePose = pose(thePose.x + .10*cos(thePose.th),thePose.y + .10*sin(thePose.th),thePose.th);
    
    disp(thePose.getPoseVec());
    RobotEstimate = estRobot(thePose.x,thePose.y,thePose.th,robot);
    
    xf = 0.25; yf = 0.75; thf = pi()/2.0;
    Twg = [cos(thf) -sin(thf) xf ; sin(thf) cos(thf) yf ; 0 0 1];
    Twr = [cos(thePose.th) -sin(thePose.th) thePose.x ; sin(thePose.th) cos(thePose.th) thePose.y ; 0 0 1];
    Trw = inv(Twr);
    goal = Trw * Twg ;
    
    disp(goal);
    executeTrajectory(goal(1,3),goal(2,3),atan2(goal(2,1),goal(1,1)),robot,2);
    
    getNewPose(robot);
    
    xf = 0.75; yf = 0.25; thf = 0.0;
    Twg = [cos(thf) -sin(thf) xf ; sin(thf) cos(thf) yf ; 0 0 1];
    Twr = [cos(thePose.th) -sin(thePose.th) thePose.x ; sin(thePose.th) cos(thePose.th) thePose.y ; 0 0 1];
    Trw = inv(Twr);
    goal = Trw * Twg ;
    
    executeTrajectory(goal(1,3),goal(2,3),atan2(goal(2,1),goal(1,1)),robot,.1);
    robot.sendVelocity(-.05,-.05);
    pause(.8)
    robot.sendVelocity(0,0);
    
    getNewPose(robot);
    
    xf = 0.5; yf = 0.5; thf = pi()/2.0;
    Twg = [cos(thf) -sin(thf) xf ; sin(thf) cos(thf) yf ; 0 0 1];
    Twr = [cos(thePose.th) -sin(thePose.th) thePose.x ; sin(thePose.th) cos(thePose.th) thePose.y ; 0 0 1];
    Trw = inv(Twr);
    goal = Trw * Twg ;
    
    executeTrajectory(goal(1,3),goal(2,3),atan2(goal(2,1),goal(1,1)),robot,.1);
    robot.sendVelocity(-.05,-.05);
    pause(.5)
    robot.sendVelocity(0,0);
    
    
    getNewPose(robot);
 
end

function getNewPose(robot)
    global thePose;
    global RobotEstimate ;
    timer = tic;
    thePose = pose(RobotEstimate.x(end),RobotEstimate.y(end),RobotEstimate.th);
    while(toc(timer) < 5)
       disp(thePose.getPoseVec());
       LineMap.testLineMap(robot); 
    end
    thePose = pose(thePose.x + .10*cos(thePose.th),thePose.y + .10*sin(thePose.th),thePose.th);
    RobotEstimate = estRobot(thePose.x,thePose.y,thePose.th,robot);
    disp(thePose.getPoseVec());
end


function executeTrajectory(xf,yf,thf,robot,pauseTime)
    global thePose;
    global RobotEstimate;

    kpx = 0;
    kdx = 0;%.1;

    kppsi = 0;   
    kdpsi = 0;%.01;

    kpth = 0;   
    kdth = 0;%.3;

    width = .235;
    curve = cubicSpiral.planTrajectory(xf,yf,thf,1);
    planVelocities(curve,.20);
    
    for i = 1: length(curve.poseArray)
       xPredict(i) = curve.poseArray(1,i);
       yPredict(i) = curve.poseArray(2,i);
       th = atan2(yPredict(i),xPredict(i));
       Trg = [cos(th) -sin(th) xPredict(i) ; sin(th) cos(th) yPredict(i) ; 0 0 1];
       Twr = [cos(thePose.th) -sin(thePose.th) thePose.x ; sin(thePose.th) cos(thePose.th) thePose.y ; 0 0 1];
       Twg = Twr * Trg; 
       xP(i) = Twg(1,3) ;
       yP(i) = Twg(2,3) ;
    end
    figure(1)
    plot(xP,yP);
    hold on;
    
    prevDistRight = robot.encoders.data.right;
    prevDistLeft = robot.encoders.data.left;
    
    time = tic();
    prevTime = 0;
    completionTime = curve.timeArray(end);
    index=1;
    th=0;
    xActual(1) = 0;
    yActual(1) = 0;  
    while toc(time) < (completionTime + pauseTime)
        currTime = toc(time);
        if currTime < completionTime
            V = getVAtTime(curve,currTime);
            w = getwAtTime(curve,currTime);
            targetPose = getPoseAtTime(curve,currTime);
        else
            V = 0;
            w = 0;
            targetPose = getPoseAtTime(curve,completionTime);
        end
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
        xActual(index+1) = xActual(index) + (dS/dt)*cos(th)*dt;
        yActual(index+1) = yActual(index) + (dS/dt)*sin(th)*dt;
        th = th + dth/2;
        
        Twb = inv([cos(th) -sin(th) xActual(index+1) ; sin(th) cos(th) yActual(index+1) ; 0 0 1]);
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
        
        V = V + kpx*xError(index+1) + kdx*rabx_dt;
        if xError(index+1) > 0 
           w = w + kppsi_mag*psiError(index+1) + kdpsi_mag*dpsi_dt + kpth*thError(index+1) + kdth*dth_dt;
        else
           w = w + kpth*thError(index+1) + kdth*dth_dt;    
        end
        
        vr = V + (width/2)*w;
        vl = V - (width/2)*w; 
        
        vl  = min(.3,vl);
        vr  = min(.3,vr);
        vl  = max(-.3,vl);
        vr  = max(-.3,vr);

        robot.sendVelocity(vl,vr);
                
        index = index+1;
        prevTime = currTime;

        if(RobotEstimate.x(end) < 2 && RobotEstimate.x(end) > 0 ...
            && RobotEstimate.y(end) < 2 && RobotEstimate.y(end) > 0)
            plot(RobotEstimate.x(end),RobotEstimate.y(end),'.r');
        end
        axis([0 2 0 2]);
        axis equal;
        hold on;
        grid on;
        
        pause(.1);
    end
    
end