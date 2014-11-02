function Lab10(robot)
    global thePose;
    global RobotEstimate ;
      
    thePose = pose(.5,.5,pi/2);
    figure(1);
    LineMap.makeMap();
    robot.startLaser();
     
    t1 = tic;
    while(toc(t1) < 5)
       LineMap.testLineMap(robot); 
    end
    disp(thePose.getPoseVec());
    RobotEstimate = estRobot(thePose.x,thePose.y,thePose.th,robot);
    
    xf = 0.25; yf = 0.75; thf = pi()/2.0;
    Twg = [cos(thf) -sin(thf) xf ; sin(thf) cos(thf) yf ; 0 0 1];
    Twr = [cos(thePose.th) -sin(thePose.th) thePose.x ; sin(thePose.th) cos(thePose.th) thePose.y ; 0 0 1];
    Trw = inv(Twr);
    goal = Twg * Trw ;
    
    %xf = 0.75; yf = 0.25; thf = 0.0;
    %xf = 0.5; yf = 0.5; thf = pi()/2.0;
    disp([goal(1,3), goal(2,3), atan2(goal(2,1),goal(1,1))]);
    executeTrajectory(-goal(1,3),goal(2,3),atan2(goal(2,1),goal(1,1))),robot,2);
    t2 = tic;
    while(toc(t2) < 5)
       LineMap.testLineMap(robot); 
    end
    disp(thePose.getPoseVec());
    executeTrajectory(-.5,-.5,-pi/2.0,robot,2);
    executeTrajectory(-.25,.25,pi/2.0,robot,2);
    
    for run = 1:3
       if run == 1
            curve = cubicSpiral.planTrajectory(.25,.25,0,1);
            planVelocities(curve,.20);
       elseif run == 2
            curve = cubicSpiral.planTrajectory(-.5,-.5,-pi/2,1);
            planVelocities(curve,.20);
       elseif run == 3
            curve = cubicSpiral.planTrajectory(-.25,.25,pi/2,1);
            planVelocities(curve,.20);
       end
       for i = 1: length(curve.poseArray)
           xPredict(i) = curve.poseArray(1,i);
           yPredict(i) = curve.poseArray(2,i);
           if run == 1
               xy0 = [xPredict(i);yPredict(i);1];
           elseif run == 2
               xy0 = [1,0,0.25;0,1,0.25;0,0,1]*[xPredict(i);yPredict(i);1];
           elseif run == 3
               xy0 = [0,1,-0.25;-1,0,-0.25;0,0,1]*[xPredict(i);yPredict(i);1];
           end
           xP(i) = xy0(1,1);
           yP(i) = xy0(2,1);
        end
        figure(1);
        plot(xP,yP);
        hold on;
    end
    hold on;
    plot(RobotEstimate.x,RobotEstimate.y,'r');
    grid on;
    robot.sendVelocity(0,0);
end

function executeTrajectory(xf,yf,thf,robot,pauseTime)
    
    kpx = 1.5;
    kdx = .1;

    kppsi = .1;   
    kdpsi = .01;

    kpth = 3;   
    kdth = .3;

    width = .235;
    curve = cubicSpiral.planTrajectory(xf,yf,thf,1);
    planVelocities(curve,.20);
    
    for i = 1: length(curve.poseArray)
       xPredict(i) = curve.poseArray(1,i);
       yPredict(i) = curve.poseArray(2,i);
    end
    
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

        pause(.01);
    end
    
end