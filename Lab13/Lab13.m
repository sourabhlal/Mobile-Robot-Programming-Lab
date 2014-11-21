function Lab13(robot)
    global thePose;
    global RobotEstimate ;
    
    thePose = pose(.23,.23,-pi/2);
    RobotEstimate = estRobot(thePose.x,thePose.y,thePose.th,robot);
    
    LineMap.makeMap();
    
    backUp(robot);
    getNewPose(robot);
    pickUpY =1.65;
    dropOffY = .30;
    
    %object 1
    pickUpPose = pose(.30,pickUpY,pi/2);
    dropOffPose = pose(.30,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    %object 2
    pickUpPose = pose(.30*2,pickUpY,pi/2);
    dropOffPose = pose(.30*2,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    %object 3
    pickUpPose = pose(.30*3,pickUpY,pi/2);
    dropOffPose = pose(.30*3,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    %object 4
    pickUpPose = pose(.30*4,pickUpY,pi/2);
    dropOffPose = pose(.30*4,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    %object 5 
    pickUpPose = pose(.30*5,pickUpY,pi/2);
    dropOffPose = pose(.30*5,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    %object 6 
    pickUpPose = pose(.30*6,pickUpY,pi/2);
    dropOffPose = pose(.30*6,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    %object 7 (dont attempt)
    %pickUpPose = pose(.30*7,pickUpY,pi/2);
    %dropOffPose = pose(.30*7,dropOffY,-pi/2);
    %getObject(pickUpPose,dropOffPose,robot);
    
    %object 8 (side) (dont attempt)
    %object 9 (side) (dont attempt)
    
    %object 10 (side) 
    pickUpPose = pose(.3*6,.6,0);
    dropOffPose = pose(.30*7,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    robot.sendVelocity(0,0);    
end


function getObject(pickUpPose,dropOffPose,robot)
    global thePose;
    pauseTime = .1;
    
    xf1p = pickUpPose.getPoseVec.x;
    yf1p = pickUpPose.getPoseVec.y;
    thf1p = pickUpPose.getPoseVec.th;
    
    xf1d = dropOffPose.getPoseVec.x;
    yf1d = dropOffPose.getPoseVec.y;
    thf1d = dropOffPose.getPoseVec.th;
    
    Ttw = [cos(thf1p) -sin(thf1p) xf1p ; sin(thf1p) cos(thf1p) yf1p ; 0 0 1];
    Trw = [cos(thePose.th) -sin(thePose.th) thePose.x; sin(thePose.th) cos(thePose.th) thePose.y; 0 0 1];
    Ttr = inv(Trw) * Ttw;
    executeTrajectory(Ttr(1,3),Ttr(2,3),atan2(Ttr(2,1),Ttr(2,2)),robot,pauseTime); %get close to target
    goToTarget(robot); % scan and go to pickup pose
    pickUp(robot);
    backUp(robot);
    getNewPose(robot);
    Ttw = [cos(thf1d) -sin(thf1d) xf1d ; sin(thf1d) cos(thf1d) yf1d ; 0 0 1];
    Trw = [cos(thePose.th) -sin(thePose.th) thePose.x; sin(thePose.th) cos(thePose.th) thePose.y; 0 0 1];
    Ttr = inv(Trw) * Ttw;
    executeTrajectory(Ttr(1,3)-.18,Ttr(2,3),atan2(Ttr(2,1),Ttr(2,2)),robot,pauseTime); %get close to target
    dropOff(robot);
    reverse(robot);

end


function pickUp(robot)
    robot.forksUp();
    pause(2);
end

function dropOff(robot)
    robot.forksDown();
    pause(2);
end


function goToTarget(robot)
  
    stopDistance = .2;
    sailSize = .125;
 
    ranges = robot.laser.data.ranges;
    image = rangeImage(ranges,1,true);
    
    removeLargeTheta(image,pi/6);

    [minError, bestNum, th, dist] = findLineCandidate(image,1,sailSize);
    bestIndex = 1;
    for i = 2:image.numPix
       [err, num, th, dist] = findLineCandidate(image,i,sailSize);

       if(err < 1000 && num > bestNum && dist > .08 )
          bestNum = num;
          bestIndex = i;
       end
    end
    [err, num, th, dist] = findLineCandidate(image,bestIndex,sailSize);
    x = image.xArray(bestIndex);
    y = image.yArray(bestIndex);


    % convert to robot coordinates     
    %object in sensor coordinates
    Tos = [cos(th) -sin(th) x ; sin(th) cos(th) y ; 0 0 1];
    %goal in object coordinates
    Tgo = [cos(0) -sin(0) 0; sin(0) cos(0) -stopDistance ; 0 0 1];
    %sensor in robot coordinates
    Tsr = [cos(0) -sin(0) -.075 ; sin(0) cos(0) 0 ; 0 0 1];

    %goal in robot coordinates
    Tgr = (Tsr * Tos) * Tgo;
    x = Tgr(1,3);
    y = Tgr(2,3);

    th = th+pi/2;
    if th > pi
        th = th-2*pi;
    end

    % move
    if(abs(th > pi/6))
        pause(1);
    end
    executeTrajectory(x,y,th,robot,1);
   
end

function reverse(robot)
    robot.sendVelocity(-.15,-.15);
    pause(1.5);
        

    robot.sendVelocity(-.2,.2);
    pause(.151);
    robot.sendVelocity(-.2,.2);
    
    pause(5);
    
    robot.sendVelocity(0,0);
end

function backUp(robot)

    robot.sendVelocity(-.1,.1);
    pause(.2);
    robot.sendVelocity(-.1,.1);
    
    pause(5);
    
    robot.sendVelocity(.15,.15);
    pause(2);
    robot.sendVelocity(0,0);
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