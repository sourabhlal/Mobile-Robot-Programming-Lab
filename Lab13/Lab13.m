function Lab13(robot)
    global thePose;
    global RobotEstimate ;
    
    thePose = pose(.23,.23,-pi/2);
    
    RobotEstimate = estRobot(thePose.x,thePose.y,thePose.th,robot);
    
    LineMap.makeMap();
    robot.forksDown();
    
    robot.sendVelocity(-.1,.1);
    pause(1.8);
    robot.sendVelocity(.2,.2);
    pause(3);
    robot.sendVelocity(.2,.2);
    pause(1);
    
    getNewPose(robot);
    
    pickUpY =1.35;
    dropOffY = .28;
    
    %object 10 (side) 
    pickUpPose = pose(1.9,.6,0);
    dropOffPose = pose(.30,dropOffY,-pi/2);
    getSideObject(pickUpPose, dropOffPose, robot);
    
    
    %object 1
    pickUpPose = pose(.30,pickUpY,pi/2);
    dropOffPose = pose(.30*2,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    %object 2
    pickUpPose = pose(.30*2,pickUpY,pi/2);
    dropOffPose = pose(.30*3,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    %object 3
    pickUpPose = pose(.30*3,pickUpY,pi/2);
    dropOffPose = pose(.30*4,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    %object 4
    pickUpPose = pose(.30*4,pickUpY,pi/2);
    dropOffPose = pose(.30*5,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    %object 5 
    pickUpPose = pose(.30*5,pickUpY,pi/2);
    dropOffPose = pose(.30*6,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    %object 6 
    pickUpPose = pose(.30*6,pickUpY,pi/2);
    dropOffPose = pose(.30*7,dropOffY,-pi/2);
    getObject(pickUpPose,dropOffPose,robot);
    
    %object 7 (dont attempt)
    %pickUpPose = pose(.30*7,pickUpY,pi/2);
    %dropOffPose = pose(.30*7,dropOffY,-pi/2);
    %getObject(pickUpPose,dropOffPose,robot);
    
    %object 8 (side) (dont attempt)
    %object 9 (side) (dont attempt)
    
    
    
    robot.sendVelocity(0,0);    
end


function getObject(pickUpPose,dropOffPose,robot)
    global thePose;
    pauseTime = .01;
    
    xf1p = pickUpPose.x;
    yf1p = pickUpPose.y;
    thf1p = pickUpPose.th;
    
    xf1d = dropOffPose.x;
    yf1d = dropOffPose.y;
    thf1d = dropOffPose.th;
    
    getNewPose(robot);
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

function getSideObject(pickUpPose,dropOffPose,robot)
    global thePose;
    pauseTime = .01;
    
    xf1p = pickUpPose.x;
    yf1p = pickUpPose.y;
    thf1p = pickUpPose.th;
    
    xf1d = dropOffPose.x;
    yf1d = dropOffPose.y;
    thf1d = dropOffPose.th;
    
    getNewPose(robot);
    Ttw = [cos(thf1p) -sin(thf1p) xf1p ; sin(thf1p) cos(thf1p) yf1p ; 0 0 1];
    Trw = [cos(thePose.th) -sin(thePose.th) thePose.x; sin(thePose.th) cos(thePose.th) thePose.y; 0 0 1];
    Ttr = inv(Trw) * Ttw;
    executeTrajectory(Ttr(1,3),Ttr(2,3),atan2(Ttr(2,1),Ttr(2,2)),robot,pauseTime); %get close to target
    
    robot.sendVelocity(.15,.15);
    pause(.8);
    pickUp(robot);
    backUp(robot);
    robot.sendVelocity(.2,.2);
    pause(4);
    robot.sendVelocity(.2,.2);
    pause(4);
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

    th=-pi/2;
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
%     if(abs(th > pi/6))
%         pause(1);
%     end
    executeTrajectory(x,y,th,robot,.1);
    robot.sendVelocity(.1,.1);
    pause(1);
    robot.sendVelocity(0,0);
end


function reverse(robot)
    robot.sendVelocity(-.2,-.2);
    pause(1.5);
        

    robot.sendVelocity(-.2,.2);
    pause(.151);
    robot.sendVelocity(-.2,.2);
    
    pause(4);
    
    robot.sendVelocity(0,0);
end

function backUp(robot)

    robot.sendVelocity(-.1,.1);
    pause(.2);
    robot.sendVelocity(-.1,.1);
    pause(4);
    
    robot.sendVelocity(.2,.2);
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

    width = .235;
    curve = cubicSpiral.planTrajectory(xf,yf,thf,1);
    planVelocities(curve,.22);
    
    time = tic();
 
    completionTime = curve.timeArray(end);

    while toc(time) < (completionTime + pauseTime)
        currTime = toc(time);
        if currTime < completionTime
            V = getVAtTime(curve,currTime);
            w = getwAtTime(curve,currTime);  
        else
            V = 0;
            w = 0;
        end
       
        vr = V + (width/2)*w;
        vl = V - (width/2)*w; 
        
        vl  = min(.3,vl);
        vr  = min(.3,vr);
        vl  = max(-.3,vl);
        vr  = max(-.3,vr);

        robot.sendVelocity(vl,vr);
                
     
        pause(.05);
    end
    
    
    
    
end