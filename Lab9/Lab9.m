function Lab9(robot)
    global RobotEstimate ;
    RobotEstimate = estRobot(0,0,0,robot);
    robot.startLaser()
    pause(1);
    
    stopDistance = .2;
    sailSize = .125;
    MAX_LINE_ERROR = 1000;
    
    found = false;
    % find sail
        while (found == false)
            objectData = load('objectData.mat');
            %ranges = objectData.laserRanges(4,:);
            ranges = robot.laser.data.ranges;
            image = rangeImage(ranges,1,true);

            plotRvsTh(image, 1.5);
            plotXvsY(image, 1.5);
            
            [minError, num, th, dist] = findLineCandidate(image,1,sailSize);
            bestIndex = 1;
            for i = 2:image.numPix
               [err, num, th, dist] = findLineCandidate(image,i,sailSize);
               %disp(err);
               if(err < minError && num > 1 && dist > .08)
                  minError = err;
                  bestIndex = i;
               end
            end
            [err, num, th, dist] = findLineCandidate(image,bestIndex,sailSize);
            %if  num >= 3 && err < MAX_LINE_ERROR 
                found = true;
                x = image.xArray(bestIndex);
                y = image.yArray(bestIndex);
              
            %else
            %    pause(.05);
            %end
        end
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
        
        
        disp([x y (th/pi)*180]);
    % move
        executeTrajectory(x,y,th,robot,2);
        axis equal;
    robot.sendVelocity(0,0);
end


function executeTrajectory(xf,yf,thf,robot,pauseTime) 
  global RobotEstimate ;
    RobotEstimate = estRobot(0,0,0,robot);
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
    hold on;
    figure(1);
    plot(-yPredict,xPredict);
    axis equal
    
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
        figure(1);
        plot(-RobotEstimate.y,RobotEstimate.x,'r');
        axis equal
    
        currTime = toc(time);
        if currTime < (completionTime)
            t =  currTime - .22;
            t = max(0,t);
            V = getVAtTime(curve,t);
            w = getwAtTime(curve,t);
            targetPose = getPoseAtTime(curve,t);
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