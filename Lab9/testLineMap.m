function testLineMap(robot,count) 
global thePose;
%check sensor
%     prevImage = robot.laser.data.ranges;
%     pause(1);
%     newImage = robot.laser.data.ranges;
%     if(prevImage == newImage)
%         disp('Error, sensor offline');
%         pause(100);
%     end

    % pick a pose
    %thePose = pose(0,0,0);
        
    %make map
    x1s = [0, 1.2];
    y1s = zeros(1,length(x1s));
    
    y2s = [1.2, 0];
    x2s = zeros(1,length(y2s));
    
    lines_p1 = [x1s;y1s ];
    lines_p2 = [x2s;y2s ];
    
%     plot(x1s,y1s);
%     hold on;
%     plot(x2s,y2s);
  
    gain = 0.01;
    errThresh = 0.001;
    gradThresh = 0.0005; 

    obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);

    index=1;
    %while(1==1)
       index = index+1;
        % Set up test points
        ranges = robot.laser.data.ranges;
        image = rangeImage(ranges,1,true);
        removeBadPoints(image);

        x1pts = image.xArray;
        y1pts = image.yArray;
        w1pts = ones(1,image.numPix);
   

        modelPts = [x1pts ; y1pts ; w1pts];
        
            figure(2);
            plot(0,0);
            hold on
            testPose = pose(robotModel.senToWorld(thePose));


            worldLidarPts = robotModel.senToWorld(thePose)*modelPts;
            worldLidarPts = [1 0 .1; 0 1 0; 0 0 1]*worldLidarPts;
            plot(worldLidarPts(1,:),worldLidarPts(2,:));
            plot( x1s,y1s,'r');
            plot(x2s,y2s,'r');


            graphPose =  [cos(testPose.th) -sin(testPose.th) testPose.x; sin(testPose.th) cos(testPose.th) testPose.y; 0 0 1] * robotModel.bodyGraph() ;
            graphPose = [1 0 .2; 0 1 0; 0 0 1]*graphPose;
            plot(graphPose(1,:),graphPose(2,:),'g');
            axis([-2 2 -2 2]);
            grid on;
            hold off
        
        [E(index), J] = getJacobian(obj,thePose,modelPts);

        %plot(E);
        %plot(thePose.y,thePose.x,'*');
        %hold on;
        %axis([-2, 2, -2, 2]);
        
        %disp(thePose.getPoseVec());
        thePose = pose(thePose.getPoseVec() - transpose(J)); 
        
        
    
        
    %end
end   




    
    