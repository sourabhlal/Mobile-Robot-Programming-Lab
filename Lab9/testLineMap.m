function testLineMap(robot)
   
    % pick a pose
    thePose = pose(.5,.5,0);
        
    %make map
    x1s = [0:.2:2];
    y1s = zeros(1,length(x1s));
    
    y2s = [0:.2:2];
    x2s = zeros(1,length(y2s));
    
    lines_p1 = [x1s;y1s ];
    lines_p2 = [x2s;y2s ];
    
    
    gain = 0.01;
    errThresh = 0.001;
    gradThresh = 0.0005; 

    obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);

    while(1==1)
        % Set up test points
        ranges = robot.laser.data.ranges;
        image = rangeImage(ranges,1,true);
        removeBadPoints(image);

        x1pts = image.xArray;
        y1pts = image.yArray;
        w1pts = ones(1,image.numPix);

        modelPts = [x1pts ; y1pts ; w1pts];
       
        
        [E, J] = getJacobian(obj,thePose,modelPts);
      
%         plot(modelPts(1,:),modelPts(2,:));
%         hold on;
%         plot (lines_p1(1,:),lines_p1(2,:),'-.r');   
%         plot (lines_p2(1,:),lines_p2(2,:),'-.r');
%         axis equal;
        
        
        v = J(1);
        v = max(-.15,v);
        v = min(.15,v);
        disp(v);
        %robot.sendVelocity(v,v);
        %pause(.5);
    
    end
    
end