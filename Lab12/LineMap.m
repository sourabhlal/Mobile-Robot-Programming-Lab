classdef LineMap

    methods(Static = true)
        function testLineMap(robot) 
            global thePose;
            global theMap;

            % Set up test points
            ranges = robot.laser.data.ranges;
            image = rangeImage(ranges,1,true);
            removeBadPoints(image);
            
            i=1;
            numPoints = 1;
            x1pts(1) = 0;
            y1pts(1) = 0;
            w1pts(1) = 0;
             
            while i < image.numPix;
                x1pts(numPoints) = image.xArray(i);
                y1pts(numPoints) = image.yArray(i);
                w1pts(numPoints) = 1;
                numPoints = numPoints + 1;
                i = i+5;
            end
            
            modelPts = [x1pts ; y1pts ; w1pts];

            %figure(2);
            %plot(0,0);
            %hold on
            testPose = pose(robotModel.senToWorld(thePose));

            worldLidarPts = robotModel.senToWorld(thePose)*modelPts;
            worldLidarPts = [1 0 .1; 0 1 0; 0 0 1]*worldLidarPts;
            %plot(worldLidarPts(1,:),worldLidarPts(2,:));
            %plot( x1s,y1s,'r');
            %plot(x2s,y2s,'r');


            graphPose =  [cos(testPose.th) -sin(testPose.th) testPose.x; sin(testPose.th) cos(testPose.th) testPose.y; 0 0 1] * robotModel.bodyGraph() ;
            graphPose = [1 0 .2; 0 1 0; 0 0 1]*graphPose;
            %plot(graphPose(1,:),graphPose(2,:),'g');
            %axis([-2 2 -2 2]);
            %grid on;
            %hold off

            [E, J] = getJacobian(theMap,thePose,modelPts);

            %plot(E);
            %plot(thePose.y,thePose.x,'*');
            %hold on;
            %axis([-2, 2, -2, 2]);

            %disp(thePose.getPoseVec());
            thePose = pose(thePose.getPoseVec() - transpose(J)); 
            pause(.01);
        end   


        function makeMap()
            global theMap;

            x1s = [0, 1.2, 1.2];
            y1s = [0, 0, 0];

            y2s = [1.2, 0, 1.2];
            x2s = [0, 0, 1.2];

            lines_p1 = [x1s;y1s ];
            lines_p2 = [x2s;y2s ];

            gain = 0.01;
            errThresh = 0.001;
            gradThresh = 0.0005; 

            theMap = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);

        end
    end
end
    
    