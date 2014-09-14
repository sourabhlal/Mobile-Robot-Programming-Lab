function out = Lab2_3_2(robot)
%plot the location of the cloest object relative to the robot frame

MAX_RANGE = 2;
robot.startLaser();
pause(2);
while 1==1
    shortestRange = MAX_RANGE;
    shortestIndex = 1;
    laserRanges = robot.laser.data.ranges;
    for i = 1:360
        if laserRanges(i) > .1 && laserRanges(i) < shortestRange
            shortestIndex = i;
            shortestRange = laserRanges(i);
        end
    end
    %do not plot if no object found
    if shortestRange < MAX_RANGE
        [x,y,th] = irToXy(shortestIndex,shortestRange);
        plot(-1*y,x,'x');
        axis([-2 2 -2 2]);
        %disp(shortestRange);
        Lab2_track(th,shortestRange,robot);
    else
        robot.sendVelocity(0,0);
    end
    pause(.1);
end

end
