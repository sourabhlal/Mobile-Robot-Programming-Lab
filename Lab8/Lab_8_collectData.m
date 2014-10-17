function Lab_8_collectData(robot)
    robot.startLaser();
    pause(2);
    laserRanges = zeros(10,360);
    for i = 1:10
        currentData = robot.laser.data.ranges;
        for j = 1:360
            laserRanges(i,j) = currentData(j);
        end
        pause(10);
        beep;
    end
    pause(2);
    robot.stopLaser();
    save('objectData.mat','laserRanges')
    beep;
    pause(1);
    beep;
    pause(1);
    beep;
end