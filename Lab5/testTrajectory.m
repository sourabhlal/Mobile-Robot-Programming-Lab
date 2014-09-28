%main function for Lab 5
function testTrajectory(robot)
    rm = robotModel(20);
    obj = figure8ReferenceControl(.5,.5,0);
    
    totalTime = getTrajectoryDuration(obj);
    time = tic;
    while toc(time) < totalTime
        [V, w] = computeControl(obj, toc(time));
        [vl, vr] = VwTovlvr(rm,V,w);
        robot.sendVelocity(vl,vr);
        pause(.01);
    end
    
    maxTime = getTrajectoryDuration(obj);
    trajectory = robotTrajectory(1000);
    for i = 1:1000
        pose = getPoseAtTime(trajectory, i/1000*maxTime);
        x(i) = pose(1,1);
        y(i) = pose(2,1);
        disp([x(i), y(i)])
        th(i) = pose(3,1);
    end
    plot(y,x);
    
    robot.sendVelocity(0,0);
    %clear classes;
end