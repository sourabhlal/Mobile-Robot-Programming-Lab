%main function for Lab 5
function testTrajectory(robot)
    obj = figure8ReferenceControl(.5,.5,0);
    
    totalTime = getTrajectoryDuration(obj);
   
    time = tic;
    while toc(time) < totalTime
        [V, w] = computeControl(obj, toc(time));
        rm = robotModel(.237);
        [vl, vr] = VwTovlvr(rm,V,w);
        robot.sendVelocity(vl,vr);
        pause(.01);
    end
    
    
    maxTime = getTrajectoryDuration(obj);
    trajectory = robotTrajectory(1000);
    calcPose(trajectory,obj);
    for i = 1:1000
        pose = getPoseAtTime(trajectory,obj, i/1000*maxTime);
        x(i) = pose(1,1);
        y(i) = pose(2,1);
        th(i) = pose(3,1);
        %disp([x(i), y(i), th(i)]);
    end
    plot(y,x);
    
    robot.sendVelocity(0,0);
    %clear classes;
end