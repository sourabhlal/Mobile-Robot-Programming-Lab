function Lab7(robot)
    global RobotEstimate;
    RobotEstimate = estRobot(0,0,0,robot);
    
    RobotReference = refRobot();
        
    control = mrplSystem(RobotReference, RobotEstimate);
    
    curve = cubicSpiral.planTrajectory(.25,.25,0,1);
    planVelocities(curve,.25);
    addCurve(RobotReference,curve);
    executeTrajectory(control,.25,.25,0,robot,5);
    
    curve = cubicSpiral.planTrajectory(-.5,-.5,-pi/2,1);
    planVelocities(curve,.25);
    addCurve(RobotReference,curve);
    executeTrajectory(control,-.5,-.5,-pi/2,robot,5);
    
    curve = cubicSpiral.planTrajectory(-.25,.25,pi/2,1);
    planVelocities(curve,.25);
    addCurve(RobotReference,curve);
    executeTrajectory(control,-.25,.25,pi/2,robot,5);
    
    
end

