function neatoEncoderEventListener(handle,event)
    global RobotEstimate;
   
    %disp('entering isr');
    width = .235; %in meters

    currDistRight = RobotEstimate.robot.encoders.data.right;
    dSr = (currDistRight - RobotEstimate.prevDistRight)/1000; %convert to meters
    RobotEstimate.prevDistRight = currDistRight;

    currDistLeft = RobotEstimate.robot.encoders.data.left;
    dSl = (currDistLeft - RobotEstimate.prevDistLeft)/1000; %convert to meters
    RobotEstimate.prevDistLeft = currDistLeft;

    dth = (dSr - dSl)/width;
    dS = (dSr+dSl)/2;

    RobotEstimate.th = RobotEstimate.th + dth/2;
    RobotEstimate.x = RobotEstimate.x + (dS)*cos(RobotEstimate.th);
    RobotEstimate.y = RobotEstimate.y + (dS)*sin(RobotEstimate.th);
    RobotEstimate.th = RobotEstimate.th + dth/2;

end
