function testLineMap()
    p1 = [-10,.1];
    p2 = [0,.1];
    p3 = [10,-.1];
    line1 = [p1,p2,p3];
    
    p4 = [.1,-10];
    p5 = [.1,0];
    p6 = [-.1,10];
    line2 = [p4,p5,p6];
    
    gain = 0.01;
    errThresh = 0.001;
    gradThresh = 0.0005; 

    lineMapLocalizer(line1,line2,gain,errThresh,gradThresh);
    
    poseIn = [5,5,0];
    
    [E, J] = getJacobian(obj,poseIn,modelPts);
end