function out = Lab2_track(th,r,robot)

    if th > pi
        th = th-2*pi;
    end    
    
    if r > .5;
        v = .1 +.075 * r; 
        disp(v);
        %v = r/10;
        w = .75 + .2 * abs(th); 
    elseif r < .4
        v = -(3/4) * (.4-r);
        w = -.75 -.2* abs(th); 
    else
        v = 0;
        w = 1; 
    end
    
    curve = th;
    angleVel = curve * v;
    vr = v + w/2*angleVel;
    vl = v - w/2*angleVel;

    %disp([vl vr]);
    if vl > 0
       vl = min(.3,vl); 
    else
       vl = max(-.3,vl);
    end
    
    if vr > 0
       vr = min(.3,vr); 
    else
       vr = max(-.3,vr);
    end
    
    robot.sendVelocity(vl,vr);

out = 0;
end