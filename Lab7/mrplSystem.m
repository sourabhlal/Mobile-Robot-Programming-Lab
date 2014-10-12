classdef mrplSystem < handle
    
    properties(Access = public)
        RobotReference;
        RobotEstimate;
    end
    
    methods(Access = public)
         %constructor
        function obj = mrplSystem(ref, est)
            obj.RobotReference = ref;
            obj.RobotEstimate = est;
        end
        
        
        function executeTrajectory(obj,xf,yf,thf,robot,pauseTime)
            kpx = 1.5;
            kdx = .1;

            kppsi = .1;   
            kdpsi = .01;

            kpth = 3;   
            kdth = .3;

            width = .235;

            refPose = getPoseAtTime(obj.RobotReference);
            xi = refPose(1,1) - obj.RobotEstimate.x;
            yi = refPose(1,2) - obj.RobotEstimate.y;
            thi = refPose(1,3) - obj.RobotEstimate.th;

            curve = cubicSpiral.planTrajectory(xf-xi,yf-yi,thf-thi,1);
            planVelocities(curve,.25);

            time = tic();
            completionTime = curve.timeArray(end);
            index=1;
            th=0;
            while toc(time) < (completionTime + pauseTime)
                currTime = toc(time);
                if currTime < completionTime
                    V = getVAtTime(curve,currTime);
                    w = getwAtTime(curve,currTime);
                    targetPose = getPoseAtTime(curve,currTime);
                else
                    V = 0;
                    w = 0;
                    targetPose = getPoseAtTime(curve,completionTime);
                end

                Twb = inv([cos(th) -sin(th) xActual(index+1) ; sin(th) cos(th) yActual(index+1) ; 0 0 1]);
                raw = [targetPose(1,1) ; targetPose(2,1) ; 1];
                rab = Twb*raw;
                xError(index+1) = rab(1,1);
                yError(index+1) = rab(2,1);

                %calc angle between robot and predicted pose
                Taw = [cos(targetPose(3,1)) -sin(targetPose(3,1)) targetPose(1,1) ; sin(targetPose(3,1)) cos(targetPose(3,1)) targetPose(2,1) ; 0 0 1];
                Tab = Twb*Taw;
                thError(index+1) = atan2(Tab(2,1),Tab(1,1));

                psiError(index+1) = atan2(yError(index+1),xError(index+1));

                rabx_dt = (xError(index+1)-xError(index))*dt; 
                dpsi_dt = (psiError(index+1)-psiError(index))*dt;
                dth_dt = (thError(index+1)-thError(index))*dt;

                mag = 10*sqrt(xError(index+1)*xError(index+1)+yError(index+1)*yError(index+1));
                kppsi_mag = min(kppsi/mag,3*kppsi);
                kdpsi_mag = min(kdpsi/mag,3*kdpsi);

                V = V + kpx*xError(index+1) + kdx*rabx_dt;
                if xError(index+1) > 0 
                   w = w + kppsi_mag*psiError(index+1) + kdpsi_mag*dpsi_dt + kpth*thError(index+1) + kdth*dth_dt;
                else
                    w = w + kpth*thError(index+1) + kdth*dth_dt;    
                end

                vr = V + (width/2)*w;
                vl = V - (width/2)*w; 

                vl  = min(.3,vl);
                vr  = min(.3,vr);
                vl  = max(-.3,vl);
                vr  = max(-.3,vr);

                robot.sendVelocity(vl,vr);

                index = index+1;
                
                pause(.01);
            end
        end
    end
end
