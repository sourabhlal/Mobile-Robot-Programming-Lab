classdef trapezoidalStepReferenceControl < handle
    
    properties
        tPause;
        amax; 
        vmax;
        dist; 
        sgn;
    end
    
    methods
        %class constructor
        function obj = trapezoidalStepReferenceControl(amax, vmax,dist, sgn, tPause)
           obj.tPause = tPause;
           obj.amax = amax;
           obj.vmax = vmax;
           obj.dist = dist;
           obj.sgn = sgn;
        end
        
        function [V, w] = computeControl (obj, timeNow)
            tRamp = obj.vmax/obj.amax;
            tf = (obj.dist/obj.vmax) + tRamp;

            if timeNow < tRamp
                uref = obj.amax*timeNow; 
            elseif tRamp < timeNow && timeNow < tf-tRamp
                uref = obj.vmax;
            elseif timeNow < tf && timeNow > tf-tRamp
                uref = obj.amax*(tf-timeNow);    
            else
                uref = 0;
            end
            w = 0;
            V = obj.sgn*uref;

        end
        
        function duration = getTrajectoryDuration(obj)
            tRamp = obj.vmax/obj.amax;
            tf = (obj.dist/obj.vmax) + tRamp;
            duration = 2*obj.tPause + tf;
        end  
    end
    
end

