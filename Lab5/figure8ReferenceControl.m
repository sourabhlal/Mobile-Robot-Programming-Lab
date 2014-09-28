classdef figure8ReferenceControl < handle
    
    properties
        tPause;
        Ks;
        Kv;
    end
    
    methods
        %class constructor
        function obj = figure8ReferenceControl(Ks,Kv,tPause)
           obj.tPause = tPause;
           obj.Ks = Ks;
           obj.Kv = Kv;
        end
        
        %assumes timeNow starts at zero
        %ignore tPause for now
        function [V, w] = computeControl (obj, timeNow)
            vr = .3*obj.Kv + .14125*obj.Kv/obj.Ks*sin(timeNow*obj.Kv/(2*obj.Ks)); 
            vl = .3*obj.Kv - .14125*obj.Kv/obj.Ks*sin(timeNow*obj.Kv/(2*obj.Ks)); 
            rm = robotModel(.237);
            [V,w] = vlvrToVw(rm,vl, vr);
        end
        
        function duration = getTrajectoryDuration(obj)
            duration = 2*obj.tPause + 4*pi*obj.Ks/obj.Kv;
        end  
    end
    
end

