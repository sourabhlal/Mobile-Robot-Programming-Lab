classdef refRobot < handle
    properties
        x;
        y;
        t;
        index;
        
        xprev;
        yprev;
        tprev;
        
        numCurves;
        
        transform;
        
    end
    
    methods
        function obj = refRobot()
            obj.index = 1;
            obj.x(obj.index) = 0;
            obj.y(obj.index) = 0;
            obj.t(obj.index) = 0;
            
            obj.xprev = 0;
            obj.yprev = 0;
            obj.tprev = 0;
            
            obj.numCurves = 0;
            
            obj.transform = [1,0,0;0,1,0;0,0,1]; 
        end
        
        function [] = addCurve(obj, newx,newy,newt)
            newCurve = cubicSpiral.planTrajectory(newx,newy,newt,1);
            currentCurveValue = cubicSpiral.planTrajectory(obj.x(obj.index),obj.y(obj.index),obj.t(obj.index),1);
            if currentCurveValue ~= 1
                obj.transform = obj.transform * [cos(obj.tprev),-sin(obj.tprev),obj.xprev; sin(obj.tprev), cos(obj.tprev), obj.yprev;0,0,1];
            end
            for i = 1: length(newCurve.poseArray)
               pX = newCurve.poseArray(1,i);
               pY = newCurve.poseArray(2,i);      
               xy = obj.transform*[pX;pY;1];
               
            end
            obj.predictedX = [obj.predictedX ; obj.xy(1,:)];
            obj.predictedY = [obj.predictedY ; obj.xy(2,:)];
            obj.xprev = obj.x(obj.index);
            obj.yprev = obj.y(obj.index);
            obj.tprev = obj.t(obj.index);
            obj.index = obj.index+1;
        end   
    end
end