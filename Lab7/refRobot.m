classdef refRobot
    properties
        x;
        y;
        t;
        xprev;
        yprev;
        tprev;
        curve;
        predictedX;
        predictedY;
        xy;
        transform;
    end
    
    methods
        function obj = refRobot(xFin, yFin, tFin)
            obj.x = xFin;
            obj.y = yFin;
            obj.t = tFin;
            obj.curve = [];
            obj.predictedX = [];
            obj.predictedY = [];
            obj.xy = [];
            obj.transform = [1,0,0;0,1,0;0,0,1];
        end
        
        function [] = addCurve(obj, newCurve, newx,newy,newt)
            obj.x = newx;
            obj.y = newy;
            obj.t = newt;
            obj.curve(end+1) = newCurve;
            currentCurveValue = length(obj.curve);
            if currentCurveValue ~= 1
                obj.transform = obj.transform * [cos(obj.tprev),-sin(obj.tprev),obj.xprev; sin(obj.tprev), cos(obj.tprev), obj.yprev;0,0,1];
            end
            for i = 1: length(newCurve.poseArray)
               pX(i) = newCurve.poseArray(1,i);
               pY(i) = newCurve.poseArray(2,i);      
               obj.xy(i) = obj.transform*[obj.pX(i);obj.pY(i);1];
            end
            obj.predictedX = [obj.predictedX ; obj.xy(1,:)];
            obj.predictedY = [obj.predictedY ; obj.xy(2,:)];
            obj.xprev = obj.x;
            obj.yprev = obj.y;
            obj.tprev = obj.t;
        end   
    end
end