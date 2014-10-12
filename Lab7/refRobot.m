classdef refRobot
    properties
        x;
        y;
        t;
        curve;
        predictedX;
        predictedY;
        predictedT;
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
            obj.predictedT = [];
            obj.xy = [];
            obj.transform = [1,0,0;0,1,0;0,0,1];
        end
        
        function [] = addCurve(obj, newCurve)
            obj.curve(end+1) = newCurve;
            
            %obj.curve.planVelocities(.25);
            for i = 1: length(obj.curve(1).poseArray)
               pX(i) = newCurve.poseArray(1,i);
               pY(i) = newCurve.poseArray(2,i);
               pT(i) = newCurve.poseArray(3,i);
            end
            
            obj.predictedX = [obj.predictedX ; pX];
            obj.predictedY = [obj.predictedY ; pY];
            obj.predictedT = [obj.predictedT ; pT];
            
            currentCurveValue = length(obj.curve);
            if currentCurveValue ~= 1
                %obj.transform                
                %xy0 = [1,0,0.25;0,1,0.25;0,0,1]*[xPredict(i);yPredict(i);1];
                %xy0 = [0,1,-0.25;-1,0,-0.25;0,0,1]*[xPredict(i);yPredict(i);1];
                
                
            end
            obj.xy = obj.transform*[obj.predictedX(1);obj.predictedY(i);1];           
        end   
    end
end