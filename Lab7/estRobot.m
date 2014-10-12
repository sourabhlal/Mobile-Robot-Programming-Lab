classdef estRobot < handle
    
    properties(Access = public)
       x;
       y;
       th;
       prevDistLeft;
       prevDistRight;
       robot;
       ISR;
    end
    
    methods(Access = public)
    
        %constructor
        function obj = estRobot(initialX, initialY, initialTH, r)
            obj.x = initialX;
            obj.y = initialY;
            obj.th = initialTH; 
            obj.robot = r;
            obj.prevDistLeft = r.encoders.data.left;
            obj.prevDistRight = r.encoders.data.right;
            obj.ISR = event.listener(r.encoders,'OnMessageReceived',@neatoEncoderEventListener);
        end
        
        %this function doesn't work for some reason, but its not needed
%         function [X, Y, TH] = getCurrPose(obj)
%             X = obj.x;
%             Y = obj.y;
%             TH = obj.th;
%         end        
    end
    
end

