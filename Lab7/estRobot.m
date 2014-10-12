classdef estRobot < handle
    
    properties(Access = public)
       x;
       y;
       th;
       index;
       prevDistLeft;
       prevDistRight;
       robot;
       ISR;
    end
    
    methods(Access = public)
    
        %constructor
        function obj = estRobot(initialX, initialY, initialTH, r)
            obj.x(1) = initialX;
            obj.y(1) = initialY;
            
            obj.th = initialTH; 
            
            obj.index = 1;
            obj.robot = r;
            obj.prevDistLeft = r.encoders.data.left;
            obj.prevDistRight = r.encoders.data.right;
            obj.ISR = event.listener(r.encoders,'OnMessageReceived',@neatoEncoderEventListener);
        end 
      
    end
end

