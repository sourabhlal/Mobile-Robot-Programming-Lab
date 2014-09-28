classdef robotTrajectory
    properties
        numSamples;
    end
    
    methods
        %constructor
        function obj = robotTrajectory(numSamples)
            obj.numSamples = numSamples;
        end
        
        %obj is an instant of a ReferenceControl
        function pose = getPoseAtTime(obj, t)
            x=0;
            y=0;
            th=0;
            time = tic;
            prevTime = time;
            while time < t
                currTime = toc(time);
                dt = currTime - prevTime;
                prevTime = currTime;
                
                [V,w] = computeControl (obj, currTime);
                th = th + w*dt/2;
                x = x + V*cos(th)*dt;
                y = y + V*sin(th)*dt;
                th = th + w*dt/2;
            end
            pose = [x ; y ; th];
            
        end
    end
    
end

