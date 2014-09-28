classdef robotTrajectory < handle
    properties
        numSamples;
        pose;
    end
    
    methods
        %constructor
        function obj = robotTrajectory(numSamples);
            obj.numSamples = numSamples;
            obj.pose = zeros(3,numSamples);
        end
        
        %obj is an instant of a ReferenceControl
        function pose = getPoseAtTime(trajectory, obj, t)
            maxTime = getTrajectoryDuration(obj);
            index = floor(trajectory.numSamples*t/maxTime);
            index = max(1,index);
            
            x = trajectory.pose(1,index);
            y = trajectory.pose(2,index);
            th = trajectory.pose(3,index);
            pose = [x ; y ; th];
        end
        
        function v = calcPose(trajectory,obj)
            x=0;
            y=0;
            th=0;
            
            maxTime = getTrajectoryDuration(obj);
            dt = maxTime/trajectory.numSamples;
            
            for index = 1:trajectory.numSamples                
                currTime = dt*index;
                [V,w] = computeControl(obj, currTime);
                th = th + w*dt/2;
                x = x + V*cos(th)*dt;
                y = y + V*sin(th)*dt;
                th = th + w*dt/2;
                
                trajectory.pose(1,index) = x;
                trajectory.pose(2,index) = y;
                trajectory.pose(3,index) = th;               
            end      
            v=0;
        end
    end
    
end

