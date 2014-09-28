%This class have static methods with constant values for W
%but when I did that nothing worked and I couldn't figure out why
classdef robotModel < handle
  
    properties
        W;%width
        W2;
    end
    
    methods
        %constructor
        function obj = robotModel(W)
            obj.W = W;
            obj.W2 = W/2;
        end
        
        function [V, w] = vlvrToVw(rm,vl, vr)
            V = (vr + vl)/2;
            w = (vr - vl)/rm.W;
        end
        
        function [vl, vr] = VwTovlvr(rm,V, w)
            vr = V + rm.W/2*w;
            vl = V - rm.W/2*w;
        end
    end
    
end

