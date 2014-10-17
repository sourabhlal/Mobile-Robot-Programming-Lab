classdef rangeImage < handle %rangeImage Stores a 1D range image and provides related services. 
 
 properties(Constant) 
     maxUsefulRange = 2.0; 
     minUsefulRange = 0.05; 
     maxRangeForTarget = 1.0; 
 end 
 
 properties(Access = public) 
     rArray = []; 
     tArray = []; 
     xArray = []; 
     yArray = []; 
     numPix; 
 end 
 
 methods(Access = public)
     function obj = rangeImage(ranges,skip,cleanFlag) 
         % Constructs a rangeImage for the supplied data. 
         % Converts the data to rectangular coordinates 
         if(nargin == 3) 
             n=0; 
             for i=1:skip:length(ranges) 
                 n = n + 1; 
                 obj.rArray(n) = ranges(i); 
                 obj.tArray(n) = (i-1)*(pi/180); 
                 obj.xArray(n) = ranges(i)*cos(obj.tArray(n)); 
                 obj.yArray(n) = ranges(i)*sin(obj.tArray(n)); 
             end
             obj.numPix = n; 
             if cleanFlag; 
                 obj.removeBadPoints(); 
             end; 
         end
     end
     
     function removeBadPoints(obj) 
         % takes all points above and below two range thresholds 
         % out of the arrays. This is a convenience but the result 
         % should not be used by any routine that expects the points 
         % to be equally separated in angle. The operation is done 
         % inline and removed data is deleted. 
 
         v = find(obj.rArray > rangeImage.minUsefulrange & obj.rArray < rangeImage.maxUsefulrange);
         obj.rArray = obj.rArray(v);
         obj.tArray = obj.tArray(v);
         obj.xArray = obj.xArray(v);
         obj.yArray = obj.yArray(v);
     end
     
     function plotRvsTh(obj, maxRange) 
         % plot the range image after removing all points exceeding 
         % maxRange
         R = obj.rArray;
         Th = obj.tArray;
         v = find(R < maxRange);
         R = R(v);
         Th = Th(v);
         
         figure;
         plot(Th,R);
         title('R vs Th');
 
     end
     
     function plotXvsY(obj, maxRange)
         % plot the range image after removing all points exceeding 
         % maxRange 
 
         v = find(R < maxRange);
         X = obj.xArray(v);
         Y = obj.yArray(v);
         figure;
         plot(Y,X);
         title('X vs Y')
 
     end
     
     function [err, num, th] = findLineCandidate(obj,middle,maxLen) 
         % Find the longest sequence of pixels centered at pixel
         % “middle” whose endpoints are separated by a length less 
         % than the provided maximum. Return the line fit error, the 
         % number of pixels participating, and the angle of 
         % the line relative to the sensor. 
 
         lenSoFar = 0;
         num = 1;
         lineCandidate = middle;
         
         while lenSoFar <= halfLen
             left = indexAdd(obj,middle,-1);
             right = indexAdd(obj,middle,1);
             dist = sqrt((obj.xArray(left)-obj.xArray(right))^2 + (obj.yArray(left)-obj.yArray(right))^2);
             if dist <= maxLen
                 num = num + 2;
                 lineCandidate = cat(2,left,lineCandidate,right);
             end
         end
         startX = obj.xArray(lineCandidate(1));
         startY = obj.yArray(lineCandidate(1));
         endX = obj.xArray(lineCandidate(end));
         endY = obj.yArray(lineCandidate(end));
         unitX = (endX-startX)/num;
         unitY = (endY-startY)/num;
         for i = 1:1:num
             pX = obj.xArray(lineCandidate(i));
             pY = obj.yArray(lineCandidate(i));
             errX = pX - (startX + i*unitX);
             errY = pY - (startY + i*unitY);
             err = err + errX^2 + errY^2;
         end
         
         th = atan2((endY-startY),(endX-startX));
                 
     end 
     
     function num = numPixels(obj)
         num = obj.numPix; 
     end
     
     % Modulo arithmetic on nonnegative integers. MATLABs choice to % have matrix indices start at 1 has implications for 
     % calculations relating to the position of a number in the 
     % matrix. In particular, if you want an operation defined on 
     % the sequence of numbers 1 2 3 4 that wraps around, the 
     % traditional modulus operations will not work correctly. 
     % The trick is to convert the index to 0 1 2 3 4, do the 
 	 % math, and convert back. 
     function out = inc(obj,in) 
         % increment with wraparound over natural numbers 
         out = indexAdd(obj,in,1); 
     end 
     
     function out = dec(obj,in) 
         % decrement with wraparound over natural numbers 
         out = indexAdd(obj,in,-1); 
     end 
     
     function out = indexAdd(obj,a,b) 
         % add with wraparound over natural numbers. First number 
         % “a” is "natural" meaning it >=1. Second number is signed. 
         % Convert a to 0:3 and add b (which is already 0:3). 
         % Convert the result back by adding 1. 
         out = mod((a-1)+b,obj.numPix)+1; 
     end
 end
end


