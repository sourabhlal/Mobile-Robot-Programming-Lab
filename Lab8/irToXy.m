function [ x y th] = irToXy( i, r )
%irToXy finds position and bearing of a range pixel endpoint
% Finds the position and bearing of the endpoint of a range pixel in the plane

th = i*pi/180;
x = r*cos(th);
y = r*sin(th);


end