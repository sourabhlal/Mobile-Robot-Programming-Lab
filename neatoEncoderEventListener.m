%Created for Lab 3
function neatoEncoderEventListener(handle,event)
%disp('entering ISR');
global prevTime;
global prevDistLeft;
global prevDistRight;
global x;
global y;
global th;
global robot;
global index;

width = .235; %in meters

currTime = event.data.header.stamp.secs +(event.data.header.stamp.nsecs/1000000000);
dt = currTime - prevTime;
prevTime = currTime;

currDistRight = robot.encoders.data.right;
dSr = (currDistRight - prevDistRight)/1000; %convert to meters
prevDistRight = currDistRight;

currDistLeft = robot.encoders.data.left;
dSl = (currDistLeft - prevDistLeft)/1000; %convert to meters
prevDistLeft = currDistLeft;

dth = atan((dSr - dSl)/width);
dS = (dSr+dSl)/2;

th = th + dth/2;
x(index+1) = x(index) + (dS/dt)*cos(th)*dt;
y(index+1) = y(index) + (dS/dt)*sin(th)*dt;
th = th + dth/2;

index = index+1;

%disp([x y]);
plot(x,y);

end