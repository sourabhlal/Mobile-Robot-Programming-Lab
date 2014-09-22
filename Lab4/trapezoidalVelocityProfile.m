function uref = trapezoidalVelocityProfile( t , amax, vmax,dist, sgn)
tRamp = vmax/amax;
tf = (dist/vmax) + tRamp;


if t < tRamp
    uref = amax*t; 
elseif tRamp < t && t < tf-tRamp
    uref = vmax;
elseif t < tf && t > tf-tRamp
    uref = amax*(tf-t);    
else
    uref = 0;
end

uref = sgn*uref;

end