function uref = trapezoidalVelocityProfile( t , amax, vmax,dist, sgn)
tRamp = vmax/amax;
tf = dist;


if t < tRamp
    uref = amax*t; 
elseif 0 < (tf - t) && (tf - t) < tRamp
    uref = amax*(tf-t);
elseif tRamp < t && t < tf-tRamp
    uref = vmax;    
else
    uref = 0;
end


uref = sgn*uref;

end