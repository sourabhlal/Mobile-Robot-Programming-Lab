function Lab9(robot)
     global thePose;
     thePose = pose(.5,.5,0);
     figure(1);
    
     set(gcf,'doublebuffer','on');
     axis([-10 10 -10 10]);
 
     set(gcf,'KeyPressFcn','keydown=1;');
     key = '';
    
     vGain = 1.0;
     Vmax = 0.02*vGain;
     dV = 0.002*vGain;
  
     while(1==1)
      
             testLineMap(robot);
        
          key = get(gcf,'CurrentCharacter');
          %disp(key);
          if (strcmp(key,'w' ))
                %disp( 'up');
                robot.sendVelocity(Vmax,Vmax);
            elseif(strcmp(key, 's'))
                %disp('down');
                robot.sendVelocity( -Vmax, -Vmax);
            elseif(strcmp(key,'a' ))
                %disp('left');
                robot.sendVelocity(Vmax,Vmax+dV);
            elseif(strcmp(key, 'd'))
                %disp('right' );
                robot.sendVelocity(Vmax+dV,Vmax);
            elseif(strcmp(key,'p' ))
                %disp('stop');
                robot.sendVelocity(0.0,0.0);
          end;

         
         pause(.1);
     end
end