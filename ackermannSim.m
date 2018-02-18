function [globalState] = ackermannSim(desiredControlVector, globalState, dt,mass, carLength)
    
    %globalState = [x, y, theta, dx, dy, T, phi) 
    %desiredControlVector = [T, phi]
    globalState(6) = globalState(6) + 0.98 * (desiredControlVector(1) - globalState(6)); %T, x, T
    globalState(7) = globalState(7) + 0.5 * (desiredControlVector(2) - globalState(7)); %phi, Y, phi
    
    r = carLength / tan(globalState(7)); %turn radius (denom is phi)
    
    theta = globalState(3);
    
    R = [cos(theta) , -sin(theta); sin(theta) , cos(theta)]; %rotation matrix from inertial coordinate frame to body coordinate frame
    
    if (abs(r) < 1.1)
      r = 1.1;
    end
    
    if (globalState(6) < -20)
      globalState(6) = -20;
    end
    
    if (globalState(6) > 20)
      globalState(6) = 20;
    end
      
    bv = R' * globalState(4:5, 1);
      
    bay = bv(1)^2 / r;
    
    globalState(3) = globalState(3) + sqrt(bay * r) * dt; %angluar vel update
    
    acci = R * ([globalState(6) / mass; 0] + [0 ; bay])
    
    globalState(1:2, 1) = globalState(1:2, 1) + dt * globalState(4:5, 1) + 0.5 * dt * dt * acci;
    globalState(4:5, 1) = globalState(4:5, 1) + acci * dt;
  
end
  
 
  












